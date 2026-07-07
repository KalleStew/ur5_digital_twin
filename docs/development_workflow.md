# Development Workflow

This document covers branching conventions, the **threaded Python Action Client architecture** used by the HIL research scripts, the procedure for safely swapping controllers via `/controller_manager/switch_controller`, and the hardware integration roadmap.

---

## 1. Branching Strategy

Never commit experimental code directly to `main`. The sim must always compile and launch on `main`.

| Branch | Purpose |
|---|---|
| `main` | Stable, always launchable |
| `feature/*` | New control schemes, new nodes, tuning experiments |
| `hardware/*` | Physical hardware interface development (Platinum Maestro) |

```bash
# Standard workflow
git checkout main && git pull origin main
git checkout -b feature/my-experiment

# ... write code, test in Gazebo ...

git add .
git commit -m "feat: implement adaptive torque ramp"
git push -u origin feature/my-experiment
```

---

## 2. Synchronizing Workstations (Mac ARM64 VM ↔ Windows WSL2)

**Start of session — always pull first:**
```bash
git fetch --all
git pull origin main
```

**End of session — always push even if WIP:**
```bash
git commit -am "WIP: tuning shoulder_lift gains"
git push
```

---

## 3. Threaded Python Action Client Architecture

All Python HIL scripts (`unified_control.py`, `conventional_cartesian_ik.py`) use a **two-thread pattern** to prevent executor deadlocks.

### The Problem: SingleThreadedExecutor Deadlock

`rclpy.spin()` blocks the calling thread indefinitely. If control logic (e.g., waiting on a service response from `/controller_manager/switch_controller`) runs on the same thread as `spin()`, the executor can never process the incoming service response — producing an infinite hang.

### The Solution: Main-thread spin + background-thread logic

```python
import rclpy
import threading
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64MultiArray

class HILController(Node):
    def __init__(self):
        super().__init__('hil_controller')

        # Publisher for raw torque injection
        self.torque_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_effort_controller/commands',
            10
        )

        # Client for controller switching
        self.switch_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

    def switch_to_effort(self):
        """Deactivate position controller, activate torque controller."""
        req = SwitchController.Request()
        req.activate_controllers   = ['forward_effort_controller']
        req.deactivate_controllers = ['ur_manipulator_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT

        # This call blocks — MUST run from the background thread, not main.
        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().ok

    def run_control_loop(self):
        """All blocking logic lives here. Called from a background thread."""
        self.get_logger().info("Switching to torque control...")
        if not self.switch_to_effort():
            self.get_logger().error("Controller switch failed!")
            return

        # 100 Hz torque injection loop
        rate = self.create_rate(100)
        while rclpy.ok():
            msg = Float64MultiArray()
            msg.data = [0.0, -5.0, 2.0, 0.0, 0.0, 0.0, 0.0]  # 7 joints
            self.torque_pub.publish(msg)
            rate.sleep()


def main():
    rclpy.init()
    node = HILController()

    # Background thread runs all blocking logic
    control_thread = threading.Thread(target=node.run_control_loop, daemon=True)
    control_thread.start()

    # Main thread runs the executor — never call blocking code here
    rclpy.spin(node)

    rclpy.shutdown()
    control_thread.join()
```

**Key rules:**
- `rclpy.spin(node)` — **main thread only**
- Service calls, `spin_until_future_complete`, HDF5 writes, `rate.sleep()` — **background thread only**
- Use `daemon=True` on the control thread so it exits cleanly when `rclpy.shutdown()` is called

---

## 4. Switching Controllers via `/controller_manager/switch_controller`

The controller manager enforces that only one controller may command a given hardware interface at a time. The `switch_controller` service atomically deactivates one set and activates another in a single physics step, so the arm never receives conflicting commands.

### Service Definition

```bash
# Inspect the service type
ros2 service type /controller_manager/switch_controller
# controller_manager_msgs/srv/SwitchController
```

### Switching from Position → Torque (Python)

```python
from controller_manager_msgs.srv import SwitchController

req = SwitchController.Request()
req.activate_controllers   = ['forward_effort_controller']
req.deactivate_controllers = ['ur_manipulator_controller']
req.strictness = SwitchController.Request.BEST_EFFORT
# BEST_EFFORT: switches even if one side fails gracefully.
# STRICT: aborts the entire switch if any controller fails.
```

### Switching Back from Torque → Position

```python
req.activate_controllers   = ['ur_manipulator_controller']
req.deactivate_controllers = ['forward_effort_controller']
req.strictness = SwitchController.Request.BEST_EFFORT
```

### CLI One-Liner (for manual testing without restarting Gazebo)

```bash
# Activate torque, deactivate position
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['forward_effort_controller'], \
    deactivate_controllers: ['ur_manipulator_controller'], \
    strictness: 2}"

# Restore position control
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['ur_manipulator_controller'], \
    deactivate_controllers: ['forward_effort_controller'], \
    strictness: 2}"
```

---

## 5. HDF5 Telemetry Logging (100 Hz)

The research scripts subscribe to `/joint_states` at 100 Hz and buffer readings into in-memory NumPy arrays, flushing to HDF5 at the end of a trial run.

```python
import h5py
import numpy as np
from sensor_msgs.msg import JointState

class TelemetryLogger:
    def __init__(self, max_samples=10000):
        self.positions = np.zeros((max_samples, 7))
        self.velocities = np.zeros((max_samples, 7))
        self.efforts   = np.zeros((max_samples, 7))
        self.timestamps = np.zeros(max_samples)
        self.idx = 0

    def record(self, msg: JointState):
        if self.idx >= len(self.positions):
            return
        self.positions[self.idx]  = msg.position[:7]
        self.velocities[self.idx] = msg.velocity[:7]
        self.efforts[self.idx]    = msg.effort[:7]
        self.timestamps[self.idx] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.idx += 1

    def save(self, filename: str):
        n = self.idx
        with h5py.File(filename, 'w') as f:
            f.create_dataset('positions',  data=self.positions[:n])
            f.create_dataset('velocities', data=self.velocities[:n])
            f.create_dataset('efforts',    data=self.efforts[:n])
            f.create_dataset('timestamps', data=self.timestamps[:n])
        print(f"Saved {n} samples to {filename}")
```

Log files are written to the working directory as `joint_space_log.h5`.

---

## 6. Hardware Integration Roadmap (Platinum Maestro)

When integrating the physical arm, follow this pipeline. The digital twin remains a simulation fallback on `main`.

**Step 1: Isolate on a hardware branch**
```bash
git checkout -b hardware/maestro-integration
```

**Step 2: Create a hardware URDF variant**

Do not modify `ur5.urdf`. Copy it to `ur5_maestro.urdf` and replace only the `<hardware>` block in the `<ros2_control>` section:

```xml
<!-- In ur5_maestro.urdf — replace the gz_ros2_control block: -->
<hardware>
    <plugin>maestro_hardware_interface/MaestroSystemHardware</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
</hardware>
```

**Step 3: Create a hardware launch file**

Create `ur5_moveit_config/launch/maestro_real.launch.py` that:
- Loads `ur5_maestro.urdf` via `MoveItConfigsBuilder`
- Starts `robot_state_publisher` and `controller_manager`
- Does **not** launch Gazebo

**Step 4: Validate and document**

Once the interface is validated, create `docs/hardware_deployment.md` covering USB routing, baud rates, and Maestro-specific commands.