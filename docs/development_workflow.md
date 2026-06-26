# Development Workflow & Hardware Integration

This document outlines standard practices for modifying the UR5 Digital Twin, the threaded Python Action Client architecture used in HIL scripts, the controller swap procedure, and the roadmap for integrating physical hardware.

---

## 1. Branching Strategy

Never commit experimental code directly to `main`.

* **`main`**: Stable, working version. Must compile and launch Gazebo without errors.
* **`feature/*`**: New AI control schemes, new ROS 2 nodes, MoveIt kinematics tuning.
* **`hardware/*`**: Physical controller integration (Platinum Maestro).
* **`migration/*`**: Platform migrations (e.g., `feature/ubuntu26-ll-migration`).

**Standard workflow:**
```bash
# Ensure main is up to date
git checkout main && git pull origin main

# Create feature branch
git checkout -b feature/new-torque-strategy

# After testing in Gazebo
git add . && git commit -m "feat: add adaptive torque ramp controller"
git push -u origin feature/new-torque-strategy
```

---

## 2. Threaded Python Action Client Architecture

All Python HIL scripts (`conventional_joint_space.py`, `conventional_cartesian_ik.py`) use a mandatory two-thread pattern to prevent `SingleThreadedExecutor` deadlocks.

### The Problem

The `FollowJointTrajectory` action server inside `ros2_control` sends a `GoalResponse` callback back to the caller. If `rclpy.spin()` and `send_goal_async()` both run on the same thread, the response callback can never be processed — the node deadlocks waiting for itself.

### The Solution

```python
class HILController(Node):
    def __init__(self):
        super().__init__('hil_controller',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Action client talks to ur_manipulator_controller
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/ur_manipulator_controller/follow_joint_trajectory')

        # Sensor subscriber (100 Hz telemetry recorder)
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.sensor_callback, 10)

        # CRITICAL: control logic runs in a background thread.
        # rclpy.spin() will hold the main thread (see __main__ block below).
        self.control_thread = threading.Thread(target=self.run_experiment)
        self.control_thread.daemon = True
        self.control_thread.start()

    def run_experiment(self):
        """Runs in background thread. Never call rclpy.spin() here."""
        self.action_client.wait_for_server()   # blocks background thread only
        # ... build goal, send, record telemetry ...

def main():
    rclpy.init()
    node = HILController()
    rclpy.spin(node)   # main thread processes callbacks
    node.destroy_node()
    rclpy.shutdown()
```

### Rules
- `rclpy.spin(node)` **must** run in `main()` on the main thread.
- All blocking calls (`wait_for_server`, `send_goal_async`, HDF5 writes) run inside `run_experiment()` on the daemon thread.
- Never call `rclpy.spin()` or `spin_until_future_complete()` from the background thread.

---

## 3. On-the-Fly Controller Swap (No Gazebo Restart)

The dual-brain architecture switches authority between position control and raw torque injection via the `/controller_manager/switch_controller` service. Gazebo does not need to be restarted.

### Swap: Position → Torque (activate `forward_effort_controller`)

```bash
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{
    activate_controllers: ['forward_effort_controller'],
    deactivate_controllers: ['ur_manipulator_controller'],
    strictness: 2
  }"
```

### Swap: Torque → Position (restore `ur_manipulator_controller`)

```bash
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{
    activate_controllers: ['ur_manipulator_controller'],
    deactivate_controllers: ['forward_effort_controller'],
    strictness: 2
  }"
```

**`strictness: 2`** (`STRICT`) means the service call fails entirely if any requested controller cannot be switched — preventing partial states where both controllers are active simultaneously (which would cause conflicting effort commands on the same joints).

### Verify current controller state

```bash
ros2 control list_controllers
```

Expected output after standard launch:
```
joint_state_broadcaster   [active]
ur_manipulator_controller [active]
forward_effort_controller [inactive]
```

---

## 4. HDF5 Telemetry Logging

HIL scripts write joint state telemetry to `.h5` files at 100 Hz using the `sensor_callback` registered on `/joint_states`. Files are written to `ur5_controller/src/`.

**Reading a log file:**
```python
import h5py
import numpy as np

with h5py.File('joint_space_log.h5', 'r') as f:
    timestamps = np.array(f['timestamps'])
    positions  = np.array(f['positions'])   # shape: (N, 7)
    velocities = np.array(f['velocities'])  # shape: (N, 7)
    efforts    = np.array(f['efforts'])     # shape: (N, 7)

print(f"{len(timestamps)} samples at ~{1/(timestamps[1]-timestamps[0]):.0f} Hz")
```

---

## 5. Synchronizing Workstations (Mac VM & Windows WSL2)

**At the start of your session:**
```bash
git fetch --all && git pull origin main
```

**At the end of your session:**
```bash
git commit -am "WIP: tuning shoulder_lift PID gains"
git push
```

---

## 6. Hardware Integration Roadmap (Platinum Maestro)

When integrating the `maestro_hardware_interface` for the physical arm:

### Step 1: Isolate on a hardware branch
```bash
git checkout -b hardware/maestro-integration
```

### Step 2: URDF hardware injection
Create `ur5_maestro.urdf` and replace the `gz_ros2_control` plugin:
```xml
<hardware>
    <plugin>maestro_hardware_interface/MaestroSystemHardware</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
</hardware>
```

### Step 3: Hardware launch file
Create `maestro_sim.launch.py` that boots `robot_state_publisher` and `controller_manager` using `ur5_maestro.urdf` without launching Gazebo.

### Step 4: Document in `docs/hardware_deployment.md`
Detail USB routing, baud rates, and Maestro terminal commands once validated.


---

## 2. Synchronizing Workstations (Mac VM & Windows)

Because this repository will be actively developed across two separate OS environments, use the following protocol to avoid merge conflicts:

**At the start of your work session:**
Always pull the latest changes before writing new code.
```bash
git fetch --all
git pull origin main
```

**At the end of your work session:**
Always push your current state, even if the feature isn't finished, so it is accessible from the other machine.
```bash
git commit -am "WIP: tuning MoveIt PID gains"
git push
```

---

## 3. Hardware Integration Roadmap (Platinum Maestro)

When the time comes to integrate the `maestro_hardware_interface` and connect the physical arm, follow this designated pipeline. This ensures the digital twin remains fully functional as a simulation fallback.

### Step 1: Isolate the Hardware Branch
Do not attempt hardware integration on `main`. Create a dedicated environment:
```bash
git checkout -b hardware/maestro-integration
```

### Step 2: URDF Hardware Injection
To use the physical arm instead of Gazebo, you will modify the `<ros2_control>` tags in `ur5.urdf`. 
* **Do not overwrite the Gazebo plugin.** * Instead, create a copy of the URDF named `ur5_maestro.urdf` and replace the `ign_ros2_control` plugin with your custom hardware interface:
  ```xml
  <hardware>
      <plugin>maestro_hardware_interface/MaestroSystemHardware</plugin>
      <param name="serial_port">/dev/ttyACM0</param>
  </hardware>
  ```

### Step 3: Hardware Launch File
Create a new launch file (e.g., `maestro_sim.launch.py`) that boots RViz and the `controller_manager` using `ur5_maestro.urdf` instead of launching the Gazebo physics engine. 

### Step 4: Revise the Deployment Guide
Once the physical interface is validated, a new file named `hardware_deployment.md` should be created in this `docs/` folder detailing the physical USB routing, baud rates, and Maestro-specific terminal commands.