# Development Workflow & Hardware Integration

This document outlines the standard practices for modifying the UR5 Digital Twin, synchronizing the codebase across multiple machines, and the roadmap for integrating physical hardware.

---

## 1. Branching Strategy

To protect the stability of the core simulation, **never commit experimental code directly to the `main` branch.**

* **`main`**: The stable, working version of the digital twin. It should always compile successfully and launch Gazebo without errors.
* **`feature/*`**: Used for developing new AI control schemes, adding new ROS 2 nodes, or tweaking MoveIt kinematics.
* **`hardware/*`**: Dedicated branches for introducing and testing physical controllers (e.g., the Platinum Maestro).

**Standard Workflow Commands:**
```bash
# 1. Ensure you are on main and up to date
git checkout main
git pull origin main

# 2. Create a new branch for your experiment
git checkout -b feature/new-ai-controller

# [Write your code, test in Gazebo...]

# 3. Add, commit, and push your branch
git add .
git commit -m "Added new dynamic torque array generator"
git push -u origin feature/new-ai-controller
```

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