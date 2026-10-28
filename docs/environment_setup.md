# Development Environment Setup

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, and `rmw_zenoh_cpp`. The dispatcher is `scripts/ur5`. It sources Jazzy, sets the middleware, and starts the Zenoh router.

## Table of Contents

1. Host platform
2. ROS 2 Jazzy
3. Simulation dependencies
4. Python numerics
5. Workspace
6. Build
7. VS Code and Git
8. Gazebo graphics
9. Controller configuration path
10. Environment verification
11. Initial tests
12. Troubleshooting
13. Quick setup commands
14. Final verification checklist

# 1. Host Platform Configuration

Use one host. Do not mix a WSL filesystem clone with a VM clone of the same `build/`.

## Option A: Windows Subsystem for Linux 2, WSL2

### Step 1: Install Ubuntu 22.04 in WSL2

From an elevated PowerShell:

```powershell
wsl --install -d Ubuntu-22.04
```

A 24.04 WSL image is also fine. The ROS install below is Jazzy, which wants Ubuntu 24.04. If the WSL guest is 22.04, use Option B or Option C for the ROS workspace, or upgrade the guest before installing Jazzy.

### Step 2: Confirm Ubuntu version

```bash
lsb_release -a
```

Expected: `Distributor ID: Ubuntu` and a release of 22.04 or 24.04.

### Step 3: Update Ubuntu

```bash
sudo apt update
sudo apt upgrade -y
```

### Step 4: Allocate enough memory to WSL2

Create `C:\Users\<you>\.wslconfig`:

```ini
[wsl2]
memory=8GB
processors=4
```

Then `wsl --shutdown` and reopen the guest. Gazebo and a torque slice want at least 4 vCPU. Fewer cores drop 10 ms samples.

### Step 5: Confirm Linux GUI support

```bash
echo "$DISPLAY"
```

A value such as `:0` means WSLg can open a window. Headless `ur5 sim` does not need it.

### Important WSL filesystem note

Clone the repository inside the Linux filesystem (`~/ros2_ws`), not under `/mnt/c`. Builds on the Windows mount are slow and file watchers miss saves.

## Option B: Dedicated Ubuntu Virtual Machine

Give the VM 4 vCPU and 8 GB RAM. Ubuntu 24.04 is the native match for Jazzy. Install the guest additions your hypervisor documents, then continue at section 2.

### Apple Silicon ARM64

An ARM64 Ubuntu 24.04 VM is a supported host. Gazebo's default renderer often fails there. Use the Ogre 1 renderer:

```bash
export GZ_SIM_RENDER_ENGINE=ogre
```

`ur5 simgui` reads that variable and defaults to `ogre` when it is unset. `ur5 sim` is headless and does not open a window. Confirm the machine with `uname -m` (`aarch64`).

## Option C: Native Ubuntu 24.04

```bash
lsb_release -ds
```

Expected: `Ubuntu 24.04` or a later 24.04 point release. Continue at section 2.

# 2. ROS 2 Jazzy Installation

## Step 1: Configure locale

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Expected: `locale` prints `LANG=en_US.UTF-8`.

## Step 2: Enable the Universe repository

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

## Step 3: Add the ROS 2 GPG key

```bash
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

## Step 4: Add the ROS 2 repository

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Step 5: Update apt

```bash
sudo apt update
```

Expected: no `NO_PUBKEY` line. If there is one, repeat step 3.

## Step 6: Install ROS 2 Jazzy Desktop

```bash
sudo apt install -y ros-jazzy-desktop
```

## Step 7: Source ROS 2 automatically

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

Expected: `echo $ROS_DISTRO` prints `jazzy`.

# 3. System and Simulation Dependencies

The packages are also named in the three `package.xml` files. `rosdep` installs them in section 6. This section is the explicit list when you are debugging a missing binary.

## Step 1: Install build tools

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential
```

## Step 2: Install MoveIt 2 and ros2_control

```bash
sudo apt install -y ros-jazzy-moveit ros-jazzy-moveit-configs-utils ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller ros-jazzy-effort-controllers
```

## Step 3: Install ROS-Gazebo integration packages

```bash
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control
```

## Step 4: Install robot description and visualization tools

```bash
sudo apt install -y ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-xacro
```

## Step 5: Install and run the Zenoh router

```bash
sudo apt install -y ros-jazzy-rmw-zenoh-cpp
```

`ur5` sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and runs `ros2 run rmw_zenoh_cpp rmw_zenohd` when the router is down. Do not export a different RMW.

## Step 6: Numeric and logging packages

```bash
sudo apt install -y python3-numpy python3-h5py python3-yaml ros-jazzy-python-orocos-kdl-vendor python3-pytest
```

Expected:

```bash
python3 -c "import numpy, h5py, yaml, PyKDL; print('ok')"
```

prints `ok`.

# 4. Python Numerical Package Compatibility

Jazzy's desktop install uses the Ubuntu Python 3.12 packages. Prefer those over a pip overlay.

## Recommended fix

If `import numpy` fails or ABI-mismatches h5py:

```bash
sudo apt install -y --reinstall python3-numpy python3-h5py
```

## Alternative fix

A virtualenv that hides `/usr/lib/python3/dist-packages` will break `rclpy`. Do not `pip install` numpy into the ROS interpreter. If you already did, remove that pip copy so the apt package is the one on `sys.path`.

## Important Python warning

`ur5 test --unit` imports PyKDL and h5py from the system site. A conda `base` activation that puts another Python first will fail those imports. Open the shell without conda, or `conda deactivate`, before `ur5 build`.

# 5. Workspace and Repository Setup

## Step 1: Create the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Step 2: Clone the project repository

```bash
cd ~/ros2_ws/src
git clone <repository-url> ur5_digital_twin
cd ur5_digital_twin
git switch refactor_v1
```

## Step 3: Inspect package structure

```bash
ls ~/ros2_ws/src/ur5_digital_twin
```

Expected directories: `ur5_description`, `ur5_moveit_config`, `ur5_controller`, `scripts`, `docs`.

# 6. Build and Source the Workspace

## Step 1: Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

`rosdep init` fails with "already exists" if you have done this before. That is fine. Run `rosdep update` anyway.

## Step 2: Install package dependencies

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
```

rosdep reads `ur5_description/package.xml`, `ur5_moveit_config/package.xml`, and `ur5_controller/package.xml`.

## Step 3: Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

Shortcut, after `scripts/ur5` is on `PATH` (section 6 is enough to call it by path):

```bash
~/ros2_ws/src/ur5_digital_twin/scripts/ur5 build
```

A plain `colcon build` (no `--symlink-install`) also succeeds. Both put the Python modules in `install/ur5_controller/lib/ur5_controller/`.

## Step 4: Source the local workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add that line to `~/.bashrc` after the Jazzy line. `ur5` sources both itself, so a new terminal can call `ur5` by path without a manual source.

# 7. VS Code and Git Source Control Setup

## Step 1: Install the VS Code WSL extension

On Windows, install "WSL" from the VS Code extension marketplace. On a VM or native Ubuntu, install VS Code or Cursor on the guest and skip the WSL extension.

## Step 2: Open the repository correctly

Open `~/ros2_ws/src/ur5_digital_twin`, not `~/ros2_ws`. The colcon workspace root is the parent of `src`.

## Step 3: Configure Git identity

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

## Step 4: Confirm Git is tracking the right repository

```bash
cd ~/ros2_ws/src/ur5_digital_twin
git rev-parse --show-toplevel
```

Expected: the path ends in `ur5_digital_twin`.

## Step 5: Fix accidental Git repository in `ros2_ws`

If `~/ros2_ws/.git` exists, that was a mistaken `git init` in the workspace root. The repository is the package directory.

```bash
rm -rf ~/ros2_ws/.git
cd ~/ros2_ws/src/ur5_digital_twin
git status -sb
```

# 8. Gazebo Graphics Configuration

## Step 1: Test Gazebo by itself

```bash
source /opt/ros/jazzy/setup.bash
gz sim -v 4
```

Expected: a window, or a server log if you pass `-s`. Ctrl-C to stop.

## Step 2: Use software rendering workaround

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_SIM_RENDER_ENGINE=ogre
gz sim -v 4
```

## Step 3: Make the workaround permanent if needed

```bash
echo 'export GZ_SIM_RENDER_ENGINE=ogre' >> ~/.bashrc
```

`ur5 simgui` already defaults the render engine to `ogre` when the variable is empty.

## Step 4: Update WSL graphics support

On WSL2: `wsl --update` from Windows, then `wsl --shutdown`. On an Apple Silicon VM, software rendering plus `ogre` is the supported path. Native Ubuntu with a working GPU can leave `GZ_SIM_RENDER_ENGINE` unset.

# 9. Controller Configuration Path Check

The controller YAML is not hardcoded to a home directory. `sim.launch.py` replaces `__ROS2_CONTROLLERS_YAML__` with the share path of `ur5_moveit_config/config/ros2_controllers.yaml`.

## Step 1: Search for hardcoded paths

```bash
grep -RIn "/home/" ~/ros2_ws/src/ur5_digital_twin/ur5_description ~/ros2_ws/src/ur5_digital_twin/ur5_moveit_config || echo "no hardcoded home paths"
```

Expected: `no hardcoded home paths`.

## Step 2: Quick machine-specific fix

If a stale checkout still contains `/home/<someone>/...` inside a YAML `parameters:` entry, replace it with the launch substitution. Do not commit a personal home path.

## Step 3: Verify the controller YAML file exists

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix ur5_moveit_config
ls "$(ros2 pkg prefix ur5_moveit_config)/share/ur5_moveit_config/config/ros2_controllers.yaml"
```

## Step 4: Rebuild after fixing paths

```bash
ur5 build
```

## Long-term recommended fix

Keep using the placeholder in the URDF. The same launch file injects `candle_pose` into `__CANDLE_<joint>__`. Pose numbers live only in the SRDF.

# 10. Environment Verification

## Step 1: Check ROS distribution

```bash
echo "$ROS_DISTRO"
```

Expected: `jazzy`.

## Step 2: Check middleware

```bash
# after any ur5 command, or export it yourself:
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
echo "$RMW_IMPLEMENTATION"
```

Expected: `rmw_zenoh_cpp`.

## Step 3: Check workspace sourcing

```bash
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head
```

Expected: a line containing `ros2_ws/install`.

## Step 4: Check project packages

```bash
ros2 pkg list | grep ur5
```

Expected:

```text
ur5_controller
ur5_description
ur5_moveit_config
```

## Step 5: Check ROS-Gazebo packages

```bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix gz_ros2_control
```

Both print a path under `/opt/ros/jazzy`.

## Step 6: ur5 doctor

With or without a running sim:

```bash
~/ros2_ws/src/ur5_digital_twin/scripts/ur5 doctor
```

Expected: a report on stdout and a file under `$UR5_DATA_DIR/doctor/doctor_*.log`. It records `ROS_DISTRO`, `RMW_IMPLEMENTATION`, and whether `rmw_zenohd` is running.

# 11. Running Initial Tests

## Step 1: Launch the master simulation

```bash
ur5 sim
```

Equivalent:

```bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py gz_headless:=true start_rviz:=false
```

Expected: the log reaches `Candle reached` and the process stays up. Ctrl-C or, from another terminal, `ur5 kill`.

## Step 2: Verify controller manager

In a second terminal:

```bash
ur5 ctl
```

Expected lines include `joint_state_broadcaster` active and `ur_manipulator_controller` active.

## Step 3: Run a Python control script directly

```bash
ur5 demo
```

Equivalent:

```bash
ros2 run ur5_controller interactive_control.py --scheme position --csv demo_15s.csv
```

Expected: exit 0, an HDF5 path, and a Candle reset line.

Unit tests, no simulator:

```bash
ur5 test --unit
```

Expected: pytest ends with no failures.

# 12. Common Troubleshooting

## Issue 1: `package 'ros_gz_sim' not found`

rosdep did not install the Gazebo bridge. Repeat section 3 step 3, then `ur5 build`.

## Issue 2: Gazebo opens then immediately closes under WSL

Software rendering is off or the guest has no GPU. Use section 8 step 2, or run `ur5 sim` (headless).

## Issue 3: Controller manager is not available

Bring-up is still spawning, or Zenoh is down. Wait for the Candle line. If `ur5 ctl` stays empty, `ur5 zenoh`.

## Issue 4: NumPy/SciPy import error

Section 4. Reinstall the apt packages. Do not pip-install over them.

## Issue 5: Wrong Python script path

Entry points are `ros2 run ur5_controller interactive_control.py` and `ros2 run ur5_controller telemetry.py`. They live in `ur5_controller/src/` and are installed to `lib/ur5_controller/`. Do not call a removed `scripts/` path inside the package.

## Issue 6: ROS 2 repository has malformed URL

`/etc/apt/sources.list.d/ros2.list` must be one line and must contain your Ubuntu codename. Rewrite it with section 2 step 4.

## Issue 7: ROS GPG key error

Repeat section 2 step 3, then `sudo apt update`.

## Issue 8: `ros2: command not found`

```bash
source /opt/ros/jazzy/setup.bash
```

`ur5` does this for you.

## Issue 9: Packages do not appear after building

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep ur5
```

If the build failed, read the colcon log under `~/.ros/log` or `~/ros2_ws/log`.

# 13. Quick Setup Command Summary

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8

# Enable Universe
sudo add-apt-repository universe

# Add ROS 2 key and repository
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Install ROS 2 Jazzy
sudo apt install -y ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

# Install core development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential

# Install MoveIt, ros2_control, and Gazebo dependencies
sudo apt install -y ros-jazzy-moveit ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control \
  ros-jazzy-rmw-zenoh-cpp python3-numpy python3-h5py python3-yaml ros-jazzy-python-orocos-kdl-vendor python3-pytest

# Create workspace and clone repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository-url> ur5_digital_twin

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Install package dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# Build workspace
colcon build --symlink-install
source install/setup.bash

# Optional graphics workaround
export GZ_SIM_RENDER_ENGINE=ogre

# Verify packages
ros2 pkg list | grep ur5
```

# Final Verification Checklist

* `echo $ROS_DISTRO` is `jazzy`.
* `ur5 doctor` writes a log and shows `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
* `ur5 test --unit` passes.
* `ur5 sim` ends its bring-up with `Candle reached` and the arm upright.
* `ur5 ctl` shows `ur_manipulator_controller` active and `forward_effort_controller` inactive.
* `ur5 demo` writes an HDF5 file and returns to Candle.
* `ur5 validate` on that file prints `PASS:`.
* `ur5 kill` leaves no `gz sim` process.
