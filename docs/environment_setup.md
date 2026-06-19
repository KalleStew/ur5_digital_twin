# Development Environment Setup

This guide provides step-by-step instructions for configuring a local machine to run the UR5 Digital Twin and HIL platform.

The project is designed for:

- **Ubuntu 22.04 LTS Jammy Jellyfish**
- **ROS 2 Humble Hawksbill**
- **MoveIt 2**
- **Gazebo / Ignition Gazebo**
- **ros2_control**
- **WSL2, native Ubuntu, or an Ubuntu virtual machine**

This guide is written primarily for **Windows users running Ubuntu 22.04 through WSL2**, but the ROS 2 installation and workspace setup steps also apply to native Ubuntu and Ubuntu virtual machines.

---

## Table of Contents

1. [Host Platform Configuration](#1-host-platform-configuration)
2. [ROS 2 Humble Installation](#2-ros-2-humble-installation)
3. [System and Simulation Dependencies](#3-system-and-simulation-dependencies)
4. [Python Numerical Package Compatibility](#4-python-numerical-package-compatibility)
5. [Workspace and Repository Setup](#5-workspace-and-repository-setup)
6. [Build and Source the Workspace](#6-build-and-source-the-workspace)
7. [VS Code and Git Source Control Setup](#7-vs-code-and-git-source-control-setup)
8. [WSL2 Gazebo Graphics Configuration](#8-wsl2-gazebo-graphics-configuration)
9. [Controller Configuration Path Check](#9-controller-configuration-path-check)
10. [Environment Verification](#10-environment-verification)
11. [Running Initial Tests](#11-running-initial-tests)
12. [Common Troubleshooting](#12-common-troubleshooting)
13. [Quick Setup Command Summary](#13-quick-setup-command-summary)

---

# 1. Host Platform Configuration

Choose the setup that matches your deployment hardware.

---

## Option A: Windows Subsystem for Linux 2, WSL2

This is the recommended option for Windows hosts.

Windows 11 includes WSLg, which allows Linux GUI applications such as RViz and Gazebo to run without manually installing an external X11 server.

---

### Step 1: Install Ubuntu 22.04 in WSL2

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-22.04
```

Restart your computer if prompted.

After installation, open **Ubuntu 22.04** from the Windows Start Menu.

You will be prompted to create a Linux username and password.

Example:

```text
Enter new UNIX username: kstew
New password:
Retype new password:
```

The password will not visibly appear while typing. This is normal.

---

### Step 2: Confirm Ubuntu version

In the Ubuntu terminal, run:

```bash
lsb_release -a
```

Expected output should include:

```text
Release: 22.04
Codename: jammy
```

ROS 2 Humble is intended for Ubuntu 22.04 Jammy.

---

### Step 3: Update Ubuntu

Run:

```bash
sudo apt update
sudo apt upgrade -y
```

---

### Step 4: Allocate enough memory to WSL2

Gazebo, RViz, and MoveIt can be memory intensive. It is recommended to allocate at least **8 GB of RAM** to WSL2.

On Windows, create or edit this file:

```text
C:\Users\<YourWindowsUsername>\.wslconfig
```

Example `.wslconfig`:

```ini
[wsl2]
memory=8GB
processors=4
swap=4GB
localhostForwarding=true
```

If your machine has sufficient resources, this is better:

```ini
[wsl2]
memory=12GB
processors=6
swap=4GB
localhostForwarding=true
```

After saving the file, restart WSL from PowerShell:

```powershell
wsl --shutdown
```

Then reopen Ubuntu 22.04.

---

### Step 5: Confirm Linux GUI support

Install a simple GUI test package:

```bash
sudo apt install x11-apps -y
```

Run:

```bash
xeyes
```

If a small window opens, WSLg GUI support is working.

Close the `xeyes` window when finished.

---

### Important WSL filesystem note

Keep the ROS 2 workspace inside the Linux filesystem:

```text
/home/<username>/ros2_ws
```

Recommended:

```text
/home/kstew/ros2_ws
```

Avoid building the workspace inside:

```text
/mnt/c/Users/...
```

Building under `/mnt/c` can cause slow builds, symbolic link problems, and permission issues.

---

## Option B: Dedicated Ubuntu Virtual Machine

If using VMware, VirtualBox, or Parallels, install Ubuntu 22.04 Desktop.

Recommended VM settings:

- **RAM:** 8 GB minimum, 16 GB recommended
- **CPU:** 4 cores minimum
- **Disk:** 40 GB minimum, 60 GB recommended
- **Graphics:** Enable 3D acceleration
- **Video memory:** Maximize available video memory

After installing Ubuntu, update packages:

```bash
sudo apt update
sudo apt upgrade -y
```

---

## Option C: Native Ubuntu 22.04

For native Ubuntu 22.04, update the system:

```bash
sudo apt update
sudo apt upgrade -y
```

Confirm the release:

```bash
lsb_release -a
```

Expected:

```text
Release: 22.04
Codename: jammy
```

---

# 2. ROS 2 Humble Installation

ROS 2 Humble must be installed through the official ROS 2 apt repository.

---

## Step 1: Configure locale

Run:

```bash
locale
sudo apt update
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

Confirm the output includes:

```text
LANG=en_US.UTF-8
```

---

## Step 2: Enable the Universe repository

Run:

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update
```

If prompted, press `Enter`.

---

## Step 3: Add the ROS 2 GPG key

Install `curl`:

```bash
sudo apt install curl -y
```

Add the ROS 2 key:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Verify the key exists:

```bash
ls -l /usr/share/keyrings/ros-archive-keyring.gpg
```

---

## Step 4: Add the ROS 2 repository

Run this command exactly:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Check the file:

```bash
cat /etc/apt/sources.list.d/ros2.list
```

Expected output on Ubuntu 22.04:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

Important: the file must **not** contain Markdown link formatting.

Incorrect:

```text
[http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu)
```

Correct:

```text
http://packages.ros.org/ros2/ubuntu
```

---

## Step 5: Update apt

Run:

```bash
sudo apt update
```

If this completes without ROS repository errors, continue.

---

## Step 6: Install ROS 2 Humble Desktop

Run:

```bash
sudo apt install ros-humble-desktop -y
```

This installs:

- ROS 2 command line tools
- core ROS 2 libraries
- RViz2
- common messages and visualization tools

---

## Step 7: Source ROS 2 automatically

Source ROS 2 in the current terminal:

```bash
source /opt/ros/humble/setup.bash
```

Add it to `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check the active ROS distribution:

```bash
echo $ROS_DISTRO
```

Expected output:

```text
humble
```

---

# 3. System and Simulation Dependencies

Install the development tools, MoveIt 2 packages, Gazebo integration packages, and ros2_control packages required by this project.

---

## Step 1: Install build tools

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep git tree build-essential cmake python3-pip -y
```

These packages provide:

- `colcon` for ROS 2 workspace builds
- `rosdep` for dependency resolution
- `git` for source control
- `tree` for directory inspection
- compiler and CMake tools
- Python package tooling

---

## Step 2: Install MoveIt 2 and ros2_control

```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control -y
```

---

## Step 3: Install ROS-Gazebo integration packages

The launch file uses `ros_gz_sim`, so the following packages are required:

```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces ros-humble-ros-gz-image -y
```

If `ros_gz_sim` is missing, the launch file may fail with:

```text
PackageNotFoundError: "package 'ros_gz_sim' not found"
```

Verify installation:

```bash
ros2 pkg list | grep ros_gz
```

Expected packages include:

```text
ros_gz_bridge
ros_gz_image
ros_gz_interfaces
ros_gz_sim
```

---

## Step 4: Install robot description and visualization tools

```bash
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2 -y
```

These are used for:

- URDF/Xacro processing
- joint state publishing
- robot state publishing
- RViz visualization

---

## Step 5: Install CycloneDDS middleware

CycloneDDS is recommended for this project because it handles high-frequency ROS 2 communication reliably.

Install:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
```

Add it to `.bashrc`:

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
echo $RMW_IMPLEMENTATION
```

Expected output:

```text
rmw_cyclonedds_cpp
```

---

# 4. Python Numerical Package Compatibility

This project uses Python scripts that may import:

```python
from scipy.signal import butter, filtfilt
```

On Ubuntu 22.04, the system SciPy package is not compatible with NumPy 2.x.

If NumPy was upgraded through `pip`, you may see errors such as:

```text
A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy
detected version 2.2.6
```

or:

```text
AttributeError: _ARRAY_API not found
ImportError: numpy.core.multiarray failed to import
```

---

## Recommended fix

Install a SciPy-compatible NumPy version:

```bash
python3 -m pip install --user "numpy==1.24.4" --force-reinstall
```

Verify:

```bash
python3 -c "import numpy; print('NumPy:', numpy.__version__, numpy.__file__)"
python3 -c "import scipy; print('SciPy:', scipy.__version__, scipy.__file__)"
```

NumPy should report:

```text
1.24.4
```

---

## Alternative fix

If you want to remove pip-installed NumPy and use the Ubuntu version:

```bash
python3 -m pip uninstall numpy
```

You may need to run that more than once until it reports NumPy is no longer installed.

Then install Ubuntu packages:

```bash
sudo apt install python3-numpy python3-scipy -y
```

Verify again:

```bash
python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"
python3 -c "import scipy; print(scipy.__version__, scipy.__file__)"
```

---

## Important Python warning

Avoid running global upgrades like this in a ROS 2 Humble environment:

```bash
pip install --upgrade numpy
pip install --upgrade scipy
```

unless you are using a virtual environment.

ROS 2 Humble on Ubuntu 22.04 expects Python packages that are compatible with the Jammy system package versions.

---

# 5. Workspace and Repository Setup

The expected workspace layout is:

```text
~/ros2_ws
└── src
    └── ur5_digital_twin
        ├── ur5_controller
        ├── ur5_description
        └── ur5_moveit_config
```

---

## Step 1: Create the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Verify location:

```bash
pwd
```

Expected:

```text
/home/<username>/ros2_ws/src
```

Example:

```text
/home/kstew/ros2_ws/src
```

---

## Step 2: Clone the project repository

Clone the repository into the `src` directory:

```bash
git clone https://github.com/KalleStew/ur5_digital_twin.git
```

Then enter the repository:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
```

Check the remote:

```bash
git remote -v
```

Expected HTTPS output:

```text
origin  https://github.com/KalleStew/ur5_digital_twin.git (fetch)
origin  https://github.com/KalleStew/ur5_digital_twin.git (push)
```

---

## Step 3: Inspect package structure

Return to the workspace root:

```bash
cd ~/ros2_ws
```

List packages:

```bash
find ~/ros2_ws/src -name package.xml
```

Expected output should include something like:

```text
/home/kstew/ros2_ws/src/ur5_digital_twin/ur5_controller/package.xml
/home/kstew/ros2_ws/src/ur5_digital_twin/ur5_description/package.xml
/home/kstew/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/package.xml
```

---

# 6. Build and Source the Workspace

---

## Step 1: Initialize rosdep

Run:

```bash
sudo rosdep init
```

If you see:

```text
ERROR: default sources list file already exists
```

that is okay. It means `rosdep` was already initialized.

Continue with:

```bash
rosdep update
```

---

## Step 2: Install package dependencies

From the workspace root:

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

If you see a warning related to an old `gazebo_ros_control` dependency, it may be a legacy ROS 1 artifact in a MoveIt configuration package. Review it, but it may not block compilation if the required ROS 2 control packages are installed.

---

## Step 3: Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

The `--symlink-install` flag is recommended for development because edits to Python scripts, launch files, YAML files, and URDF/Xacro resources are reflected more easily.

---

## Step 4: Source the local workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add this to `.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
echo $AMENT_PREFIX_PATH
```

The output should include:

```text
/home/kstew/ros2_ws/install
```

---

# 7. VS Code and Git Source Control Setup

---

## Step 1: Install the VS Code WSL extension

In Windows VS Code:

1. Open Extensions.
2. Search for:

```text
WSL
```

3. Install the Microsoft **WSL** extension.

---

## Step 2: Open the repository correctly

From the WSL terminal:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
code .
```

In VS Code, the bottom-left corner should show:

```text
WSL: Ubuntu-22.04
```

This confirms that VS Code is connected to WSL.

---

## Step 3: Configure Git identity

If this is your first time using Git inside WSL:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

Check:

```bash
git config --global --list
```

---

## Step 4: Confirm Git is tracking the right repository

In the VS Code terminal:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
git status
git remote -v
```

Expected output should indicate that the repository is connected to:

```text
https://github.com/KalleStew/ur5_digital_twin.git
```

---

## Step 5: Fix accidental Git repository in `ros2_ws`

A common mistake is accidentally creating a Git repository in:

```text
~/ros2_ws
```

instead of using the actual cloned repository:

```text
~/ros2_ws/src/ur5_digital_twin
```

Check for nested Git repositories:

```bash
find ~/ros2_ws -name .git -type d
```

If you see:

```text
/home/kstew/ros2_ws/.git
/home/kstew/ros2_ws/src/ur5_digital_twin/.git
```

then the outer Git repo was probably accidental.

Remove only the outer one:

```bash
cd ~/ros2_ws
rm -rf .git
```

Do **not** remove:

```text
~/ros2_ws/src/ur5_digital_twin/.git
```

That is the actual cloned repository.

Then reopen VS Code correctly:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
code .
```

---

# 8. WSL2 Gazebo Graphics Configuration

When running Gazebo through WSL2, Gazebo may initially open and then close unexpectedly.

A common error is:

```text
Ogre::UnimplementedException
GL3PlusTextureGpu::copyTo
```

This is a WSL/OpenGL/OGRE rendering issue.

---

## Step 1: Test Gazebo by itself

Run:

```bash
ign gazebo
```

If Gazebo crashes, the issue is likely WSL graphics-related.

---

## Step 2: Use software rendering workaround

Before launching the project, run:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
```

Then launch the simulation:

```bash
cd ~/ros2_ws
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

This forces Mesa software rendering. It may be slower, but it is often more stable under WSL2.

---

## Step 3: Make the workaround permanent if needed

If the software rendering workaround fixes Gazebo, add it to `.bashrc`:

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> ~/.bashrc
echo "export MESA_GLSL_VERSION_OVERRIDE=330" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 4: Update WSL graphics support

From Windows PowerShell:

```powershell
wsl --update
wsl --shutdown
```

Then reopen Ubuntu.

Inside WSL:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install mesa-utils -y
```

Check OpenGL information:

```bash
glxinfo -B
```

Look at:

```text
OpenGL renderer string
OpenGL version string
```

---

# 9. Controller Configuration Path Check

The Gazebo control plugin must be able to load the controller configuration YAML file.

A known issue is a hardcoded absolute path inside the URDF, such as:

```text
/home/kallestewart/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/config/ros2_controllers.yaml
```

On another machine, this path will fail.

The error may look like:

```text
Error opening YAML file
Couldn't parse params file
```

This can prevent the controller manager from starting, which then causes:

```text
Could not contact service /controller_manager/list_controllers
```

---

## Step 1: Search for hardcoded paths

Run:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
grep -R "/home/kallestew" -n .
```

If output appears, the project contains a hardcoded path that should be fixed.

---

## Step 2: Quick machine-specific fix

If your WSL username is `kstew`, replace the old path with your current home path:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
grep -RIl "/home/kallestew" . | xargs sed -i 's|/home/kallestewart|/home/kstew|g'
grep -RIl "/home/kallestew" . | xargs sed -i 's|/home/kallestewa|/home/kstew|g'
```

Then confirm the bad path is gone:

```bash
grep -R "/home/kallestew" -n .
```

---

## Step 3: Verify the controller YAML file exists

Run:

```bash
ls -l ~/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/config/ros2_controllers.yaml
```

If it does not exist, find it:

```bash
find ~/ros2_ws/src/ur5_digital_twin -name "ros2_controllers.yaml"
```

---

## Step 4: Rebuild after fixing paths

```bash
cd ~/ros2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Long-term recommended fix

Hardcoded absolute paths should eventually be replaced with package-relative paths using launch substitutions or Xacro arguments.

A portable launch-file approach is:

```python
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

controllers_yaml = PathJoinSubstitution([
    FindPackageShare("ur5_moveit_config"),
    "config",
    "ros2_controllers.yaml"
])
```

This avoids username-specific paths.

---

# 10. Environment Verification

After installation and build, verify the environment.

---

## Step 1: Check ROS distribution

```bash
echo $ROS_DISTRO
```

Expected:

```text
humble
```

---

## Step 2: Check middleware

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_cyclonedds_cpp
```

---

## Step 3: Check workspace sourcing

```bash
echo $AMENT_PREFIX_PATH
```

Expected output should include:

```text
/home/kstew/ros2_ws/install
```

---

## Step 4: Check project packages

```bash
ros2 pkg list | grep ur5
```

Expected packages may include:

```text
ur5_controller
ur5_description
ur5_moveit_config
```

---

## Step 5: Check ROS-Gazebo packages

```bash
ros2 pkg list | grep ros_gz
```

Expected packages include:

```text
ros_gz_bridge
ros_gz_image
ros_gz_interfaces
ros_gz_sim
```

---

# 11. Running Initial Tests

---

## Step 1: Launch the master simulation

In Terminal 1:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

Expected behavior:

- Gazebo opens
- RViz opens
- Robot model appears
- Controller spawners start
- `controller_manager` becomes available

---

## Step 2: Verify controller manager

Open Terminal 2:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
```

Expected controllers may include:

```text
joint_state_broadcaster
ur_manipulator_controller
forward_effort_controller
```

A healthy state may look like:

```text
joint_state_broadcaster active
ur_manipulator_controller active
forward_effort_controller inactive
```

---

## Step 3: Run a Python control script directly

Some scripts are located inside:

```text
~/ros2_ws/src/ur5_digital_twin/ur5_controller/src
```

For example:

```bash
cd ~/ros2_ws/src/ur5_digital_twin/ur5_controller/src
python3 conventional_joint_space.py
```

Do not run:

```bash
python3 ur5_controller/src/conventional_joint_space.py
```

from `~/ros2_ws`, because that path does not exist.

If running from the workspace root, use the full relative path:

```bash
cd ~/ros2_ws
python3 src/ur5_digital_twin/ur5_controller/src/conventional_joint_space.py
```

---

# 12. Common Troubleshooting

---

## Issue 1: `package 'ros_gz_sim' not found`

Error:

```text
PackageNotFoundError: "package 'ros_gz_sim' not found"
```

Fix:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces ros-humble-ros-gz-image -y
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep ros_gz
```

---

## Issue 2: Gazebo opens then immediately closes under WSL

Error may include:

```text
Ogre::UnimplementedException
GL3PlusTextureGpu::copyTo
```

Fix:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
```

Then relaunch:

```bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

If it works, add the variables to `.bashrc`.

---

## Issue 3: Controller manager is not available

Error:

```text
Could not contact service /controller_manager/list_controllers
```

Possible causes:

1. Gazebo crashed before loading `gz_ros2_control`.
2. The controller YAML file path is wrong.
3. The controller YAML file does not exist.
4. Required `ros2_control` packages are missing.

Check for hardcoded paths:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
grep -R "/home/kallestew" -n .
```

Check for the controller file:

```bash
ls -l ~/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/config/ros2_controllers.yaml
```

Then rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Issue 4: NumPy/SciPy import error

Error:

```text
A NumPy version >=1.17.3 and <1.25.0 is required
detected version 2.2.6
```

or:

```text
ImportError: numpy.core.multiarray failed to import
```

Fix:

```bash
python3 -m pip install --user "numpy==1.24.4" --force-reinstall
```

Verify:

```bash
python3 -c "import numpy; print(numpy.__version__)"
python3 -c "import scipy; print(scipy.__version__)"
```

---

## Issue 5: Wrong Python script path

Error:

```text
python3: can't open file '/home/kstew/ros2_ws/ur5_controller/src/conventional_joint_space.py': No such file or directory
```

Cause:

The script is inside:

```text
~/ros2_ws/src/ur5_digital_twin/ur5_controller/src
```

Correct command:

```bash
cd ~/ros2_ws/src/ur5_digital_twin/ur5_controller/src
python3 conventional_joint_space.py
```

Or:

```bash
cd ~/ros2_ws
python3 src/ur5_digital_twin/ur5_controller/src/conventional_joint_space.py
```

---

## Issue 6: ROS 2 repository has malformed URL

Error:

```text
E: The method driver /usr/lib/apt/methods/[http could not be found
```

Cause:

The ROS 2 repository file contains Markdown link syntax.

Fix:

```bash
sudo nano /etc/apt/sources.list.d/ros2.list
```

Replace the file contents with:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

Then:

```bash
sudo apt update
```

---

## Issue 7: ROS GPG key error

Error:

```text
NO_PUBKEY F42ED6FBAB17C654
```

Fix:

```bash
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

---

## Issue 8: `ros2: command not found`

Cause:

ROS 2 was not sourced.

Fix:

```bash
source /opt/ros/humble/setup.bash
```

Make permanent:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Issue 9: Packages do not appear after building

If this command does not show project packages:

```bash
ros2 pkg list | grep ur5
```

try:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep ur5
```

Also verify package files exist:

```bash
find ~/ros2_ws/src -name package.xml
```

---

# 13. Quick Setup Command Summary

This section provides a condensed command sequence for a fresh Ubuntu 22.04 / WSL2 setup.

```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Locale
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable Universe
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update

# Add ROS 2 key and repository
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install core development tools
sudo apt install python3-colcon-common-extensions python3-rosdep git tree build-essential cmake python3-pip -y

# Install MoveIt, ros2_control, and Gazebo dependencies
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control -y
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces ros-humble-ros-gz-image -y
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2 -y

# Install CycloneDDS
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# Fix NumPy/SciPy compatibility
python3 -m pip install --user "numpy==1.24.4" --force-reinstall

# Create workspace and clone repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/KalleStew/ur5_digital_twin.git

# Initialize rosdep
sudo rosdep init
rosdep update

# Install package dependencies
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y

# Check for hardcoded old home paths
cd ~/ros2_ws/src/ur5_digital_twin
grep -R "/home/kallestew" -n .

# If needed, replace old hardcoded path with current WSL user path
grep -RIl "/home/kallestew" . | xargs sed -i 's|/home/kallestewart|/home/kstew|g'
grep -RIl "/home/kallestew" . | xargs sed -i 's|/home/kallestewa|/home/kstew|g'

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Optional WSL Gazebo graphics workaround
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> ~/.bashrc
echo "export MESA_GLSL_VERSION_OVERRIDE=330" >> ~/.bashrc
source ~/.bashrc

# Verify packages
ros2 pkg list | grep ur5
ros2 pkg list | grep ros_gz
```

If `sudo rosdep init` reports that it already exists, continue with:

```bash
rosdep update
```

---

# Final Verification Checklist

Before running the full UR5 Digital Twin system, verify the following:

```bash
lsb_release -a
```

Expected:

```text
Ubuntu 22.04
jammy
```

Check ROS:

```bash
echo $ROS_DISTRO
```

Expected:

```text
humble
```

Check middleware:

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_cyclonedds_cpp
```

Check workspace:

```bash
echo $AMENT_PREFIX_PATH
```

Should include:

```text
/home/kstew/ros2_ws/install
```

Check project packages:

```bash
ros2 pkg list | grep ur5
```

Check ROS-Gazebo packages:

```bash
ros2 pkg list | grep ros_gz
```

Check controller YAML:

```bash
ls -l ~/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/config/ros2_controllers.yaml
```

Launch simulation:

```bash
cd ~/ros2_ws
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

In another terminal, verify controllers:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 control list_controllers
```

If Gazebo stays open, RViz loads, and the controllers appear, the development environment is ready.
