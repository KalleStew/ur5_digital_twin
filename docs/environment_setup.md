# Development Environment Setup

This guide provides step-by-step instructions for configuring a machine to run the UR5 Digital Twin on **ROS 2 Jazzy Jalisco** and **Ubuntu 24.04 LTS Noble Numbat**.

The recommended Windows development workflow is:

- Use **Windows 11 + WSL2 + Ubuntu 24.04**
- Use **ROS 2 Jazzy**
- Use **Gazebo Harmonic**
- Use **MoveIt 2 for Jazzy**
- Use **Eclipse Zenoh middleware**
- Build from the ROS workspace root:

  ```bash
  ~/ros2_ws
  ```

- Use Git from the repository root:

  ```bash
  ~/ros2_ws/src/ur5_digital_twin
  ```

---

## Target Platforms

- Windows 11 with **WSL2** running Ubuntu 24.04
- Apple Silicon with an **ARM64 Ubuntu 24.04 virtual machine** using UTM or Parallels
- Native Ubuntu 24.04 x86-64

---

## Software Stack

- Ubuntu 24.04 LTS Noble Numbat
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- MoveIt 2 Jazzy
- Eclipse Zenoh middleware: `rmw_zenoh_cpp`
- `ros2_control`
- `gz_ros2_control`
- Python 3.12
- `h5py`, `numpy`, `scipy`

---

## Table of Contents

1. [Host Platform Configuration](#1-host-platform-configuration)
2. [Ubuntu 24.04 First-Boot Setup](#2-ubuntu-2404-first-boot-setup)
3. [ROS 2 Jazzy Installation](#3-ros-2-jazzy-installation)
4. [Eclipse Zenoh Middleware](#4-eclipse-zenoh-middleware)
5. [Gazebo Harmonic and gz_ros2_control](#5-gazebo-harmonic-and-gz_ros2_control)
6. [MoveIt 2 Jazzy](#6-moveit-2-jazzy)
7. [Python Research Dependencies](#7-python-research-dependencies)
8. [Workspace and Repository Setup](#8-workspace-and-repository-setup)
9. [Build and Source](#9-build-and-source)
10. [VS Code and Git Source Control Setup](#10-vs-code-and-git-source-control-setup)
11. [WSL2 Gazebo Graphics Configuration](#11-wsl2-gazebo-graphics-configuration)
12. [Controller Configuration Path Check](#12-controller-configuration-path-check)
13. [Environment Verification](#13-environment-verification)
14. [Running Initial Tests](#14-running-initial-tests)
15. [Common Troubleshooting](#15-common-troubleshooting)
16. [Quick Setup Command Summary](#16-quick-setup-command-summary)
17. [Final Verification Checklist](#17-final-verification-checklist)

---

# 1. Host Platform Configuration

## Option A: Windows 11 — WSL2

Windows 11 ships with WSLg, which allows Linux GUI applications such as RViz and Gazebo to run without a separate X11 server.

### Step 1: Install Ubuntu 24.04 in WSL2

Open **PowerShell as Administrator**:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart if prompted. Then open **Ubuntu 24.04** from the Start Menu and create your Linux username and password.

### Step 2: Confirm the Ubuntu version

Inside WSL:

```bash
lsb_release -a
```

Expected:

```text
Ubuntu 24.04 LTS
Codename: noble
```

### Step 3: Update WSL

From Windows PowerShell:

```powershell
wsl --update
wsl --shutdown
```

Reopen Ubuntu 24.04 afterward.

### Step 4: Verify WSLg graphics support

Inside WSL:

```bash
sudo apt update
sudo apt install -y mesa-utils
glxinfo -B
```

Check:

```text
OpenGL renderer string
OpenGL version string
```

Ideally, the renderer should show your Windows GPU or a WSL graphics backend. If Gazebo later crashes with OpenGL or OGRE errors, use the WSL2 Gazebo graphics workaround in [Section 11](#11-wsl2-gazebo-graphics-configuration).

---

## Option B: Apple Silicon ARM64 — UTM or Parallels VM

ROS 2 Jazzy supports `linux/arm64`. Use an ARM64 Ubuntu 24.04 VM through UTM or Parallels.

Recommended VM settings:

- Architecture: ARM64
- RAM: 8 GB minimum
- CPU: 4 cores minimum
- Disk: 50 GB minimum
- Display: VirtIO GPU where available
- Shared Clipboard: enabled

After first boot, install a minimal desktop if needed:

```bash
sudo apt update
sudo apt install -y ubuntu-desktop-minimal
```

If Gazebo Harmonic crashes with the default `ogre2` render engine, try:

```bash
gz sim --render-engine ogre -r empty.sdf
```

---

## Option C: Native Ubuntu 24.04

For native Ubuntu 24.04 x86-64, update the system:

```bash
sudo apt update
sudo apt upgrade -y
```

Confirm:

```bash
lsb_release -a
```

Expected:

```text
Release: 24.04
Codename: noble
```

---

# 2. Ubuntu 24.04 First-Boot Setup

Run these commands on a fresh Ubuntu 24.04 installation.

```bash
sudo apt update
sudo apt upgrade -y

sudo apt install -y \
  curl \
  gnupg2 \
  lsb-release \
  software-properties-common \
  build-essential \
  cmake \
  git \
  tree \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete \
  mesa-utils
```

## Initialize rosdep

```bash
sudo rosdep init
```

If this reports:

```text
ERROR: default sources list file already exists
```

that is fine. Continue with:

```bash
rosdep update
```

---

# 3. ROS 2 Jazzy Installation

## Step 1: Add the ROS 2 apt repository

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

Confirm the repository file does not contain Markdown link formatting:

```bash
cat /etc/apt/sources.list.d/ros2.list
```

Correct format:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main
```

## Step 2: Install ROS 2 Jazzy Desktop

```bash
sudo apt install -y ros-jazzy-desktop
```

This installs the ROS 2 CLI tools, RViz, rqt, standard messages, and common development packages.

## Step 3: Install ros2_control

```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager
```

## Step 4: Source ROS 2 automatically

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
echo $ROS_DISTRO
```

Expected:

```text
jazzy
```

---

# 4. Eclipse Zenoh Middleware

## Why Zenoh?

Eclipse Zenoh is recommended for this project because it is generally more robust across WSL2, VMs, and network boundary conditions than multicast-dependent DDS discovery.

For high-frequency control and telemetry topics, Zenoh can reduce discovery issues and improve communication stability in virtualized environments.

## Installation

```bash
sudo apt install -y ros-jazzy-rmw-zenoh-cpp
```

Verify:

```bash
ros2 pkg list | grep zenoh
```

Expected:

```text
rmw_zenoh_cpp
```

## Configure Zenoh as the default RMW

```bash
echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_zenoh_cpp
```

## Basic communication test

Terminal 1:

```bash
ros2 topic pub /test std_msgs/msg/String "data: 'zenoh_test'" --rate 1
```

Terminal 2:

```bash
ros2 topic echo /test
```

Expected output:

```text
data: zenoh_test
```

---

# 5. Gazebo Harmonic and gz_ros2_control

ROS 2 Jazzy uses **Gazebo Harmonic**. The older `ignition-*` package naming has been replaced by `gz-*`.

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-ros-gz-image \
  ros-jazzy-gz-ros2-control
```

Verify Gazebo:

```bash
gz sim --version
```

Expected:

```text
Gazebo Sim, version 8.x.x
```

Test Gazebo:

```bash
gz sim -r empty.sdf
```

If Gazebo crashes under WSL2 with an OpenGL or OGRE error, use the workaround in [Section 11](#11-wsl2-gazebo-graphics-configuration).

---

# 6. MoveIt 2 Jazzy

Install MoveIt 2 and related packages:

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-planners \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-moveit-configs-utils \
  ros-jazzy-moveit-ros-visualization
```

Verify that `MoveItConfigsBuilder` is importable:

```bash
python3 -c "from moveit_configs_utils import MoveItConfigsBuilder; print('MoveIt OK')"
```

Expected:

```text
MoveIt OK
```

---

# 7. Python Research Dependencies

The project uses Python packages such as `h5py`, `numpy`, and `scipy` for numerical operations and telemetry logging.

Prefer Ubuntu system packages to avoid conflicts with ROS 2 Python packages:

```bash
sudo apt install -y \
  python3-h5py \
  python3-numpy \
  python3-scipy
```

Verify:

```bash
python3 -c "import h5py, numpy, scipy; print('h5py:', h5py.__version__); print('numpy:', numpy.__version__); print('scipy:', scipy.__version__)"
```

Avoid global upgrades like:

```bash
pip install --upgrade numpy
pip install --upgrade scipy
```

unless you are intentionally using a virtual environment. ROS 2 Jazzy on Ubuntu 24.04 expects packages that are compatible with the system Python environment.

---

# 8. Workspace and Repository Setup

The ROS workspace root is:

```bash
~/ros2_ws
```

The Git repository root is:

```bash
~/ros2_ws/src/ur5_digital_twin
```

Expected structure:

```text
~/ros2_ws
├── build
├── install
├── log
└── src
    └── ur5_digital_twin
        ├── .git
        ├── ur5_controller
        ├── ur5_description
        └── ur5_moveit_config
```

## Step 1: Create the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Verify:

```bash
pwd
```

Expected:

```text
/home/<username>/ros2_ws/src
```

## Step 2: Clone the repository

```bash
git clone https://github.com/KalleStew/ur5_digital_twin.git
```

Enter the repository:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
```

Check Git status:

```bash
git status
```

Check the remote:

```bash
git remote -v
```

Expected:

```text
origin  https://github.com/KalleStew/ur5_digital_twin.git (fetch)
origin  https://github.com/KalleStew/ur5_digital_twin.git (push)
```

## Step 3: Confirm package structure

```bash
find ~/ros2_ws/src -name package.xml
```

Expected output should include:

```text
/home/<username>/ros2_ws/src/ur5_digital_twin/ur5_controller/package.xml
/home/<username>/ros2_ws/src/ur5_digital_twin/ur5_description/package.xml
/home/<username>/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/package.xml
```

---

# 9. Build and Source

## Step 1: Install package dependencies

From the workspace root:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

## Step 2: Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

The `--symlink-install` flag is recommended during development because edits to Python scripts, launch files, YAML files, URDF files, and Xacro files are reflected more conveniently.

Expected packages include:

- `ur5_controller`
- `ur5_description`
- `ur5_moveit_config`

## Step 3: Source the workspace overlay

```bash
source ~/ros2_ws/install/setup.bash
```

Add workspace sourcing to `.bashrc` only after the first successful build:

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
/home/<username>/ros2_ws/install
```

---

# 10. VS Code and Git Source Control Setup

## Important folder distinction

There are two relevant folders:

| Purpose | Folder |
|---|---|
| ROS workspace root | `~/ros2_ws` |
| Git repository root | `~/ros2_ws/src/ur5_digital_twin` |

Build and launch commands should generally be run from:

```bash
~/ros2_ws
```

Git commands should generally be run from:

```bash
~/ros2_ws/src/ur5_digital_twin
```

## Step 1: Install VS Code on Windows

On Windows, install VS Code normally from:

```text
https://code.visualstudio.com/
```

Then install the Microsoft **WSL** extension in VS Code.

Do not install VS Code through `snap` inside WSL for the standard Windows + WSL workflow.

## Step 2: Open the repository directly

For the cleanest Source Control behavior, open the actual Git repository root:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
code .
```

In VS Code, the lower-left corner should show something like:

```text
WSL: Ubuntu-24.04
```

This confirms that VS Code is connected to the WSL environment.

When opened this way, VS Code Source Control should show:

- the repository name
- the current branch
- changed files
- commit box
- branch selector in the bottom status bar

## Step 3: Build from the ROS workspace root

Even if VS Code is opened at the repository root, build from the ROS workspace root:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Step 4: Optional aliases

Add useful shortcuts:

```bash
cat << 'EOF' >> ~/.bashrc

# ROS 2 workspace shortcuts
alias ws='cd ~/ros2_ws'
alias repo='cd ~/ros2_ws/src/ur5_digital_twin'
alias build='cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash'
alias cbuild='cd ~/ros2_ws && rm -rf build/ install/ log/ && colcon build --symlink-install && source install/setup.bash'
alias src_ws='source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash'
alias run_sim='ros2 launch ur5_moveit_config gazebo_sim.launch.py' 

alias plan='cd ~/ros2_ws && source install/setup.bash && ros2 launch ur5_controller planner.launch.py'
alias plan_cli='cd ~/ros2_ws/src/ur5_digital_twin/ur5_controller/src && python3 planner_cli.py'

EOF

source ~/.bashrc
```

If planner stdin is not forwarded in your terminal, start the fallback CLI after `plan`:

```bash
plan_cli
```

This publishes commands to `/planner_command` and avoids launch-stdin issues.

All terminals must use the same `RMW_IMPLEMENTATION`. If you use Zenoh, run a router before using topic-based helper tools:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Usage:

```bash
ws
```

takes you to:

```bash
~/ros2_ws
```

and:

```bash
repo
```

takes you to:

```bash
~/ros2_ws/src/ur5_digital_twin
```

## Step 5: Configure Git identity

If this is your first time using Git inside WSL:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

Verify:

```bash
git config --global --list
```

## Step 6: Confirm Git is tracking the correct folder

```bash
cd ~/ros2_ws/src/ur5_digital_twin
git status
git remote -v
git rev-parse --show-toplevel
```

Expected top-level path:

```text
/home/<username>/ros2_ws/src/ur5_digital_twin
```

## Step 7: If opening `~/ros2_ws` instead of the repo

If you prefer opening the entire ROS workspace:

```bash
cd ~/ros2_ws
code .
```

then enable nested Git repository detection:

```bash
mkdir -p ~/ros2_ws/.vscode

cat > ~/ros2_ws/.vscode/settings.json << 'EOF'
{
  "git.autoRepositoryDetection": "subFolders",
  "git.repositoryScanMaxDepth": 10,
  "git.decorations.enabled": true,
  "scm.alwaysShowRepositories": true
}
EOF
```

Then reload VS Code:

```text
Ctrl + Shift + P
Developer: Reload Window
```

If the nested repo still does not show clearly, open the repo directly instead:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
code .
```

## Step 8: Fix accidental Git repository in the wrong folder

A common mistake is accidentally running `git init` in the home directory or ROS workspace root.

Check for `.git` directories:

```bash
find ~ -maxdepth 5 -name .git -type d
```

The correct repo should be:

```text
/home/<username>/ros2_ws/src/ur5_digital_twin/.git
```

If you accidentally created:

```text
/home/<username>/.git
```

remove only that accidental home-level Git metadata:

```bash
rm -rf ~/.git
```

If you accidentally created:

```text
/home/<username>/ros2_ws/.git
```

remove only that accidental workspace-level Git metadata:

```bash
rm -rf ~/ros2_ws/.git
```

Do not remove:

```bash
~/ros2_ws/src/ur5_digital_twin/.git
```

That is the actual repository.

---

# 11. WSL2 Gazebo Graphics Configuration

Gazebo Harmonic may crash under WSL2 due to OpenGL or OGRE rendering issues.

Possible errors include:

```text
Ogre::UnimplementedException
GL3PlusTextureGpu::copyTo
```

or Gazebo may open and immediately close.

## Step 1: Test Gazebo by itself

```bash
gz sim -r empty.sdf
```

## Step 2: Try software rendering

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

gz sim -r empty.sdf
```

If this fixes Gazebo, use the same variables before launching the project:

```bash
cd ~/ros2_ws
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

## Step 3: Make the workaround permanent only if needed

Only add these to `.bashrc` if Gazebo is unstable without them:

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> ~/.bashrc
echo "export MESA_GLSL_VERSION_OVERRIDE=330" >> ~/.bashrc
source ~/.bashrc
```

## Step 4: Update WSL graphics support

From Windows PowerShell:

```powershell
wsl --update
wsl --shutdown
```

Then reopen Ubuntu and run:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y mesa-utils
glxinfo -B
```

---

# 12. Controller Configuration Path Check

The Gazebo control plugin must be able to load the controller YAML file.

The expected controller configuration file is:

```bash
~/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/config/ros2_controllers.yaml
```

Verify:

```bash
ls -l ~/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/config/ros2_controllers.yaml
```

If it does not exist, search for it:

```bash
find ~/ros2_ws/src/ur5_digital_twin -name "ros2_controllers.yaml"
```

## Check for hardcoded user paths

Hardcoded absolute paths can break the simulation on another machine.

Search for old hardcoded paths:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
grep -R "/home/" -n ur5_description ur5_moveit_config ur5_controller | grep -E "kalle|kstew|ice|ros2_ws"
```

If a path points to another user's home directory, replace it with a portable package-relative solution where possible.

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

After fixing paths, rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

# 13. Environment Verification

Run these checks after installation and build.

## Check Ubuntu

```bash
lsb_release -a
```

Expected:

```text
Ubuntu 24.04
Codename: noble
```

## Check ROS distribution

```bash
echo $ROS_DISTRO
```

Expected:

```text
jazzy
```

## Check middleware

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_zenoh_cpp
```

## Check Gazebo

```bash
gz sim --version | head -1
```

Expected:

```text
Gazebo Sim, version 8.x
```

## Check gz_ros2_control

```bash
ros2 pkg list | grep gz_ros2_control
```

Expected:

```text
gz_ros2_control
```

## Check project packages

```bash
ros2 pkg list | grep ur5
```

Expected packages include:

```text
ur5_controller
ur5_description
ur5_moveit_config
```

## Check workspace sourcing

```bash
echo $AMENT_PREFIX_PATH
```

Expected output should include:

```text
/home/<username>/ros2_ws/install
```

---

# 14. Running Initial Tests

## Step 1: Launch the simulation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

Expected behavior:

- Gazebo opens
- RViz opens
- Robot model appears
- Controller spawners start
- `controller_manager` becomes available

If Gazebo crashes under WSL2, try:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

## Step 2: Verify controllers

In another terminal:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
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

## Step 3: Run a Python control script directly

Some scripts are located inside:

```bash
~/ros2_ws/src/ur5_digital_twin/ur5_controller/src
```

Example:

```bash
cd ~/ros2_ws/src/ur5_digital_twin/ur5_controller/src
python3 conventional_joint_space.py
```

If running from the workspace root:

```bash
cd ~/ros2_ws
python3 src/ur5_digital_twin/ur5_controller/src/conventional_joint_space.py
```

Do not run:

```bash
python3 ur5_controller/src/conventional_joint_space.py
```

from `~/ros2_ws`, because that path does not exist.

---

# 15. Common Troubleshooting

## Issue 1: `ros2: command not found`

Cause: ROS 2 was not sourced.

Fix:

```bash
source /opt/ros/jazzy/setup.bash
```

Make permanent:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Issue 2: Project packages do not appear after building

If this does not show project packages:

```bash
ros2 pkg list | grep ur5
```

run:

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

## Issue 3: `package 'ros_gz_sim' not found`

Error:

```text
PackageNotFoundError: "package 'ros_gz_sim' not found"
```

Fix:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-ros-gz-image

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep ros_gz
```

---

## Issue 4: Gazebo opens then immediately closes under WSL2

Possible error:

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
cd ~/ros2_ws
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

If it works, optionally add the variables to `.bashrc`.

---

## Issue 5: Controller manager is not available

Error:

```text
Could not contact service /controller_manager/list_controllers
```

Possible causes:

1. Gazebo crashed before loading `gz_ros2_control`.
2. The controller YAML file path is wrong.
3. The controller YAML file does not exist.
4. Required `ros2_control` packages are missing.
5. The workspace was not sourced after building.

Check required packages:

```bash
ros2 pkg list | grep gz_ros2_control
ros2 pkg list | grep controller_manager
```

Check the controller file:

```bash
ls -l ~/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/config/ros2_controllers.yaml
```

Rebuild and source:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Issue 6: NumPy, SciPy, or h5py import error

Prefer system packages:

```bash
sudo apt install -y python3-numpy python3-scipy python3-h5py
```

Verify:

```bash
python3 -c "import numpy, scipy, h5py; print(numpy.__version__); print(scipy.__version__); print(h5py.__version__)"
```

Avoid unnecessary global pip upgrades in the ROS 2 system Python environment.

---

## Issue 7: Wrong Python script path

Error:

```text
python3: can't open file '/home/<username>/ros2_ws/ur5_controller/src/conventional_joint_space.py': No such file or directory
```

Correct path:

```bash
cd ~/ros2_ws/src/ur5_digital_twin/ur5_controller/src
python3 conventional_joint_space.py
```

Or from the workspace root:

```bash
cd ~/ros2_ws
python3 src/ur5_digital_twin/ur5_controller/src/conventional_joint_space.py
```

---

## Issue 8: ROS 2 repository has malformed URL

Error:

```text
E: The method driver /usr/lib/apt/methods/[http could not be found
```

Cause: the ROS 2 repository file contains Markdown link syntax.

Fix:

```bash
sudo nano /etc/apt/sources.list.d/ros2.list
```

Replace the contents with:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main
```

Then:

```bash
sudo apt update
```

---

## Issue 9: ROS GPG key error

Error:

```text
NO_PUBKEY F42ED6FBAB17C654
```

Fix:

```bash
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

---

## Issue 10: VS Code Source Control does not show the repository

The correct Git repository is:

```bash
~/ros2_ws/src/ur5_digital_twin
```

The ROS workspace root is:

```bash
~/ros2_ws
```

If Source Control does not look normal, open the actual repo directly:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
code .
```

Check:

```bash
git status
git rev-parse --show-toplevel
```

Expected:

```text
/home/<username>/ros2_ws/src/ur5_digital_twin
```

If you prefer opening `~/ros2_ws`, enable nested repo detection:

```bash
mkdir -p ~/ros2_ws/.vscode

cat > ~/ros2_ws/.vscode/settings.json << 'EOF'
{
  "git.autoRepositoryDetection": "subFolders",
  "git.repositoryScanMaxDepth": 10,
  "git.decorations.enabled": true,
  "scm.alwaysShowRepositories": true
}
EOF
```

Then reload VS Code.

---

## Issue 11: Accidental Git repository in home or workspace root

Check:

```bash
find ~ -maxdepth 5 -name .git -type d
```

Correct:

```text
/home/<username>/ros2_ws/src/ur5_digital_twin/.git
```

If this exists accidentally:

```text
/home/<username>/.git
```

remove it:

```bash
rm -rf ~/.git
```

If this exists accidentally:

```text
/home/<username>/ros2_ws/.git
```

remove it:

```bash
rm -rf ~/ros2_ws/.git
```

Do not remove:

```bash
~/ros2_ws/src/ur5_digital_twin/.git
```

---

# 16. Quick Setup Command Summary

This section provides a condensed setup sequence for a fresh Ubuntu 24.04 / WSL2 installation.

```bash
# ── System update ─────────────────────────────────────────────────────────
sudo apt update
sudo apt upgrade -y

# ── Base development tools ────────────────────────────────────────────────
sudo apt install -y \
  curl \
  gnupg2 \
  lsb-release \
  software-properties-common \
  build-essential \
  cmake \
  git \
  tree \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete \
  mesa-utils

# ── rosdep ────────────────────────────────────────────────────────────────
sudo rosdep init || true
rosdep update

# ── ROS 2 Jazzy repository ────────────────────────────────────────────────
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# ── ROS 2 Jazzy desktop ──────────────────────────────────────────────────
sudo apt install -y ros-jazzy-desktop

# ── Middleware, simulation, control, and MoveIt ───────────────────────────
sudo apt install -y \
  ros-jazzy-rmw-zenoh-cpp \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-ros-gz-image \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-moveit \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-planners \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-moveit-configs-utils \
  ros-jazzy-moveit-ros-visualization \
  python3-h5py \
  python3-numpy \
  python3-scipy

# ── Environment setup ────────────────────────────────────────────────────
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc
source ~/.bashrc

# ── Workspace and repository ─────────────────────────────────────────────
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/KalleStew/ur5_digital_twin.git

# ── Dependencies and build ───────────────────────────────────────────────
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash

# ── Source workspace automatically after successful build ────────────────
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# ── Optional aliases ─────────────────────────────────────────────────────
cat << 'EOF' >> ~/.bashrc

# ROS 2 workspace shortcuts
alias ws='cd ~/ros2_ws'
alias repo='cd ~/ros2_ws/src/ur5_digital_twin'
alias build_ws='cd ~/ros2_ws && colcon build --symlink-install'
alias src_ws='source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash'
EOF

source ~/.bashrc

# ── Verify packages ──────────────────────────────────────────────────────
ros2 pkg list | grep ur5
ros2 pkg list | grep ros_gz
ros2 pkg list | grep gz_ros2_control
```

Optional WSL2 Gazebo graphics workaround:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
```

Only add those variables to `.bashrc` if Gazebo is unstable without them.

---

# 17. Final Verification Checklist

Before running the full UR5 Digital Twin system, verify the following.

## Ubuntu version

```bash
lsb_release -a
```

Expected:

```text
Ubuntu 24.04
noble
```

## ROS distribution

```bash
echo $ROS_DISTRO
```

Expected:

```text
jazzy
```

## Middleware

```bash
echo $RMW_IMPLEMENTATION
```

Expected:

```text
rmw_zenoh_cpp
```

## Workspace overlay

```bash
echo $AMENT_PREFIX_PATH
```

Should include:

```text
/home/<username>/ros2_ws/install
```

## Project packages

```bash
ros2 pkg list | grep ur5
```

Expected:

```text
ur5_controller
ur5_description
ur5_moveit_config
```

## Gazebo packages

```bash
ros2 pkg list | grep ros_gz
```

Expected packages include:

```text
ros_gz_bridge
ros_gz_interfaces
ros_gz_sim
```

## gz_ros2_control

```bash
ros2 pkg list | grep gz_ros2_control
```

Expected:

```text
gz_ros2_control
```

## Controller YAML

```bash
ls -l ~/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/config/ros2_controllers.yaml
```

## Git repository

```bash
cd ~/ros2_ws/src/ur5_digital_twin
git status
git remote -v
git rev-parse --show-toplevel
```

Expected top level:

```text
/home/<username>/ros2_ws/src/ur5_digital_twin
```

## Launch simulation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

In another terminal:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 control list_controllers
```

If Gazebo stays open, RViz loads, the robot appears, and controllers are listed, the development environment is ready.