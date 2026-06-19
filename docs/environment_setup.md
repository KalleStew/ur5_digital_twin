# Development Environment Setup

This guide provides detailed step-by-step instructions for configuring a local development environment to run the UR5 Digital Twin and HIL platform.

The project is designed for:

- **Ubuntu 22.04 LTS Jammy Jellyfish**
- **ROS 2 Humble Hawksbill**
- **Gazebo / Ignition simulation tools**
- **MoveIt 2**
- **ros2_control**
- **WSL2, native Ubuntu, or an Ubuntu virtual machine**

This setup guide supports three host configurations:

1. Windows Subsystem for Linux 2, WSL2
2. Dedicated Ubuntu virtual machine
3. Native Ubuntu 22.04 installation

For Windows users, **WSL2 with Ubuntu 22.04 is recommended** because Windows 11 includes WSLg, which supports Linux GUI applications such as RViz and Gazebo without needing a separate X11 server.

---

## 1. Host Platform Configuration

Choose the setup that matches your deployment hardware.

---

### Option A: Windows Subsystem for Linux, WSL2

This option is recommended for Windows hosts.

Windows 11 natively supports Linux GUI applications using WSLg. This makes WSL2 a convenient environment for running ROS 2, RViz, and Gazebo from a Linux terminal while still using Windows as the main operating system.

#### Step 1: Install Ubuntu 22.04 in WSL2

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-22.04
```

If prompted, restart your computer.

After restarting, open the **Ubuntu 22.04** application from the Windows Start Menu.

You will be asked to create a UNIX username and password. This is separate from your Windows login.

Example:

```text
Enter new UNIX username: kstew
New password:
Retype new password:
```

The password will not visibly appear while typing. This is normal.

---

#### Step 2: Confirm Ubuntu version

In the Ubuntu terminal, run:

```bash
lsb_release -a
```

You should see output similar to:

```text
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.x LTS
Release:        22.04
Codename:       jammy
```

The important values are:

```text
Release: 22.04
Codename: jammy
```

ROS 2 Humble is intended for Ubuntu 22.04.

---

#### Step 3: Update Ubuntu packages

Run:

```bash
sudo apt update
sudo apt upgrade -y
```

---

#### Step 4: Allocate enough memory to WSL2

ROS 2, Gazebo, RViz, and MoveIt can be memory intensive. It is recommended to allocate at least **8 GB RAM** to WSL2. If your system has enough memory, **12 GB to 16 GB** is better.

Create or edit this file in Windows:

```text
C:\Users\<YourWindowsUsername>\.wslconfig
```

Example `.wslconfig` contents:

```ini
[wsl2]
memory=8GB
processors=4
swap=4GB
localhostForwarding=true
```

If your computer has 16 GB or more system RAM, you can use:

```ini
[wsl2]
memory=12GB
processors=6
swap=4GB
localhostForwarding=true
```

After saving `.wslconfig`, restart WSL from PowerShell:

```powershell
wsl --shutdown
```

Then reopen Ubuntu 22.04.

---

#### Step 5: Confirm WSLg GUI support

WSLg is usually included by default on Windows 11. To test Linux GUI support, run:

```bash
sudo apt install x11-apps -y
xeyes
```

A small window with moving eyes should appear.

If this works, GUI applications such as RViz and Gazebo should also be able to open later.

Close the `xeyes` program when finished.

---

#### Important WSL filesystem note

For best performance, create and build the ROS 2 workspace inside the Linux filesystem:

```text
/home/<username>/ros2_ws
```

Recommended:

```text
/home/kstew/ros2_ws
```

Avoid building inside the mounted Windows filesystem:

```text
/mnt/c/Users/...
```

Building ROS 2 workspaces under `/mnt/c` can cause slow builds, permission problems, and symbolic link issues.

---

### Option B: Dedicated Virtual Machine, VMware / VirtualBox / Parallels

If using a VM, 3D hardware acceleration is critical for Gazebo and RViz performance.

#### Step 1: Install Ubuntu 22.04 Desktop

Install a fresh Ubuntu 22.04 Desktop image in your VM software.

#### Step 2: Configure VM resources

Recommended VM settings:

- **RAM:** 8 GB minimum, 16 GB recommended
- **CPU:** 4 cores minimum
- **Display:** Enable 3D hardware acceleration
- **Video memory:** Maximize available video memory, for example 128 MB or 256 MB
- **Disk space:** 40 GB minimum, 60 GB or more recommended

After booting Ubuntu, update the system:

```bash
sudo apt update
sudo apt upgrade -y
```

---

### Option C: Native Ubuntu 22.04

No preliminary virtualization setup is required.

Update your system packages:

```bash
sudo apt update
sudo apt upgrade -y
```

Confirm the Ubuntu version:

```bash
lsb_release -a
```

You should see:

```text
Release: 22.04
Codename: jammy
```

---

## 2. ROS 2 Humble Installation

Once Ubuntu 22.04 is running, install ROS 2 Humble.

These instructions follow the standard ROS 2 Humble installation process, but the commands below are written carefully to avoid common copy/paste formatting problems.

---

### Step 1: Set locale

ROS 2 requires a UTF-8 locale.

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

Confirm that the locale output includes:

```text
LANG=en_US.UTF-8
```

---

### Step 2: Enable the Ubuntu Universe repository

Run:

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
```

If prompted, press `Enter`.

Update package lists:

```bash
sudo apt update
```

---

### Step 3: Add the ROS 2 GPG key

Install `curl` first:

```bash
sudo apt install curl -y
```

Download the ROS 2 repository signing key:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Confirm the key file exists:

```bash
ls -l /usr/share/keyrings/ros-archive-keyring.gpg
```

You should see a file listed at:

```text
/usr/share/keyrings/ros-archive-keyring.gpg
```

---

### Step 4: Add the ROS 2 apt repository

Run the following command exactly:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Check the repository file:

```bash
cat /etc/apt/sources.list.d/ros2.list
```

For Ubuntu 22.04 on a typical amd64 computer, it should look similar to:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

#### Important formatting warning

The repository line must **not** contain Markdown-style link formatting.

Incorrect:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) jammy main
```

Correct:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

If the line contains extra square brackets around the URL, `apt` may produce errors such as:

```text
E: The method driver /usr/lib/apt/methods/[http could not be found.
N: Is the package apt-transport-[http installed?
```

If that happens, edit the file:

```bash
sudo nano /etc/apt/sources.list.d/ros2.list
```

Replace the entire contents with:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

Then save and exit:

- Press `Ctrl + O`
- Press `Enter`
- Press `Ctrl + X`

---

### Step 5: Update apt package lists

Run:

```bash
sudo apt update
```

If the repository and key are configured correctly, this should complete without ROS repository errors.

---

### Step 6: Fix missing ROS GPG key error, if needed

If you see an error similar to:

```text
The following signatures couldn't be verified because the public key is not available: NO_PUBKEY F42ED6FBAB17C654
E: The repository 'http://packages.ros.org/ros2/ubuntu jammy InRelease' is not signed.
```

Re-download the ROS key:

```bash
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

Then confirm that the `signed-by` path in the repository file matches the key location:

```bash
cat /etc/apt/sources.list.d/ros2.list
```

It must reference:

```text
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg
```

---

### Step 7: Install ROS 2 Humble Desktop

Install the full desktop version of ROS 2 Humble:

```bash
sudo apt install ros-humble-desktop -y
```

This installs:

- ROS 2 core libraries
- ROS 2 command line tools
- RViz2
- common message packages
- visualization and debugging utilities

---

### Step 8: Source the ROS 2 environment

Source ROS 2 in the current terminal:

```bash
source /opt/ros/humble/setup.bash
```

Add it to `.bashrc` so it is sourced automatically in future terminals:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Confirm ROS 2 is available:

```bash
ros2 --version
```

You can also check the active ROS distribution:

```bash
echo $ROS_DISTRO
```

Expected output:

```text
humble
```

---

## 3. System and Simulation Dependencies

Next, install the build tools, simulation dependencies, control packages, and middleware required by the UR5 Digital Twin project.

---

### Step 1: Install Colcon and developer tools

Run:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep git tree build-essential cmake python3-pip -y
```

These packages provide:

- `colcon` for building ROS 2 workspaces
- `rosdep` for dependency resolution
- `git` for cloning and source control
- `tree` for inspecting directory structure
- `build-essential` and `cmake` for compiling C++ packages
- `python3-pip` for Python tooling

---

### Step 2: Install MoveIt 2 and ros2_control packages

Run:

```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control -y
```

These packages provide:

- MoveIt 2 motion planning tools
- ROS 2 control framework
- standard ROS 2 controllers
- Ignition/Gazebo integration with `ros2_control`

---

### Step 3: Install additional robot simulation dependencies

Depending on the exact launch files and robot description used by the project, the following packages may also be required:

```bash
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2 -y
```

These packages provide:

- `xacro` for processing parameterized URDF files
- joint state publisher tools
- robot state publisher
- RViz2 visualization support

---

### Step 4: Configure high-frequency middleware, CycloneDDS

By default, ROS 2 Humble commonly uses Fast DDS. For this project, CycloneDDS is recommended because the system may transmit high-frequency command, state, or torque data during hardware-in-the-loop operation.

Install CycloneDDS:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
```

Add the middleware selection to `.bashrc`:

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

Confirm it is set:

```bash
echo $RMW_IMPLEMENTATION
```

Expected output:

```text
rmw_cyclonedds_cpp
```

---

## 4. Workspace Initialization, Repository Setup, and Compilation

With ROS 2 and the system dependencies installed, create the ROS 2 workspace, clone the UR5 Digital Twin repository, install package dependencies, and build the project.

This section assumes the repository is hosted at:

```text
https://github.com/KalleStew/ur5_digital_twin.git
```

If your repository URL is different, replace the URL in the `git clone` command with the correct one.

---

### Step 1: Create the ROS 2 workspace

Create a workspace directory inside the Linux home directory:

```bash
mkdir -p ~/ros2_ws/src
```

Move into the source directory:

```bash
cd ~/ros2_ws/src
```

Confirm the current directory:

```bash
pwd
```

Expected output:

```text
/home/<username>/ros2_ws/src
```

Example:

```text
/home/kstew/ros2_ws/src
```

---

### Step 2: Clone the project repository

Clone the UR5 Digital Twin repository into the `src` directory:

```bash
git clone https://github.com/KalleStew/ur5_digital_twin.git
```

Confirm that the repository was cloned:

```bash
ls
```

Expected output:

```text
ur5_digital_twin
```

Move into the repository:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
```

Check the Git remote:

```bash
git remote -v
```

Expected output for HTTPS:

```text
origin  https://github.com/KalleStew/ur5_digital_twin.git (fetch)
origin  https://github.com/KalleStew/ur5_digital_twin.git (push)
```

If using SSH, the output may be:

```text
origin  git@github.com:KalleStew/ur5_digital_twin.git (fetch)
origin  git@github.com:KalleStew/ur5_digital_twin.git (push)
```

---

### Step 3: Inspect the repository structure

Return to the workspace root:

```bash
cd ~/ros2_ws
```

Use `tree` to inspect the workspace:

```bash
tree -L 3 ~/ros2_ws/src
```

You should see a structure similar to:

```text
/home/kstew/ros2_ws/src
└── ur5_digital_twin
    ├── docs
    ├── ur5_controller
    ├── ur5_description
    └── ur5_moveit_config
```

The exact structure may vary, but the important requirement is that ROS 2 packages contain `package.xml` files.

Find all package files:

```bash
find ~/ros2_ws/src -name package.xml
```

Expected output should include package files similar to:

```text
/home/kstew/ros2_ws/src/ur5_digital_twin/ur5_controller/package.xml
/home/kstew/ros2_ws/src/ur5_digital_twin/ur5_description/package.xml
/home/kstew/ros2_ws/src/ur5_digital_twin/ur5_moveit_config/package.xml
```

If these files appear, ROS 2 should be able to discover and build the packages.

---

### Step 4: Initialize rosdep

`rosdep` scans the `package.xml` files in the workspace and installs missing system dependencies.

Run:

```bash
sudo rosdep init
```

If this is the first time `rosdep` has been initialized, the command should complete successfully.

If you see:

```text
ERROR: default sources list file already exists
```

that is okay. It means `rosdep` was already initialized.

Then run:

```bash
rosdep update
```

---

### Step 5: Install project dependencies with rosdep

From the workspace root, run:

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

This checks all ROS 2 packages under `src` and installs required dependencies.

If you see an error related to:

```text
gazebo_ros_control
```

this may be caused by a legacy ROS 1 dependency entry in a MoveIt configuration package. If the rest of the dependencies install correctly and the package is not needed for ROS 2 compilation, it can typically be ignored. However, review the error carefully to ensure it is not blocking other dependencies.

---

### Step 6: Build the workspace

From the workspace root:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

The `--symlink-install` option is recommended for development. It allows changes to Python files, launch files, configuration files, and other non-compiled resources to take effect without rebuilding every time.

A successful build should end with a summary similar to:

```text
Summary: 3 packages finished
```

The number of packages may differ depending on the repository contents.

---

### Step 7: Source the local workspace

After building, source the workspace setup file:

```bash
source ~/ros2_ws/install/setup.bash
```

Add this to `.bashrc` so the workspace is sourced automatically in new terminals:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Confirm that the environment was sourced:

```bash
echo $AMENT_PREFIX_PATH
```

The output should include a path similar to:

```text
/home/kstew/ros2_ws/install
```

---

### Step 8: Rebuilding after changes

For C++ source changes, package configuration changes, or new package files, rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

For many Python, launch, YAML, and URDF/Xacro edits, `--symlink-install` usually allows changes to appear without a full rebuild. However, if a package is not being detected correctly, rebuild and source again.

---

## 5. VS Code and Source Control Setup

This section explains how to open the cloned repository correctly in VS Code from WSL and avoid accidentally using the wrong Git repository.

---

### Step 1: Install VS Code WSL extension

In Windows VS Code:

1. Open the Extensions tab.
2. Search for:

```text
WSL
```

3. Install the Microsoft **WSL** extension.

This allows VS Code to connect directly to the WSL Linux filesystem.

---

### Step 2: Open the repository from the WSL terminal

From the WSL Ubuntu terminal, move into the cloned repository:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
```

Open VS Code:

```bash
code .
```

VS Code should open with a WSL connection.

Check the bottom-left corner of VS Code. It should say something like:

```text
WSL: Ubuntu-22.04
```

This confirms that VS Code is editing files inside WSL, not through the Windows filesystem.

---

### Step 3: Confirm Git is tracking the correct repository

Open the VS Code terminal using:

```text
Ctrl + `
```

Run:

```bash
git status
```

Expected output should look similar to:

```text
On branch main
Your branch is up to date with 'origin/main'.

nothing to commit, working tree clean
```

Then check the remote:

```bash
git remote -v
```

Expected output:

```text
origin  https://github.com/KalleStew/ur5_digital_twin.git (fetch)
origin  https://github.com/KalleStew/ur5_digital_twin.git (push)
```

---

### Step 4: Configure Git user information

If this is your first time using Git in WSL, configure your name and email:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

Example:

```bash
git config --global user.name "Kalle Stewart"
git config --global user.email "your_email@example.com"
```

Verify:

```bash
git config --global --list
```

---

### Step 5: Use the VS Code Source Control panel

Open the Source Control panel:

```text
Ctrl + Shift + G
```

or click the Source Control icon on the left sidebar.

From here, you can:

- View changed files
- Stage files
- Commit changes
- Pull from GitHub
- Push to GitHub
- Create and switch branches

Common workflow:

```bash
git status
git add .
git commit -m "Update environment setup instructions"
git push
```

The same actions can also be performed through the VS Code Source Control interface.

---

### Step 6: Fix accidentally initialized Git repository in `ros2_ws`

A common mistake is accidentally creating a Git repository in the workspace root:

```text
~/ros2_ws
```

instead of using the cloned Git repository:

```text
~/ros2_ws/src/ur5_digital_twin
```

To check for nested Git repositories, run:

```bash
find ~/ros2_ws -name .git -type d
```

You may see:

```text
/home/kstew/ros2_ws/.git
/home/kstew/ros2_ws/src/ur5_digital_twin/.git
```

The correct repository to keep is:

```text
/home/kstew/ros2_ws/src/ur5_digital_twin/.git
```

If `/home/kstew/ros2_ws/.git` was created accidentally, remove only that outer `.git` folder:

```bash
cd ~/ros2_ws
rm -rf .git
```

This does **not** delete the workspace files. It only removes Git tracking from the outer workspace directory.

Do **not** remove:

```text
~/ros2_ws/src/ur5_digital_twin/.git
```

That is the actual cloned repository.

After removing the accidental outer Git repository, open the correct folder:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
code .
```

---

### Step 7: Recommended `.gitignore` entries

The ROS 2 build output directories should not usually be committed.

If the repository does not already include these entries, add them to `.gitignore`:

```gitignore
# ROS 2 build artifacts
build/
install/
log/

# Python cache
__pycache__/
*.pyc

# Local environment files
.env

# Optional: editor settings
.vscode/
```

If the project intentionally uses shared VS Code tasks, launch configurations, or settings, do not ignore the entire `.vscode/` directory. Instead, selectively ignore only local files.

---

## 6. Verification

After the workspace has been built and sourced, verify that ROS 2 can locate the project packages.

---

### Step 1: Source ROS 2 and the workspace

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

If both lines were added to `.bashrc`, opening a new terminal should source them automatically.

---

### Step 2: Check for UR5 packages

Run:

```bash
ros2 pkg list | grep ur5
```

Expected output should include:

```text
ur5_controller
ur5_description
ur5_moveit_config
```

If the package names differ, inspect the available packages:

```bash
ros2 pkg list | grep -i ur
```

You can also check specific packages:

```bash
ros2 pkg prefix ur5_controller
ros2 pkg prefix ur5_description
ros2 pkg prefix ur5_moveit_config
```

Expected output should show paths under:

```text
/home/kstew/ros2_ws/install
```

---

### Step 3: Check environment variables

Confirm ROS 2 Humble is active:

```bash
echo $ROS_DISTRO
```

Expected output:

```text
humble
```

Confirm CycloneDDS is active:

```bash
echo $RMW_IMPLEMENTATION
```

Expected output:

```text
rmw_cyclonedds_cpp
```

Confirm the workspace is included:

```bash
echo $AMENT_PREFIX_PATH
```

The output should include:

```text
/home/kstew/ros2_ws/install
```

---

### Step 4: Basic ROS 2 command test

Run:

```bash
ros2 topic list
```

This should execute without a `ros2: command not found` error.

If no ROS nodes are running, the output may be minimal. That is normal.

---

## 7. Common Troubleshooting

This section documents common setup errors and their fixes.

---

### Issue 1: `apt` method driver `[http` could not be found

Error example:

```text
E: The method driver /usr/lib/apt/methods/[http could not be found.
N: Is the package apt-transport-[http installed?
E: Failed to fetch [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu)/dists/jammy/InRelease
```

Cause:

The ROS 2 repository file contains Markdown-style link formatting. This usually happens when copying commands from rendered Markdown where the URL appears as a clickable link.

Fix:

Open the repository file:

```bash
sudo nano /etc/apt/sources.list.d/ros2.list
```

Replace the contents with:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

Save and exit:

- `Ctrl + O`
- `Enter`
- `Ctrl + X`

Then run:

```bash
sudo apt update
```

---

### Issue 2: ROS GPG key missing

Error example:

```text
The following signatures couldn't be verified because the public key is not available: NO_PUBKEY F42ED6FBAB17C654
E: The repository 'http://packages.ros.org/ros2/ubuntu jammy InRelease' is not signed.
```

Fix:

```bash
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

Confirm the repository file points to the same key:

```bash
cat /etc/apt/sources.list.d/ros2.list
```

It should contain:

```text
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg
```

---

### Issue 3: `ros-humble-desktop` cannot be located

Error example:

```text
E: Unable to locate package ros-humble-desktop
```

Possible causes:

1. ROS 2 apt repository was not added correctly.
2. `sudo apt update` failed.
3. Ubuntu version is not 22.04 Jammy.
4. The repository file contains malformed syntax.

Check Ubuntu version:

```bash
lsb_release -a
```

Check ROS repository file:

```bash
cat /etc/apt/sources.list.d/ros2.list
```

Expected for Ubuntu 22.04:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

Update again:

```bash
sudo apt update
```

Then retry:

```bash
sudo apt install ros-humble-desktop -y
```

---

### Issue 4: `sudo rosdep init` says it already exists

Error example:

```text
ERROR: default sources list file already exists:
    /etc/ros/rosdep/sources.list.d/20-default.list
```

This is not a serious problem. It means `rosdep` was already initialized.

Continue with:

```bash
rosdep update
```

---

### Issue 5: `ros2: command not found`

Cause:

The ROS 2 environment has not been sourced.

Fix:

```bash
source /opt/ros/humble/setup.bash
```

To make it permanent:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Confirm:

```bash
ros2 --version
```

---

### Issue 6: Workspace packages do not appear after building

If:

```bash
ros2 pkg list | grep ur5
```

does not show the expected packages, try the following.

Confirm package files exist:

```bash
find ~/ros2_ws/src -name package.xml
```

Rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Check again:

```bash
ros2 pkg list | grep ur5
```

---

### Issue 7: VS Code opens the wrong folder or wrong Git repository

The correct folder to open is:

```text
/home/kstew/ros2_ws/src/ur5_digital_twin
```

Open it from WSL:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
code .
```

If VS Code shows a closed repository named `ros2_ws`, do not reopen it unless you intentionally made the workspace root a Git repository.

Check for accidental nested Git repositories:

```bash
find ~/ros2_ws -name .git -type d
```

If you see:

```text
/home/kstew/ros2_ws/.git
```

and that repository was accidental, remove it:

```bash
cd ~/ros2_ws
rm -rf .git
```

Keep:

```text
/home/kstew/ros2_ws/src/ur5_digital_twin/.git
```

Then reopen the project:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
code .
```

---

## 8. Quick Start Command Summary

This section provides a condensed setup sequence for Ubuntu 22.04 after WSL or Ubuntu is already installed.

Run these commands in order.

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

# Install development and simulation dependencies
sudo apt install python3-colcon-common-extensions python3-rosdep git tree build-essential cmake python3-pip -y
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control -y
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2 -y

# Configure CycloneDDS
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# Create workspace and clone repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/KalleStew/ur5_digital_twin.git

# Initialize rosdep
sudo rosdep init
rosdep update

# Install project dependencies
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y

# Build workspace
colcon build --symlink-install

# Source workspace
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Verify packages
ros2 pkg list | grep ur5
```

If `sudo rosdep init` reports that it was already initialized, continue with:

```bash
rosdep update
```

---

## 9. Final Verification Checklist

Before running the UR5 Digital Twin system, confirm the following:

- Ubuntu version is 22.04 Jammy:

```bash
lsb_release -a
```

- ROS distribution is Humble:

```bash
echo $ROS_DISTRO
```

- ROS 2 command line works:

```bash
ros2 --version
```

- CycloneDDS is active:

```bash
echo $RMW_IMPLEMENTATION
```

- Workspace is sourced:

```bash
echo $AMENT_PREFIX_PATH
```

- Project packages are visible:

```bash
ros2 pkg list | grep ur5
```

- Git repository is correctly cloned:

```bash
cd ~/ros2_ws/src/ur5_digital_twin
git status
git remote -v
```

If the expected UR5 packages appear and the workspace builds successfully, the development environment is fully configured and ready for operation.
