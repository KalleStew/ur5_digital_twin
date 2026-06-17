# Development Environment Setup

This guide provides step-by-step instructions for configuring your local machine to run the UR5 Digital Twin and HIL platform. The system is designed for **Ubuntu 22.04 (Jammy Jellyfish)** and **ROS 2 Humble Hawksbill**.

You can run this project natively, inside a Virtual Machine, or via Windows Subsystem for Linux (WSL2).

---

## 1. Host Platform Configuration

Choose the setup that matches your deployment hardware.

### Option A: Windows Subsystem for Linux (WSL2) - *Recommended for Windows Hosts*
Windows 11 natively supports Linux GUI applications (WSLg), making it an excellent environment for running Gazebo and RViz without complex X11 server setups.
1. Open PowerShell as Administrator and install Ubuntu 22.04:
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```
2. Restart your computer if prompted.
3. Open the "Ubuntu 22.04" application from your start menu and create your UNIX username and password.
4. *Note: Ensure you allocate enough RAM to WSL by creating a `.wslconfig` file in your Windows `C:\Users\<YourName>` directory with at least `memory=8GB`.*

### Option B: Dedicated Virtual Machine (VMware / VirtualBox / Parallels)
If using a VM, 3D hardware acceleration is critical for the Gazebo physics engine.
1. Install a fresh Ubuntu 22.04 Desktop image.
2. **Crucial VM Settings:**
   * **RAM:** Minimum 8GB (16GB recommended).
   * **CPU:** Minimum 4 cores.
   * **Display:** Enable 3D Hardware Acceleration. Ensure Video Memory is maximized (e.g., 128MB or 256MB).

### Option C: Native Ubuntu 22.04
No preliminary host configuration is required. Ensure your system packages are up to date:
```bash
sudo apt update && sudo apt upgrade -y
```

---

## 2. ROS 2 Humble Installation

Once your Ubuntu 22.04 environment is running, open a terminal and install ROS 2 Humble.

**Step 1: Set Locale**
Ensure your system locale supports UTF-8.
```bash
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Step 2: Enable the Ubuntu Universe Repository**
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

**Step 3: Add the ROS 2 GPG Key and Repository**
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Step 4: Install ROS 2 Desktop**
This installs the core ROS 2 libraries, RViz2, and standard debugging tools.
```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

**Step 5: Automate Environment Sourcing**
Add the ROS 2 overlay to your `.bashrc` so it loads automatically in every new terminal.
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. System & Simulation Dependencies

Next, install the specific build tools, middleware, and physics engines required by this project.

**Step 1: Install Colcon and Developer Tools**
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep git tree -y
```

**Step 2: Install MoveIt 2 and Hardware Control Packages**
```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control -y
```

**Step 3: Configure High-Frequency Middleware (Optional but Recommended)**
By default, ROS 2 Humble uses FastRTPS. Because this project pumps raw torque arrays at 100Hz during the AI handoff, CycloneDDS is highly recommended to prevent dropped packets.
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. Workspace Initialization & Compilation

With the system dependencies installed, you can now build the digital twin workspace.

**Step 1: Create the Workspace Directory**
*(If you are cloning this from a Git repository, clone it into the `src` folder).*
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Step 2: Initialize and Update rosdep**
`rosdep` scans the `package.xml` files in the workspace and automatically installs any missing underlying system packages required to compile the custom controllers.
```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```
*(Note: If you see an error regarding `gazebo_ros_control`, it is a legacy ROS 1 artifact in the MoveIt config package that can be safely ignored for compilation).*

**Step 3: Build the Workspace**
Compile the C++ trajectory planners and register the Python control scripts.
```bash
cd ~/ros2_ws
colcon build --symlink-install
```
*Tip: `--symlink-install` allows you to edit your Python AI scripts and see the changes immediately without having to run `colcon build` again.*

**Step 4: Source the Local Workspace**
Append the workspace setup script to your `.bashrc` so your terminal knows where your custom packages live.
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Verification

To verify that your environment is set up correctly, check if ROS 2 can locate your custom packages:

```bash
ros2 pkg list | grep my_arm
```

You should see:
```text
ur5_controller
ur5_description
ur5_moveit_config
```
If you see these three packages, your development environment is fully configured and ready for operation!