# Relevant External Documentation

## 1. ROS 2 Core (Jazzy)

* [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
* [Jazzy installation on Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
* [colcon](https://colcon.readthedocs.io/)

## 2. MoveIt 2 & Kinematics

* [MoveIt 2 for Jazzy](https://moveit.picknik.ai/main/index.html)
* [MoveIt configuration](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)
* [Orocos KDL](https://www.orocos.org/kdl.html)
* [python_orocos_kdl](https://github.com/orocos/orocos_kinematics_dynamics)

## 3. Simulation & Physics (Gazebo)

* [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
* [ros_gz](https://github.com/gazebosim/ros_gz)
* [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)

## 4. Control Systems & Hardware-in-the-Loop

* [ros2_control](https://control.ros.org/jazzy/index.html)
* [joint_trajectory_controller](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
* [Elmo Platinum Maestro](https://www.elmomc.com/) — the intended EtherCAT drive. The sim hardware plugin is `gz_ros2_control/GazeboSimSystem` until that interface exists.
* EtherCAT: the hardware roadmap in [development workflow](development_workflow.md) keeps `platform.yaml` as the topic seam.

## 5. Middleware & Networking

* [rmw_zenoh](https://github.com/ros2/rmw_zenoh)
* [Zenoh](https://zenoh.io/)
* This platform sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and runs `rmw_zenohd`. Do not switch RMW to hide a discovery fault.

## 6. Logging

* [h5py](https://docs.h5py.org/)
* HDF5 files from this platform are raw N×7 datasets plus attributes. `ur5 validate` is the checker. It does not resample or filter.
