# robot_arm_pkg

## Overview
This ROS package provides control and simulation capabilities for a robotic arm. It includes functionality for movement control, simulation in Gazebo, and visualization in RViz. This package is part of an ongoing RAS UTSA research project.

## Features
- **Joint Position Control**: Control each joint of the robot arm using PID controllers.
- **Gazebo Simulation**: Test the robot arm in a simulated environment provided by Gazebo.
- **RViz Visualization**: Visualize the robot arm's state and movement in real-time with RViz.
- **Inverse Kinematics**: Utilize a fast IK solver, based on the [IK library](https://github.com/TheComet/ik), to calculate joint positions based on target end-effector positions.

## Dependencies
This package depends on several ROS packages and external libraries:
- [ros_control](http://wiki.ros.org/ros_control)
- [Gazebo ROS Packages](http://gazebosim.org/tutorials?tut=ros_overview)
- [RViz](http://wiki.ros.org/rviz)
- [joint_state_publisher](http://wiki.ros.org/joint_state_publisher)
- [robot_state_publisher](http://wiki.ros.org/robot_state_publisher)
- [IK library](https://github.com/TheComet/ik): A fast and flexible Inverse Kinematics library essential for the robotic arm's movement computation.

## Installation

### Prerequisites
Before installing the `robot_arm_pkg`, ensure you have the external IK library installed:
```bash
cd ~
mkdir -p libraries && cd libraries
git clone https://github.com/TheComet/ik.git
cd ik
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install