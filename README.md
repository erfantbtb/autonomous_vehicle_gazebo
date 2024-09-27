# Autonomous Vehicel 

## Overview 
This repository contains a simulation of an autonomous 4 wheeled scooter using Gazebo and ROS (Robot Operating System). The vehicle is equipped with sensors such as RGBD camera, and IMU to perform localization, mapping, and path planning tasks. It is designed to operate in a dynamic environment with moving obstacles and stationary barriers.
For path planning and obstacle avoidance we used RL + APF method to achieve our goal.we didnt need localization here because we had goal and obstacle distance and angle with respect to the car.

<!-- <p align="center">
  <img src="/home/erfan/catkin_ws/src/images_readme/" width="350" title="Method Used">
</p> -->

## Features 
**Obstacle Avoidance**: Real-time detection and avoidance of obstacles using RGBD-camera data.
**Path Planning**: Combining artificial potential field and  reinforcement learning for dynamic environments.
**Goal Reaching**: Real-time detection and reaching of the goal using RGBD-camera data 
**Gazebo Integration**: Simulates the vehicle's movement and sensor data in a realistic 3D environment.

## Installation 
To run the simulation, ensure you have the following prerequisites installed:

1. **ROS (Robot Operating System)** - [ROS Noetic](http://wiki.ros.org/noetic/Installation) (recommended version)
2. **Gazebo** - [Gazebo Simulator](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
3. **Python** (Version 3.x)
4. **Other Dependencies** - Refer to the [requirements.txt](./requirements.txt) file or below for specific libraries

### Clone the Repository
```bash
git clone https://github.com/your-username/autonomous-vehicle-gazebo.git
cd autonomous-vehicle-gazebo

