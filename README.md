# Autonomous Vehicel 

## Overview 
This repository contains a simulation of an autonomous 4 wheeled scooter using Gazebo and ROS (Robot Operating System). The vehicle is equipped with sensors such as RGBD camera, and IMU to perform localization, mapping, and path planning tasks. It is designed to operate in a dynamic environment with moving obstacles and stationary barriers.
For path planning and obstacle avoidance we used RL + APF method to achieve our goal.we didnt need localization here because we had goal and obstacle distance and angle with respect to the car.

<p align="center">
  <img src="https://github.com/erfantbtb/autonomous_vehicle_gazebo/blob/main/images_readme/Screenshot from 2024-09-27 17-53-20.png" width="350" title="Scooter">
</p>

## Features 
**Obstacle Avoidance**: Real-time detection and avoidance of obstacles using RGBD-camera data.   
**Path Planning**: Combining artificial potential field and  reinforcement learning for dynamic environments.  
**Goal Reaching**: Real-time detection and reaching of the goal using RGBD-camera data.  
**Gazebo Integration**: Simulates the vehicle's movement and sensor data in a realistic 3D environment.  
<p align="center">
  <img src="https://github.com/erfantbtb/autonomous_vehicle_gazebo/blob/main/images_readme/gazebo.png" width="350" title="Scooter">
</p>

## Installation 
To run the simulation, ensure you have the following prerequisites installed:

1. **ROS (Robot Operating System)** - [ROS Noetic](http://wiki.ros.org/noetic/Installation) (recommended version)
2. **Gazebo** - [Gazebo Simulator](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
3. **Python** (Version 3.x)
4. **Other Dependencies** - Refer to the [requirements.txt](./requirements.txt) file or below for specific libraries

### Clone the Repository
```bash
git clone https://github.com/erfantbtb/autonomous_vehicle_gazebo.git
``` 

## Architecture
The project consists of several key components:
1. **Vehicle Model**: A URDF model describing the physical parameters of the 4-wheeled Scooter.
2. **Sensor Integration**: RGBD camera, and IMU sensors for environment perception.
3. **Control**: Gazebo skid-steering plugin to controll wheel actuators
4. **Reinforcement Learning + Artificial Potential field** : A custom Gym-ROS environment used to train the vehicle for obstacle avoidance and goal reaching.

## Future Improvements
1. Add more advanced sensor fusion algorithms for better localization and detection.
2. integrate a controller like MPC for smoother path following.
3. Improve the reinforcement learning model for better performance in dynamic environments.
4. Expand the vehicle's capabilities to handle more complex tasks, such as parking or overtaking.
5. Tune APF functions parameters 

## Contributors
[Erfan Tabatabaei](https://github.com/erfantbtb)- Main Developer

## License
This project is licensed under the MIT License - see the [LICENSE](https://github.com/erfantbtb/autonomous_vehicle_gazebo/blob/main/LICENSE) file for details.

