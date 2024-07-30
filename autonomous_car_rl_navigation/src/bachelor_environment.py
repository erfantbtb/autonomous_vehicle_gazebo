#!/usr/bin/python3

import gymnasium as gym 
from gymnasium import spaces 

import numpy as np 
from collections import deque

import rospy  
from camera_image_capture import CameraImageCapture
from imu_odometry_capture import ImuDataCapture
from spawn import Spawner
from pose import GazeboModelState
from std_msgs.msg import Float64


class AutonomousCarEnv(gym.Env):
    metadata = {"render_mode": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self, 
                 timeseries_length: int = 5, 
                 image_shape: tuple = (64, 64, 3), 
                 ) -> None:
        super(AutonomousCarEnv, self).__init__()

        # Define Hyperparameters
        self.timeseries_length = timeseries_length 
        self.image_shape = image_shape 
        
        # Define action space and observation space
        self.action_space = spaces.Box(low=4, high=10, shape=(2,), dtype=np.float32)
        
        self.video_size = (timeseries_length, *self.image_shape)
        self.imu_size = (timeseries_length, 6)
        self.observation_space = {
            "videos": spaces.Box(low=0, high=255, shape=self.video_size, dtype=np.float32),
            "imu": spaces.Box(low=-np.inf, high=np.inf, shape=self.imu_size, dtype=np.float32),
        }

        # Define observation initial 
        self.imu_data = deque(maxlen=timeseries_length)
        self.camera_image = deque(maxlen=timeseries_length)

        self.collision_with_sphere = False 
        self.reached_goal = False 
        self.collision_with_wall = False 

        self.distance_from_wall = 0.0 
        self.distance_from_obstacle = 0.0 
        self.distance_from_goal = 0.0 

        # Define Wheels publisher
        self.right_wheel_publisher = rospy.Publisher("/right_wheel_controller/command", Float64, queue_size=10)
        self.left_wheel_publisher = rospy.Publisher("/left_wheel_controller/command", Float64, queue_size=10)


    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.collision_with_sphere = False 
        self.reached_goal = False 
        self.collision_with_wall = False
        self.imu_data.clear()
        self.camera_image.clear()

        position_robot = [0, np.random.uniform(-6, 4), 0]
        position_obstacle = [np.random.uniform(10, 30), np.random.uniform(-6, 4), 0]
        position_goal = [40, np.random.uniform(-6, 4), 0]

        spawner = Spawner()
        spawner.spawn_robot("robot", position=position_robot)
        spawner.spawn_robot("unit_sphere", position=position_obstacle)
        spawner.spawn_robot("bachelor_model", position=position_goal)

        observation = self._get_state()
        info = self._get_info()

        return observation, info

    def _read_image_data(self) -> np.ndarray:
        """Private function for recieving latest camera image data

        Returns:
            np.ndarray: Image array 
        """
        camera_data_class = CameraImageCapture()
        return camera_data_class.get_latest_image() 
    
    def _read_imu_data(self) -> np.ndarray:
        """Private function for recieving latest Imu odometry data

        Returns:
            np.ndarray: Odometry array
        """
        imu_data_class = ImuDataCapture()
        return imu_data_class.get_latest_data()
    
    def _get_info(self):
        return {
            "goal_reached": self.reached_goal,
            "collision_with_sphere": self.collision_with_sphere,
            "collision_with_wall": self.collision_with_wall
        }
    
    def _get_state(self):
        # Get the latest IMU and camera data
        imu_data = self._read_imu_data()
        image_data = self._read_image_data()

        # Update the buffers
        self.imu_data.append(imu_data)
        self.camera_image.append(image_data)

        # Ensure buffers are filled initially
        if len(self.imu_data) < self.timeseries_length:
            for _ in range(self.timeseries_length - len(self.imu_data)):
                self.imu_data.append(imu_data)
                self.camera_image.append(image_data)
        
        # Create the observation
        imu_observation = np.array(self.imu_data)
        video_observation = np.array(self.camera_image)

        return {"videos": video_observation, "imu": imu_observation} 

    def _get_reward(self):
        self._get_distance_status()
        reward = 0.0 

        if self.collision_with_sphere:
            reward -= 100 

        if self.distance_from_goal <= 0.5: 
            reward += 100 

        reward += max(0, 10 - self.distance_from_goal)
        reward -= min(0, 10 - self.distance_from_obstacle)
        return reward

    def _get_collission_status(self):
        terminated, trancuated = False, False 

        if self.distance_from_goal <= 0.5:
            trancuated = True

        if self.distance_from_obstacle <= 1:
            terminated = True 
            self.collision_with_sphere == True 

        return terminated, trancuated

    def _get_distance_status(self): 
        pose = GazeboModelState() 
        self.distance_from_goal = pose.calculate_distance_to_model("robot", "bachelor_model")
        self.distance_from_obstacle = pose.calculate_distance_to_model("robot", "unit_sphere")

        self.distance_from_wall = min([pose.calculate_distance_to_link("robot", "Wall_0"), 
                                       pose.calculate_distance_to_link("robot", "Wall_1"), 
                                       pose.calculate_distance_to_link("robot", "Wall_4"), 
                                       pose.calculate_distance_to_link("robot", "Wall_5")])
        
        print(self.distance_from_goal, self.distance_from_obstacle, self.distance_from_wall)

    def _set_rpm_wheels(self, rpm_left, rpm_right) -> None:
        self.right_wheel_publisher.publish(rpm_right)
        self.left_wheel_publisher.publish(rpm_left)

    def step(self, action): 
        rpm_left, rpm_right = action 
        self._set_rpm_wheels(Float64(rpm_left), Float64(rpm_right))

        observation = self._get_state()
        terminated, trancuated = self._get_collission_status()
        reward = self._get_reward()
        info = self._get_info()

        return observation, reward, terminated, trancuated, info


if __name__ == "__main__":
    rospy.init_node("car_env_node", anonymous=True)
    env = AutonomousCarEnv()
    action = env.action_space.sample()
    print(action)
    state, reward, a, b, info = env.step(action)
    rospy.spin()
      

