#!/usr/bin/python3

import gymnasium as gym 
from gymnasium import spaces 

import numpy as np 
from collections import deque

import rospy  
from spawn import Spawner
from std_msgs.msg import Float64, Float32MultiArray

from stable_baselines3 import DQN
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback


class AutonomousCarEnv(gym.Env):
    metadata = {"render_mode": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self) -> None:
        super(AutonomousCarEnv, self).__init__()

        self.action_space = spaces.Discrete(9)
        self.observation_space = spaces.Dict({
            "goal_distance": spaces.Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
            "obstacle_distance": spaces.Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
            "goal_angle": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
            "obstacle_angle": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
        })

        self.collision_with_sphere = False 
        self.reached_goal = False 

        self.obstacle_distance = None   
        self.obstacle_angle = None
        self.goal_distance = None
        self.goal_angle = None

        self.current_rpm_left = 0.0 
        self.current_rpm_right = 0.0

        self.time_step = 0  

        self.right_wheel_publisher = rospy.Publisher("/right_wheel_controller/command", Float64, queue_size=10)
        self.left_wheel_publisher = rospy.Publisher("/left_wheel_controller/command", Float64, queue_size=10)

        rospy.Subscriber('/obstacle_localization', Float32MultiArray, self.distance_callback)
        rospy.Subscriber('/goal_localization', Float32MultiArray, self.goal_callback)
        rospy.sleep(0.5)

        self.action_map = {
            0: [-1, -1],
            1: [-1, 0],
            2: [-1, 1],
            3: [0, -1],
            4: [0, 0],
            5: [0, 1],
            6: [1, -1],
            7: [1, 0],
            8: [1, 1],
        }

    def distance_callback(self, data):
        self.obstacle_distance = data.data[0] 
        self.obstacle_angle = data.data[1] * np.pi / 180 if data.data[1] != -1 else None

    def goal_callback(self, data):
        self.goal_distance = data.data[0] 
        self.goal_angle = data.data[1] * np.pi / 180 if data.data[1] != -1 else None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.time_step = 0.0
        self.collision_with_sphere = False 
        self.reached_goal = False 

        position_robot = [0, np.random.uniform(-6, 4), 0]
        position_obstacle = [np.random.uniform(10, 30), np.random.uniform(-6, 4), 0]
        position_goal = [40, np.random.uniform(-6, 4), 0]

        spawner = Spawner()
        spawner.spawn_robot("robot", position=position_robot)
        spawner.spawn_robot("unit_sphere", position=position_obstacle)
        spawner.spawn_robot("unit_box", position=position_goal)

        observation = self._get_state()
        info = self._get_info()
        rospy.sleep(0.5)

        return observation, info
    
    def _get_state(self):
        goal_distance = self.goal_distance if self.goal_distance is not None else None
        goal_angle = self.goal_angle if self.goal_angle is not None else 0.0
        obstacle_distance = self.obstacle_distance if self.obstacle_distance is not None else None
        obstacle_angle = self.obstacle_angle if self.obstacle_angle is not None else 0.0

        # Construct the observation dictionary
        observation = {
            "goal_distance": np.array([goal_distance], dtype=np.float32),
            "obstacle_distance": np.array([obstacle_distance], dtype=np.float32),
            "goal_angle": np.array([goal_angle], dtype=np.float32),
            "obstacle_angle": np.array([obstacle_angle], dtype=np.float32),
        }

        return observation
    
    def _get_info(self):
        return {
            "goal_reached": self.reached_goal,
            "collision_with_sphere": self.collision_with_sphere,
        }
    
    def _get_reward(self):
        self._get_state()
        reward = 0.0

        # Gravitational potential reward (pulling towards goal)
        if self.goal_distance != -1:
            print("hadaf")
            old_goal_distance = self.goal_distance
            gravitational_potential = 0.5 * 0.9 * ((self.goal_distance) ** 2)
            reward -= gravitational_potential

        if self.obstacle_distance <= 10 and self.obstacle_distance!= -1:
            print("mane")
            self.goal_distance = self.goal_distance if not -1 else 1e5
            repulsive_potential = 0.9 * self.goal_distance * np.exp(-self.obstacle_distance ** 2)
            reward -= repulsive_potential

        # if self.goal_distance == -1:
        #     gravitational_potential = 0.5 * 0.9 * ((old_goal_distance) ** 2)
        #     reward -= gravitational_potential 


        reward -= 0.001 * self.time_step

        return reward

    def _get_collission_status(self):
        terminated, truncated = False, False 

        obstacle_distance = self.obstacle_distance 
        goal_distance = self.goal_distance 

        if goal_distance <= 1 and goal_distance >= 0.0:
            truncated = True
            self.reached_goal = True

        if obstacle_distance <= 0.3 and obstacle_distance >= 0.0: 
            terminated = True 
            self.collision_with_sphere = True 

        if self.time_step >= 300:
            terminated = True

        # if self.goal_distance is None and self.obstacle_distance is None:
        #     terminated = True 

        return terminated, truncated


    def _set_rpm_wheels(self, rpm_left, rpm_right) -> None:
        right_speed = Float64(rpm_right)        
        left_speed = Float64(rpm_left) 
        self.right_wheel_publisher.publish(right_speed)
        self.left_wheel_publisher.publish(left_speed)

    def step(self, action): 
        rospy.sleep(0.1)
        
        rpm_left_change, rpm_right_change = self.action_map[action]
        
        # Update the current RPMs
        if self.current_rpm_left + rpm_left_change <= 7 and self.current_rpm_left + rpm_left_change >= 0:
            self.current_rpm_left += rpm_left_change
        
        if self.current_rpm_right + rpm_right_change <= 7 and self.current_rpm_right + rpm_right_change >= 0:
            self.current_rpm_right += rpm_right_change

        # Set the wheel speeds
        self._set_rpm_wheels(self.current_rpm_left, self.current_rpm_right)

        observation = self._get_state()
        terminated, truncated = self._get_collission_status()
        reward = self._get_reward()
        print(reward)
        info = self._get_info()

        self.time_step += 1  

        return observation, reward, terminated, truncated, info
    
    def render(self, mode="human"):
        pass

    def close(self):
        rospy.signal_shutdown("Closing AutonomousCarEnv environment.")

    
if __name__ == "__main__":
    rospy.init_node("RL_training")
    env = AutonomousCarEnv() 
    vec_env = make_vec_env(lambda: env, n_envs=1)
    env = Monitor(env)
    rospy.sleep(5)

    # Parameters
    Tobs = 200
    Ttrain = 800
    Tepisode = 300
    Trenew = 20
    alpha = 1e-3  
    gamma = 0.9  
    KG = 0.9  
    KR = 0.9  

    # DQN Agent Configuration
    policy_kwargs = dict(net_arch=[120, 120])  # Two hidden layers with 60 neurons each

    # Create and train the DQN agent
    model = DQN(
        policy="MultiInputPolicy", 
        env=env, 
        learning_rate=alpha, 
        gamma=gamma, 
        target_update_interval=Trenew, 
        policy_kwargs=policy_kwargs, 
        verbose=1
    )

    # Training process
    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./models/', name_prefix='dqn_autonomous_car')
    model.learn(total_timesteps=10000, callback=checkpoint_callback, progress_bar=True)

    # Evaluate the agent
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward}, Std reward: {std_reward}")

    # Save the trained model
    model.save("dqn_autonomous_car")

    env.close()


      
