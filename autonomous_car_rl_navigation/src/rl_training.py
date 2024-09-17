#!/usr/bin/python3

import gymnasium as gym 
from gymnasium import spaces 

import numpy as np 
from collections import deque
import matplotlib
import matplotlib.pyplot as plt
import torch 

import rospy  
from spawn import Spawner
from std_msgs.msg import Float64, Float32MultiArray
from autonomous_car_rl_navigation.src.autonomous_car_environment import AutonomousCarEnv


from stable_baselines3 import SAC
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import BaseCallback



if __name__ == "__main__":
    rospy.init_node("RL_training")
    env = AutonomousCarEnv()
    env = Monitor(env)
    vec_env = DummyVecEnv([lambda: env])
    rospy.sleep(5)

    # PPO Agent Configuration
    policy_kwargs = dict(net_arch=[256, 256])

    # Create and train the PPO agent
    model = SAC("MultiInputPolicy", vec_env, policy_kwargs=policy_kwargs, verbose=1)
    model.load("/home/erfan/catkin_ws/models/sac2_autonomous_car _100000_steps.zip")

