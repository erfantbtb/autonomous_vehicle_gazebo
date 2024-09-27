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
from autonomous_car_environment import AutonomousCarEnv


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

    policy_kwargs = dict(net_arch=[128, 64, 32])

    # Create and train the PPO agent
    model = PPO(
        "MultiInputPolicy",
        vec_env,
        policy_kwargs=policy_kwargs,
        verbose=1,
        batch_size=64,          # Batch size for each gradient update
        learning_rate=3e-3,     # Learning rate
        gamma=0.95,
        )
    model.load("/home/erfan/catkin_ws/models/ppo_last_74000_steps.zip")
    mean_reward, std_reward = evaluate_policy(model, vec_env, n_eval_episodes=100)
    env.close()

