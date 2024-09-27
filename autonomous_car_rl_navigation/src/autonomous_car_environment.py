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
from geometry_msgs.msg import Twist

from stable_baselines3 import SAC
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import BaseCallback
from potential_field import ObstacleAvoidance



class AutonomousCarEnv(gym.Env):
    metadata = {"render_mode": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self) -> None:
        super(AutonomousCarEnv, self).__init__()

        # Update action space to discrete changes in angular velocity
        self.action_space = spaces.Discrete(3)  # Continuous action space for angular velocity


        # Update observation space to include linear and angular velocities
        self.observation_space = spaces.Dict({
            "goal_distance": spaces.Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
            "obstacle_distance": spaces.Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
            "goal_angle": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
            "obstacle_angle": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
            "linear_speed": spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32),
            "angular_velocity": spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32),
        })

        self.collision_with_sphere = False
        self.reached_goal = False

        self.obstacle_distance = -1
        self.obstacle_angle = -1
        self.goal_distance = -1
        self.goal_angle = -1
        self.previous_goal_distance = -1
        self.previous_goal_angle = -1

        self.current_linear_speed = 4.0
        self.current_angular_velocity = 0.0
        self.wheel_base = 0.44 
        self.wheel_radius = 0.0975

        self.apf = ObstacleAvoidance()
        self.training_mode = True
        self.rate = rospy.Rate(10)

        self.time_step = 0
        # self.right_wheel_publisher = rospy.Publisher("/right_wheel_controller/command", Float64, queue_size=10)
        # self.left_wheel_publisher = rospy.Publisher("/left_wheel_controller/command", Float64, queue_size=10)

        self.velocity_msg = Twist()
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber('/obstacle_localization', Float32MultiArray, self.distance_callback)
        rospy.Subscriber('/goal_localization', Float32MultiArray, self.goal_callback)

        self.action_map = {
            0: -0.1, 
            1: 0.0, 
            2: 0.1
        }
        rospy.sleep(0.5)


    def distance_callback(self, data):
        self.obstacle_distance = data.data[0] if data.data[0] != -1 and data.data[0] is not None else -1
        self.obstacle_angle = data.data[1] if data.data[1] != -1 and data.data[1] is not None else -1

    def goal_callback(self, data):
        self.goal_distance = data.data[0] if data.data[0] != -1 and data.data[0] is not None else -1
        self.goal_angle = data.data[1] if data.data[1] != -1 and data.data[1] is not None else -1

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.time_step = 0.0
        self.collision_with_sphere = False
        self.reached_goal = False

        position_robot = [0, np.random.uniform(-3, 3), 0]
        position_goal = [32, np.random.uniform(-4, 4), 0]
        position_obstacle = [np.random.uniform(10, 22), (position_robot[1] + position_goal[1]) / 2, 0]

        spawner = Spawner()
        spawner.spawn_robot("robot", position=position_robot)
        spawner.spawn_robot("unit_sphere", position=position_obstacle)
        spawner.spawn_robot("Untitled", position=position_goal)
        
        rospy.sleep(1.5)
        observation = self._get_state()
        info = self._get_info()
        

        return observation, info

    def _get_state(self):
        goal_distance = self.goal_distance if self.goal_distance is not None else -1
        goal_angle = self.goal_angle if self.goal_angle is not None else -1
        obstacle_distance = self.obstacle_distance if self.obstacle_distance is not None else -1
        obstacle_angle = self.obstacle_angle if self.obstacle_angle is not None else -1
        linear_speed = self.current_linear_speed
        angular_velocity = self.current_angular_velocity

        # Construct the observation dictionary
        observation = {
            "linear_speed": np.array([linear_speed], dtype=np.float32),
            "angular_velocity": np.array([angular_velocity], dtype=np.float32),
            "goal_distance": np.array([goal_distance if not np.isnan(goal_distance) else -1], dtype=np.float32),
            "obstacle_distance": np.array([obstacle_distance if not np.isnan(obstacle_distance) else -1], dtype=np.float32),
            "goal_angle": np.array([goal_angle if not np.isnan(goal_angle) else -1], dtype=np.float32),
            "obstacle_angle": np.array([obstacle_angle if not np.isnan(obstacle_angle) else -1], dtype=np.float32),
        }

        return observation

    def _get_info(self):
        return {
            "goal_reached": self.reached_goal,
            "collision_with_sphere": self.collision_with_sphere,
        }

    def _get_reward(self):
        reward = 0.0

        # Reward for approaching the goal (only when in range)
        if self.goal_distance != -1:
            if self.goal_distance > 1.2:  # When far from the goal
                reward += 30 / (self.goal_distance + 1)  # Reward scales over larger distances
                reward -= 0.01 * abs(self.goal_angle)
            elif 0.0 <= self.goal_distance <= 1.2:  # Close to the goal
                reward += 150 / (1 + self.time_step) + 100  # High reward for reaching the goal

        # Penalty for when goal is out of range (goal_distance == -1)
        if self.goal_distance == -1:
            reward -= 100  # Moderate penalty for no progress

        # Penalty for being too close to obstacles (when detected)
        # if self.obstacle_distance != -1:
        #     if self.obstacle_distance <= 1.0:  # Danger zone
        #         reward -= 100  # Strong penalty for close proximity to obstacles
        #     elif 1.0 <= self.obstacle_distance < 4:  # Medium penalty for proximity
        #         reward -= 50 / (self.obstacle_distance + 1)

        #     elif 5 <= self.obstacle_distance < 15:  # Light penalty for medium distance
        #         reward -= 10 / (self.obstacle_distance + 1)
        #     elif 15 <= self.obstacle_distance <= 30:  # Safe zone
        #         reward += 5 / (self.obstacle_distance + 1)  # Small reward for maintaining safe distance

        # Time penalty to encourage quicker solutions
        reward -= 0.02 * self.time_step  # Slightly stronger time penalty

        # Ensure reward is well-defined
        if reward is None or np.isnan(reward):
            reward = -1  # Penalize invalid states

        return reward


    def _get_collission_status(self):
        terminated, truncated = False, False

        obstacle_distance = self.obstacle_distance
        goal_distance = self.goal_distance

        if goal_distance <= 1.2 and goal_distance >= 0.0:
            print("reached the goal!")
            truncated = True
            self.reached_goal = True

        if obstacle_distance <= 0.5 and obstacle_distance >= 0.0:
            print("collision with obstacle")
            terminated = True
            self.collision_with_sphere = True

        if self.time_step >= 300:
            print("Out of time")
            terminated = True

        if self.goal_distance == -1:
            print("No goal is here")
            terminated = True

        return terminated, truncated

    def _set_rpm_wheels(self, rpm_left, rpm_right) -> None:
        right_speed = Float64(rpm_right)        
        left_speed = Float64(rpm_left) 
        self.right_wheel_publisher.publish(right_speed)
        self.left_wheel_publisher.publish(left_speed)

    def _set_velocity(self, v, wz):
        self.velocity_msg.linear.x = v 
        self.velocity_msg.angular.z = wz 
        self.velocity_publisher.publish(self.velocity_msg)

    def step(self, action):
        rospy.sleep(0.1)
        self.rate.sleep()
        
        if self.obstacle_distance > 4 or self.obstacle_distance == -1:
            print("RL method is running")
            wz = self.action_map[action]
            v = self.current_linear_speed
            self._set_velocity(v, wz)
            print(f"linear velocity is {v} and angular velocity is {wz}")

        # rpm_left, rpm_right = self._calculate_rpm(self.current_linear_speed, self.current_angular_velocity)
        # rpm_left, rpm_right = np.round(rpm_left, 2), np.round(rpm_right, 2)
        # print(f"right wheel rpm is {rpm_right} and left rpm is {rpm_left}")

        if self.training_mode and self.obstacle_distance != -1 and self.obstacle_distance <= 4:
            print("Obstacle Avoidance Method is running")
            v, wz = self.apf.run()
            self._set_velocity(v, wz)
            print(f"linear velocity is {v} and angular velocity is {wz}")
            print("-------------------------------------------")
        # self._set_rpm_wheels(rpm_left, rpm_right)

        observation = self._get_state()
        reward = self._get_reward()

        print(f"reward is {reward}")
        terminated, truncated = self._get_collission_status()

        info = self._get_info()

        self.time_step += 1
        self.previous_goal_distance = self.goal_distance
        self.previous_goal_angle = self.goal_angle

        return observation, reward, terminated, truncated, info

    def _calculate_rpm(self, linear_speed, angular_velocity):
        rpm_left = (2*linear_speed + angular_velocity*self.wheel_base) / (2*self.wheel_radius)
        rpm_right = (2*linear_speed - angular_velocity*self.wheel_base) / (2*self.wheel_radius)
        
        return rpm_left, rpm_right

    def render(self, mode="human"):
        pass

    def close(self):
        rospy.signal_shutdown("Closing AutonomousCarEnv environment.")



if __name__ == "__main__":
    rospy.init_node("RL_training")
    env = AutonomousCarEnv()  # Your custom environment
    env = Monitor(env)
    vec_env = DummyVecEnv([lambda: env])  # Vectorized environment
    rospy.sleep(5)

    # Set up matplotlib for plotting
    is_ipython = 'inline' in plt.get_backend()
    if is_ipython:
        from IPython import display

    plt.ion()

    # Callback to handle plotting rewards
    class PlotCallback(BaseCallback):
        def __init__(self, plot_freq=100, verbose=1):
            super(PlotCallback, self).__init__(verbose)
            self.plot_freq = plot_freq
            self.rewards = []

        def _on_step(self) -> bool:
            if self.n_calls % self.plot_freq == 0:
                # Append the cumulative reward
                self.rewards.append(self.model.ep_info_buffer[-1]['r'])

                # Convert rewards to tensor
                rewards_t = torch.tensor(self.rewards, dtype=torch.float)

                # Clear the current figure
                plt.clf()
                plt.title('Training...')
                plt.xlabel('Episode')
                plt.ylabel('Reward')
                plt.plot(rewards_t.numpy(), label="Rewards")

                # Take 100 episode averages and plot them too
                if len(rewards_t) >= 100:
                    means = rewards_t.unfold(0, 100, 1).mean(1).view(-1)
                    means = torch.cat((torch.zeros(99), means))
                    plt.plot(means.numpy(), label="100-episode average")

                plt.legend()
                plt.pause(0.001)  # pause a bit so that plots are updated
                plt.savefig("/home/erfan/catkin_ws/src/autonomous_car_rl_navigation/results3")

                if is_ipython:
                    display.display(plt.gcf())
                    display.clear_output(wait=True)

            return True

    # PPO Agent Configuration
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
    # model.load("/home/erfan/catkin_ws/models/ppo_autonomous_car_60000_steps.zip")
    # Initialize the PlotCallback
    plot_callback = PlotCallback(plot_freq=100)

    # # Training process
    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='/home/erfan/catkin_ws/models/', name_prefix='ppo_last1')
    model.learn(total_timesteps=100000, callback=[checkpoint_callback, plot_callback], progress_bar=True)

    # Evaluate the agent
    mean_reward, std_reward = evaluate_policy(model, vec_env, n_eval_episodes=100)
    print(f"Mean reward: {mean_reward}, Std reward: {std_reward}")

    # # Save the trained model
    model.save("ppo_last1")

    env.close()



      
