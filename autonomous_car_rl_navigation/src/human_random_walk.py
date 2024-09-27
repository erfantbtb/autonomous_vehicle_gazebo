#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates
import time
import math  # For sine function
import random


class Moving():
    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.constant_x_speed = 4.0  # Constant speed along X-axis
        self.max_y_speed = 2.0  # Maximum speed along Y-axis (for the sine wave)
        self.frequency = 0.5  # Frequency of the sine wave (in rad/s)
        self.direction = 1
        self.last_change_time = time.time()
        self.change_direction_interval = 2.0  # Change direction every 3 seconds
        self.initial_time = time.time()  # Record the start time
        self.moving()

    def moving(self):
        while not rospy.is_shutdown():
            obstacle = ModelState()
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            current_time = time.time()
            time_elapsed = current_time - self.initial_time  # Calculate time since start

            for i in range(len(model.name)):
                if model.name[i] == 'unit_sphere':  # Update to match model name
                    obstacle.model_name = 'unit_sphere'  # Updated model name
                    obstacle.pose = model.pose[i]

                    if current_time - self.last_change_time >= self.change_direction_interval:
                        self.direction *= -1  # Reverse direction
                        self.last_change_time = current_time  # Reset the last change time

                    # Calculate the Y-axis velocity as a sinusoidal function
                    x_velocity = self.max_y_speed * math.sin(self.frequency * time_elapsed)

                    # Set velocity in the X-axis (constant) and Y-axis (sinusoidal)
                    obstacle.twist = Twist()
                    obstacle.twist.linear.x =  x_velocity # Constant X velocity
                    obstacle.twist.linear.y = self.constant_x_speed * self.direction  # Sinusoidal Y velocity
                    obstacle.twist.angular.z = 0.0  # No rotation

                    # Publish the updated obstacle state with the new velocities
                    self.pub_model.publish(obstacle)

                    self.wait_for_sim_time(0.1)  # Adjust the update frequency

    def wait_for_sim_time(self, duration):
        """Wait for a specific duration in Gazebo simulation time."""
        start_time = rospy.get_rostime()
        end_time = start_time + rospy.Duration(duration)

        while rospy.get_rostime() < end_time:
            rospy.sleep(0.01)

def main():
    rospy.init_node('moving_obstacle')
    moving = Moving()

if __name__ == '__main__':
    main()