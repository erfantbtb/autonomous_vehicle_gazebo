#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates
import time

class Moving():
    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.direction = 1  # 1 for forward, -1 for backward
        self.change_direction_interval = 2.0  # Change direction every 3 seconds
        self.last_change_time = time.time()
        self.initial_y_position = None
        self.moving()

    def moving(self):
        while not rospy.is_shutdown():
            obstacle = ModelState()
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            current_time = time.time()

            for i in range(len(model.name)):
                if model.name[i] == 'unit_sphere':  # Updated name
                    obstacle.model_name = 'unit_sphere'  # Updated name
                    obstacle.pose = model.pose[i]

                    if self.initial_y_position is None:
                        self.initial_y_position = obstacle.pose.position.y

                    # Check if it's time to change direction
                    if current_time - self.last_change_time >= self.change_direction_interval:
                        self.direction *= -1  # Reverse direction
                        self.last_change_time = current_time  # Reset the last change time

                    # Move the sphere in the current direction along the Y-axis
                    obstacle.twist = Twist()
                    obstacle.twist.linear.x = 0.0
                    obstacle.twist.linear.y = -2 * self.direction
                    obstacle.twist.angular.z = 0.0
                    self.pub_model.publish(obstacle)
                    
                    self.wait_for_sim_time(0.1)

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
