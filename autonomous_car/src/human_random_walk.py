#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates
import random


class Moving():
    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.moving()

    def moving(self):

        while not rospy.is_shutdown():
            obstacle = ModelState()
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            
            for i in range(len(model.name)):
                
                if model.name[i] == 'unit_sphere':
                    obstacle.model_name = 'unit_sphere'
                    obstacle.pose = model.pose[i]

                    obstacle.twist = Twist()
                    obstacle.twist.linear.x = random.choice([-2, 2])
                    obstacle.twist.linear.y = random.choice([-2, 2])
                    obstacle.twist.angular.z = random.uniform(-2.0, 2.0)
                    self.pub_model.publish(obstacle)
                    self.wait_for_sim_time(3)

    def wait_for_sim_time(self, duration):
        """Wait for a specific duration in Gazebo simulation time."""
        start_time = rospy.get_rostime()
        end_time = start_time + rospy.Duration(duration)

        while rospy.get_rostime() < end_time:
            rospy.sleep(0.1) 

def main():
    rospy.init_node('moving_obstacle')
    moving = Moving()

if __name__ == '__main__':
    main()
