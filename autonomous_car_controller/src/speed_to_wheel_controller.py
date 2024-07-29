#!/usr/bin/python3

import rospy 
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class ForwardDifferenceKinematic:
    def __init__(self, wheel_radius: float = 0.25, car_width: float = 1) -> None:
        self.wheel_radius = wheel_radius 
        self.car_width = car_width 
        
        self.right_wheel_publisher = rospy.Publisher("right_wheel_controller/command", Float64, queue_size=10)
        self.left_wheel_publisher = rospy.Publisher("left_wheel_controller/command", Float64, queue_size=10)

        self.vel_sub = rospy.Subscriber("autonomous_car_controller/cmd_vel", Twist, self.call_back)
        
        self.speed_conversion = np.array([[wheel_radius/2, wheel_radius/2], 
                                          [wheel_radius/car_width, -wheel_radius/car_width]])
        
    def call_back(self, msg):
        car_speed = np.array([[msg.linear.x], 
                              [msg.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), car_speed)
        right_speed = Float64(wheel_speed[0, 0])        
        left_speed = Float64(wheel_speed[1, 0]) 
        rospy.loginfo(right_speed)
        
        self.right_wheel_publisher.publish(right_speed)
        self.left_wheel_publisher.publish(left_speed)
        
        
if __name__ == "__main__":
    rospy.init_node("simple_controller_node", anonymous=True)
    ForwardDifferenceKinematic()
    rospy.spin()
