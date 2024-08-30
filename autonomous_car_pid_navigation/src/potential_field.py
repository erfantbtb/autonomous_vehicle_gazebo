#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float64
import math
import numpy as np

# Constants
K_REPEL = 1000.0  # Repulsive force gain
OBSTACLE_RADIUS = 8.0  # Radius around the robot to consider obstacles
WHEEL_BASE = 1.1  # Distance between the two front wheels in meters

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_with_rpm')

        # Initialize subscribers and publishers
        rospy.Subscriber('/obstacle_localization', Float32MultiArray, self.distance_callback)
        self.left_wheel_rpm_pub = rospy.Publisher('/left_wheel_controller/command', Float64, queue_size=10)
        self.right_wheel_rpm_pub = rospy.Publisher('/right_wheel_controller/command', Float64, queue_size=10)

        # Initialize variables
        self.obstacle_distance = None
        self.obstacle_angle = None
        self.desired_rpm_left = 3  # Example base RPM for left wheel
        self.desired_rpm_right = 3  # Example base RPM for right wheel

        # Rate of execution
        self.rate = rospy.Rate(10)

    def distance_callback(self, data):
        self.obstacle_distance = data.data[0]
        self.obstacle_angle = data.data[1] * math.pi / 180


    def compute_repulsive_force(self):

        if self.obstacle_distance is None or self.obstacle_angle is None:
            return 0.0, 0.0

        if self.obstacle_distance < OBSTACLE_RADIUS:
            repel_force_x = K_REPEL * (1.0 / self.obstacle_distance - 1.0 / OBSTACLE_RADIUS) * (1.0 / (self.obstacle_distance ** 2)) * np.cos(self.obstacle_angle)
            repel_force_y = K_REPEL * (1.0 / self.obstacle_distance - 1.0 / OBSTACLE_RADIUS) * (1.0 / (self.obstacle_distance ** 2)) * np.sin(self.obstacle_angle)

            # Adjust wheel RPM based on the repulsive force
            delta_rpm_left = -repel_force_x * WHEEL_BASE / 2
            delta_rpm_right = repel_force_y * WHEEL_BASE / 2
            print(delta_rpm_left, delta_rpm_right)

            return delta_rpm_left, delta_rpm_right

        return 0.0, 0.0

    def compute_rpm_command(self):
        # Get the base RPM commands
        rpm_left = self.desired_rpm_left
        rpm_right = self.desired_rpm_right

        # Calculate adjustments based on obstacle avoidance
        delta_rpm_left, delta_rpm_right = self.compute_repulsive_force()
        
        # Adjust the RPM commands
        adjusted_rpm_left = rpm_left + delta_rpm_left
        adjusted_rpm_right = rpm_right + delta_rpm_right

        return adjusted_rpm_left, adjusted_rpm_right

    def run(self):
        while not rospy.is_shutdown():
            # Compute the adjusted RPM commands
            adjusted_rpm_left, adjusted_rpm_right = self.compute_rpm_command()

            # Publish the RPM commands
            self.left_wheel_rpm_pub.publish(Float64(adjusted_rpm_left))
            self.right_wheel_rpm_pub.publish(Float64(adjusted_rpm_right))

            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        oa = ObstacleAvoidance()
        oa.run()

    except rospy.ROSInterruptException:
        pass
