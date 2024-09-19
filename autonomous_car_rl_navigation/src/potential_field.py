#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
import math
import numpy as np


class ObstacleAvoidance:
    def __init__(self):
        # Initialize subscribers and publishers
        rospy.Subscriber('/obstacle_localization', Float32MultiArray, self.obstacle_callback)
        rospy.Subscriber('/goal_localization', Float32MultiArray, self.goal_callback)
        self.left_wheel_rpm_pub = rospy.Publisher('/left_wheel_controller/command', Float64, queue_size=10)
        self.right_wheel_rpm_pub = rospy.Publisher('/right_wheel_controller/command', Float64, queue_size=10)

        # Initialize variables
        self.obstacle_distance = None
        self.obstacle_angle = None
        self.goal_distance = None
        self.goal_angle = None
        self.base_rpm = 3  # Base RPM for the wheels

        # APF parameters
        self.k_att = 10.0
        self.k_rep = 100.0
        self.d0 = 4.0  # Threshold distance for repulsive force
        self.max_rep_force = 20.0

        self.wheel_base = 0.44
        self.wheel_radius = 0.0975

        self.max_speed = 5.0
        self.max_turn_rate = 0.3

        # Rate of execution
        self.rate = rospy.Rate(10)

    def obstacle_callback(self, data):
        # Data format: [distance, angle]
        self.obstacle_distance = data.data[0]
        self.obstacle_angle = data.data[1] * math.pi / 180  # Convert degrees to radians

    def goal_callback(self, data):
        # Data format: [distance, angle]
        self.goal_distance = data.data[0]
        self.goal_angle = data.data[1] * math.pi / 180  # Convert degrees to radians

    def attractive_force(self, distance, angle):
        """
        Calculate the attractive force towards the goal based on distance and angle.
        """
        # The attractive force magnitude is inversely proportional to the distance
        force_magnitude = self.k_att / distance
        # Convert polar to Cartesian force components
        fx = force_magnitude * math.cos(angle)
        fy = force_magnitude * math.sin(angle)
        return np.array([fx, fy])

    def repulsive_force(self, distance, angle):
        """
        Calculate the repulsive force from an obstacle based on distance and angle.
        """
        if distance > self.d0:
            # If the obstacle is far enough, no repulsive force is applied
            return np.array([0.0, 0.0])

        # The repulsive force magnitude decreases as distance increases, with a cutoff at d0
        force_magnitude = self.k_rep * ((1/distance) - (1/self.d0)) * (1/distance**2)
        force_magnitude = min(force_magnitude, self.max_rep_force)  # Cap the repulsive force
        # Convert polar to Cartesian force components
        fx = force_magnitude * math.cos(angle)
        fy = force_magnitude * math.sin(angle)
        return np.array([fx, fy])

    def compute_total_force(self):
        """
        Calculate the total force acting on the robot from the goal and obstacles.
        """
        total_force = np.array([0.0, 0.0])

        if self.goal_distance is not None and self.goal_distance > 0:
            # Add attractive force towards the goal
            total_force += self.attractive_force(self.goal_distance, self.goal_angle)

        if self.obstacle_distance is not None and self.obstacle_distance > 0:
            # Add repulsive force from the obstacle
            total_force -= self.repulsive_force(self.obstacle_distance, self.obstacle_angle)

        return total_force

    def compute_control(self):
        """
        Compute the linear and angular velocity of the robot based on the forces.
        """
        total_force = self.compute_total_force()
        print(math.atan2(total_force[1], total_force[0]))

        # Compute linear velocity (v) from the force magnitude
        v = 3.0

        # Compute angular velocity (w) from the force direction
        w = -np.clip(math.atan2(total_force[1], total_force[0]), -self.max_turn_rate, self.max_turn_rate)

        return v, w

    def run(self):
        while not rospy.is_shutdown():
            v, w = self.compute_control()
            self.rate.sleep()
            return v, w
        

# if __name__ == "__main__":
#     try:
#         rospy.init_node("ss", anonymous=True)
#         obs = ObstacleAvoidance()
#         obs.run()
    
#     except:
#         pass