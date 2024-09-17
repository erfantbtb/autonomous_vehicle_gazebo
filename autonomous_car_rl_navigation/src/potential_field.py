#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
import math


class ObstacleAvoidance:
    def __init__(self):
        # rospy.init_node('obstacle_avoidance_with_rpm')

        # Initialize subscribers and publishers
        rospy.Subscriber('/obstacle_localization', Float32MultiArray, self.distance_callback)
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
        self.obstacle_weight = 25.25
        self.goal_weight = 70.0
        self.max_speed = 5.0
        self.max_turn_rate = 4.0

        self.wheel_base = 0.44 
        self.wheel_radius = 0.0975

        # Rate of execution
        self.rate = rospy.Rate(10)

    def distance_callback(self, data):
        self.obstacle_distance = data.data[0]
        self.obstacle_angle = data.data[1] * math.pi / 180  # Convert angle to radians

    def goal_callback(self, data):
        self.goal_distance = data.data[0]
        self.goal_angle = data.data[1] * math.pi / 180  # Convert angle to radians

    def compute_control(self):
        # Initialize control parameters
        v = 0
        w = 0

        # Check if goal and obstacle data are available
        if self.goal_distance is not None and self.obstacle_distance is not None and self.goal_distance != -1 and self.obstacle_distance != -1:
            # Compute attractive force towards the goal
            if self.goal_distance > 1:
                goal_force = self.goal_weight / self.goal_distance
                v += min(self.max_speed, goal_force)
                w += self.goal_weight * self.goal_angle

            # Compute repulsive force from the obstacle
            if self.obstacle_distance > 1:
                obstacle_force = self.obstacle_weight / (self.obstacle_distance ** 2)
                v -= min(self.max_speed, obstacle_force)
                w -= self.obstacle_weight * self.obstacle_angle

            # Normalize angular velocity
            w = max(-self.max_turn_rate, min(self.max_turn_rate, w))

        elif self.goal_distance is not None and self.goal_distance != -1:
            # Only goal is detected
            if self.goal_distance > 0:
                goal_force = self.goal_weight / self.goal_distance
                v += min(self.max_speed, goal_force)
                w += self.goal_weight * self.goal_angle

        elif self.obstacle_distance is not None and self.obstacle_distance != -1:
            # Only obstacle is detected
            if self.obstacle_distance > 0:
                obstacle_force = self.obstacle_weight / (self.obstacle_distance ** 2)
                v -= min(self.max_speed, obstacle_force)
                w -= self.obstacle_weight * self.obstacle_angle

        else:
            v = 0
            w = 0

        return v, w

    def run(self):
        while not rospy.is_shutdown():
            v, w = self.compute_control()

            # Simple kinematic model to set left and right wheel RPMs
            # Adjust these formulas according to your robot's kinematics
            right_rpm = (2*v + w*self.wheel_base) / (2*self.wheel_radius)
            left_rpm = (2*v - w*self.wheel_base) / (2*self.wheel_radius)
  

            print(left_rpm, right_rpm)

            # # Publish RPM commands
            # self.left_wheel_rpm_pub.publish(Float64(left_rpm))
            # self.right_wheel_rpm_pub.publish(Float64(right_rpm))

            self.rate.sleep()

            return right_rpm, left_rpm
        

# if __name__ == "__main__":
#     try:
#         rospy.init_node("ss", anonymous=True)
#         obs = ObstacleAvoidance()
#         obs.run()
    
#     except:
#         pass