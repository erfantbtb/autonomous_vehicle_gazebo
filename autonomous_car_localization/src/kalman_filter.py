#!/usr/bin/python3

import rospy 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class KalmanFilter:
    def __init__(self) -> None:
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imuCallback)
        self.odom_pub = rospy.Publisher("autonomous_controller/odom_kalman", Odometry, queue_size=10)

        self.mean = 0.0 
        self.variance = 1000.0 
        self.imu_angular_z = 0.0
        self.is_first_odom = True
        self.last_angular_z = 0.0 
        self.motion = 0.0 
        self.kalman_odom = Odometry()

    def imuCallback(self, imu):
        self.imu_angular_z = imu.angular_velocity.z 
