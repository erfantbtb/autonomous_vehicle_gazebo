#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np

class ImuDataCapture:
    def __init__(self, topic_name='/imu', imu_size=(6,)):
        self.imu_subscriber = rospy.Subscriber(topic_name, Imu, self.imu_callback)
        self.imu_data = np.zeros(imu_size, dtype=np.float32)
        rospy.wait_for_message(topic_name, Imu)
        rospy.loginfo("IMU data subscriber initialized and received first message.")
    
    def imu_callback(self, data):
        self.imu_data = np.array([
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z
        ])
    
    def get_latest_data(self):
        return np.copy(self.imu_data)
