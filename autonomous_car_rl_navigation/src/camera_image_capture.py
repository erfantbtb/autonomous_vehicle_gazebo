#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class CameraImageCapture:
    def __init__(self, topic_name="/mobile_robot_description/camera1/image_raw", image_shape=(64, 64, 3)):
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber(topic_name, Image, self.image_callback)
        self.image_shape = image_shape
        self.current_image = np.zeros(self.image_shape, dtype=np.float32)
        rospy.wait_for_message(topic_name, Image)
        rospy.loginfo("Camera data subscriber initialized and received first message.")
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.resize(cv_image, dsize=(64, 64))
            self.current_image = cv_image / 255.0  
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def get_latest_image(self):
        return np.copy(self.current_image)
