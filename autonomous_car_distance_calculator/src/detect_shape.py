#!/usr/bin/python3

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32MultiArray


class NormalImageProcessor:
    def __init__(self) -> None:
        self.bridge = cv_bridge.CvBridge()
        self.normal_image = None
        self.coord_obstacle = None 
        self.coord_goal = None 

        normal_image_topic = "/depth_camera/color/image_raw"
        rospy.Subscriber(normal_image_topic, Image, self.normal_image_callback)

        self.goal_distance_publisher = rospy.Publisher("coordinate_of_goal", Float32MultiArray, queue_size=10)
        self.obstacle_distance_publisher = rospy.Publisher("coordinate_of_obstacle", Float32MultiArray, queue_size=10)
        self.msg = Float32MultiArray()
        rospy.loginfo(f"Subscribed to {normal_image_topic}")

    def normal_image_callback(self, data):
        try:
            self.normal_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 
            gray_image = cv2.cvtColor(self.normal_image, cv2.COLOR_BGR2GRAY) 
            
            _, thresh_image = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for i, contour in enumerate(contours):
                if i == 0:
                    continue

                epsilon = 0.01*cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                cv2.drawContours(self.normal_image, contour, 0, (0, 255, 0), 4)

                self.x, self.y, self.w, self.h = cv2.boundingRect(approx)
                x_mid = int(self.x + (self.w/3)) 
                y_mid = int(self.y + (self.h/1.5)) 

                self.center_coords = [x_mid, y_mid]
                colour = (0, 255, 0)
                font = cv2.FONT_HERSHEY_DUPLEX

                if len(approx) >= 4 and len(approx) <= 7:
                    cv2.putText(self.normal_image, "Goal", self.center_coords, font, 1, colour, 1)
                    self.coord_goal = self.center_coords
                    self.msg.data = self.coord_goal
                    self.goal_distance_publisher.publish(self.msg)

                elif len(approx) >= 10:
                    cv2.putText(self.normal_image, "Obstacle", self.center_coords, font, 1, colour, 1)
                    self.coord_obstacle = self.center_coords 
                    self.msg.data = self.coord_obstacle
                    self.obstacle_distance_publisher.publish(self.msg)

            self.visualize_normal_image()

        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def visualize_normal_image(self):
        if self.normal_image is not None:
            cv2.imshow("Normal Camera", self.normal_image)
            cv2.waitKey(1)

    def get_normal_image(self):
        if self.normal_image is not None:
            return self.normal_image
        

if __name__ == "__main__":
    rospy.init_node('depth_image_processor', anonymous=True)
    dip = NormalImageProcessor()
    rospy.spin()