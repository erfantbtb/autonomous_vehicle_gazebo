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
            # Convert the ROS image message to OpenCV format (BGR)
            self.normal_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 

            # Convert the imageFrame in BGR (RGB color space) to HSV color space
            hsv_image = cv2.cvtColor(self.normal_image, cv2.COLOR_BGR2HSV) 

            # Define the color ranges for goal and obstacle detection
            blue_lower = np.array([94, 80, 2], np.uint8) 
            blue_upper = np.array([120, 255, 255], np.uint8) 

            black_lower = np.array([0, 0, 0], np.uint8)
            black_upper = np.array([50, 50, 50], np.uint8)

            # Create masks for detecting specific colors
            blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
            black_mask = cv2.inRange(hsv_image, black_lower, black_upper)

            # Morphological transformation to reduce noise and improve contour detection
            kernal = np.ones((5, 5), "uint8") 

            blue_mask = cv2.dilate(blue_mask, kernal)
            black_mask = cv2.dilate(black_mask, kernal)

            # Find contours for blue and black masks
            contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_black, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Initialize detection flags
            goal_detected = False
            obstacle_detected = False

            # Process blue contours (goals)
            for contour in contours_blue:
                area = cv2.contourArea(contour)
                if area > 1:  # Ignore small contours to reduce noise
                    x, y, w, h = cv2.boundingRect(contour)
                    x_mid = int(x + (w / 2))
                    y_mid = int(y + (h / 2))

                    self.coord_goal = [x_mid, y_mid]
                    goal_detected = True  # Set flag to True as a goal is detected
                    cv2.drawContours(self.normal_image, [contour], 0, (255, 0, 0), 4)
                    cv2.putText(self.normal_image, "Goal", (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                    self.msg.data = self.coord_goal
                    self.goal_distance_publisher.publish(self.msg)

            if not goal_detected:
                self.coord_goal = [-1, -1]
                self.msg.data = self.coord_goal
                self.goal_distance_publisher.publish(self.msg)

            # Process black contours (obstacles)
            for contour in contours_black:
                area = cv2.contourArea(contour)
                if area > 1:  # Ignore small contours to reduce noise
                    x, y, w, h = cv2.boundingRect(contour)
                    x_mid = int(x + (w / 2))
                    y_mid = int(y + (h / 2))

                    self.coord_obstacle = [x_mid, y_mid]
                    obstacle_detected = True  # Set flag to True as an obstacle is detected
                    cv2.drawContours(self.normal_image, [contour], 0, (0, 255, 0), 4)
                    cv2.putText(self.normal_image, "Obstacle", (x_mid, y_mid), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    self.msg.data = self.coord_obstacle
                    self.obstacle_distance_publisher.publish(self.msg)

            if not obstacle_detected:
                self.coord_obstacle = [-1, -1]
                self.msg.data = self.coord_obstacle
                self.obstacle_distance_publisher.publish(self.msg)

            # Visualization of the processed image with detected objects
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
    rospy.init_node('detect_shape_node', anonymous=True)
    dip = NormalImageProcessor()
    rospy.spin()