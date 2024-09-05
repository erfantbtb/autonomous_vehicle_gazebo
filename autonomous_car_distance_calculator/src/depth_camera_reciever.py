#!/usr/bin/python3

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np


class DepthImageProcessor:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.depth_image = None
        self.goal_coordinate = None 
        self.obstacle_coordinate = None 
        self.distance_from_goal = None 
        self.distance_from_obstacle = None 
        self.angle_to_goal = None
        self.angle_to_obstacle = None
        self.goal_localization_data = None 
        self.obstacle_localization_data = None

        self.f_x = 692.818364
        self.c_x = 400  

        depth_image_topic = "/depth_camera/depth/image_raw"
        rospy.Subscriber(depth_image_topic, Image, self.depth_image_callback)
        rospy.loginfo(f"Subscribed to {depth_image_topic}")

        goal_coordinate_topic = "/coordinate_of_goal"
        rospy.Subscriber(goal_coordinate_topic, Float32MultiArray, self.get_distance_from_goal)
        rospy.loginfo(f"Subscribed to {goal_coordinate_topic}")

        obstacle_coordinate_topic = "/coordinate_of_obstacle"
        rospy.Subscriber(obstacle_coordinate_topic, Float32MultiArray, self.get_distance_from_obstacle)
        rospy.loginfo(f"Subscribed to {obstacle_coordinate_topic}")

        self.obstacle_localization_publisher = rospy.Publisher("obstacle_localization", Float32MultiArray, queue_size=10)
        self.goal_localization_publisher = rospy.Publisher("goal_localization", Float32MultiArray, queue_size=10)
        self.msg = Float32MultiArray()
        
    def depth_image_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
            self.visualize_depth_image()

        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def visualize_depth_image(self):
        if self.depth_image is not None:
            depth_image_normalized = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_uint8 = depth_image_normalized.astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_image_uint8, cv2.COLORMAP_VIRIDIS)

            cv2.imshow("Depth Camera", depth_colormap)
            cv2.waitKey(1)

    def get_distance_from_goal(self, data):
        if self.depth_image is not None:
            coords = data.data
            if coords:
                x, y = int(coords[0]), int(coords[1])
                if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                    self.distance_from_goal = self.depth_image[y, x]
                    self.angle_to_goal = self.calculate_angle(x)
                    self.goal_localization_data = [self.distance_from_goal, self.angle_to_goal]
                else:
                    # rospy.logerr("Goal coordinates are out of bounds")
                    self.goal_localization_data = [-1, -1]  # No goal detected
            else:
                rospy.logwarn("No goal coordinates received")
                self.goal_localization_data = [-1, -1]  # No goal detected

            self.msg.data = self.goal_localization_data
            self.goal_localization_publisher.publish(self.msg)
        else:
            rospy.logerr("Depth image is not available")

    def get_distance_from_obstacle(self, data):
        if self.depth_image is not None:
            coords = data.data
            if coords:
                x, y = int(coords[0]), int(coords[1])
                if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                    self.distance_from_obstacle = self.depth_image[y, x]
                    self.angle_to_obstacle = self.calculate_angle(x)
                    self.obstacle_localization_data = [self.distance_from_obstacle, self.angle_to_obstacle]
                else:
                    # rospy.logerr("Obstacle coordinates are out of bounds")
                    self.obstacle_localization_data = [-1, -1]  # No obstacle detected
            else:
                rospy.logwarn("No obstacle coordinates received")
                self.obstacle_localization_data = [-1, -1]  # No obstacle detected

            self.msg.data = self.obstacle_localization_data
            self.obstacle_localization_publisher.publish(self.msg)
        else:
            rospy.logerr("Depth image is not available")
    
    def calculate_angle(self, x):
        if self.depth_image is not None:
            delta_x = x - self.c_x
            
            theta_x_radians = np.arctan2(delta_x, self.f_x)
            theta_x_degrees = np.degrees(theta_x_radians)
            
            return theta_x_degrees
        
        else:
            rospy.logerr("Depth image is not available")
            return None


if __name__ == "__main__":
    rospy.init_node('depth_image_processor', anonymous=True)
    dip = DepthImageProcessor()
    rospy.spin()


    