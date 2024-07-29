#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from ultralytics import YOLO


class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.image_sub = rospy.Subscriber("/mobile_robot_description/camera1/image_raw", Image, self.callback)
        self.image_count = 0

        
    def callback(self, data):
        try:
    
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        path = "/home/erfan/catkin_ws/src/autonomous_car_controller/images/"
        image_name = path + f"image_{self.image_count}.jpg"
        cv2.imwrite(image_name, cv_image)
        print(f"Saved {image_name}")
        results = self.model([image_name])
        
        for res in results:
        	res.show() 
        
        # Display the image
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

        self.image_count += 1

def main():
    rospy.init_node('image_saver', anonymous=True)
    print("true")
    image_saver = ImageSaver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
