#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ContactsState

class ContactDetector:
    def __init__(self):
        rospy.init_node('contact_detector', anonymous=True)
        self.contact_sub = rospy.Subscriber('/gazebo/contact', ContactsState, self.contact_callback)
        self.contacts = []

    def contact_callback(self, data):
        self.contacts = data.states
        if self.contacts:
            rospy.loginfo("Contact detected!")
            for contact in self.contacts:
                rospy.loginfo(f"Collision between: {contact.collision1_name} and {contact.collision2_name}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ContactDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
