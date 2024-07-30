#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist


class Spawner:
    def __init__(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        

    def spawn_robot(self, model_name, position, orientation=[0, 0, 0, 1]):
        # Create a new model state message
        model_state = ModelState()
        model_state.model_name = model_name

        # Set the position and orientation
        model_state.pose = Pose()
        model_state.pose.position.x = position[0]
        model_state.pose.position.y = position[1]
        model_state.pose.position.z = position[2]
        model_state.pose.orientation.x = orientation[0]
        model_state.pose.orientation.y = orientation[1]
        model_state.pose.orientation.z = orientation[2]
        model_state.pose.orientation.w = orientation[3]

        # Call the service to set the model state
        try:
            self.set_model_state_service(model_state)
            rospy.loginfo(f"Spawned {model_name} at position {position} with orientation {orientation}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn robot: {e}")

