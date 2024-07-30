#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import GetModelState, GetLinkState
from geometry_msgs.msg import Pose
import numpy as np


class GazeboModelState:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/get_link_state')
        self.get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    def get_model_state(self, model_name: str, relative_entity_name: str = '') -> Pose:
        try:
            response = self.get_model_state_service(model_name, relative_entity_name)
            if response.success:
                return response.pose
            else:
                rospy.logerr(f"Failed to get state for model '{model_name}'")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def get_link_state(self, model_name: str, link_name: str) -> Pose:
        try:
            response = self.get_link_state_service(link_name, model_name)
            if response.success:
                return response.link_state.pose
            else:
                rospy.logerr(f"Failed to get state for link '{link_name}' in model '{model_name}'")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        position1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
        position2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
        distance = np.linalg.norm(position1 - position2)
        return distance

    def calculate_distance_to_link(self, model_name: str, link_name: str) -> float:
        model_pose = self.get_model_state(model_name)
        link_pose = self.get_link_state('rl_world', link_name)  
        if model_pose and link_pose:
            return self.calculate_distance(model_pose, link_pose)
        else:
            return float('inf')
        
    def calculate_distance_to_model(self, model_name1: str, model_name2: str) -> float:
        model1_pose = self.get_model_state(model_name1)
        model2_pose = self.get_model_state(model_name2) 
        if model1_pose and model2_pose:
            return self.calculate_distance(model1_pose, model2_pose)
        else:
            return float('inf')
        

if __name__ == "__main__":
    p = GazeboModelState()
    print(p.calculate_distance_to_link("robot", "Wall_5"))