#!/usr/bin/env python

import rospy
from moveit_adapter_module.grasping import Gripper
from moveit_adapter_module.eef_control import MoveGroupControl
from benchmarking_msgs.srv import EndEffectorWaypoint, GripperCommand, CurrentPose

class MoveitAdapter:
    def __init__(self) -> None:
        self.gripper = Gripper()
        self.moveit_control = MoveGroupControl()
        
        rospy.Service('moveit_adapter/grasp', GripperCommand, self.grasp_service)
        rospy.Service('moveit_adapter/cartesian_path', EndEffectorWaypoint, self.cartesian_path_service)
        rospy.Service('moveit_adapter/vanilla', EndEffectorWaypoint, self.vanilla_path_service)
        rospy.Service('moveit_adapter/get_current_pose', CurrentPose, self.get_current_pose_service)
        
    def cartesian_path_service(self, req):
        self.moveit_control.follow_cartesian_path([req.x, req.y, req.z, req.roll, req.pitch, req.yaw])
        return True
        
    def vanilla_path_service(self, req):
        self.moveit_control.go_to_pose_goal(req.x, req.y, req.z, req.roll, req.pitch, req.yaw)
        return True
        
    def grasp_service(self, req):
        self.gripper.grasp(req.width)
        return True
    
    def get_current_pose_service(self, req):
        return self.moveit_control.get_current_pose()

if __name__ == "__main__":
    rospy.init_node('moveit_adapter_node')
    moveit_adapter = MoveitAdapter()
    rospy.spin()