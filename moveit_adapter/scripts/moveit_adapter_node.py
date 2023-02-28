#!/usr/bin/env python

import rospy
from moveit_adapter_module.grasping import Gripper
from moveit_adapter_module.eef_control import MoveGroupControl
from moveit_adapter.srv import EndEffectorWaypoint, GripperCommand, CurrentPose, SetJointVelocity

class MoveitAdapter:
    def __init__(self):
        self.gripper = Gripper()
        self.moveit_control = MoveGroupControl(gripper_as_eef=True)
        
        rospy.Service('moveit_adapter/grasp', GripperCommand, self.grasp_service)
        rospy.Service('moveit_adapter/cartesian_path', EndEffectorWaypoint, self.cartesian_path_service)
        rospy.Service('moveit_adapter/vanilla', EndEffectorWaypoint, self.vanilla_path_service)
        rospy.Service('moveit_adapter/get_current_pose', CurrentPose, self.get_current_pose_service)
        rospy.Service('moveit_adapter/set_joint_velocity', SetJointVelocity, self.set_joint_velocity)
        
    def cartesian_path_service(self, req):
        self.moveit_control.follow_cartesian_path([[req.x, req.y, req.z, req.roll, req.pitch, req.yaw]])
        return True
        
    def vanilla_path_service(self, req):
        self.moveit_control.go_to_pose_goal(req.x, req.y, req.z, req.roll, req.pitch, req.yaw)
        return True
        
    def grasp_service(self, req):
        self.gripper.grasp(req.width)
        return True
    
    def get_current_pose_service(self, req):
        return self.moveit_control.get_current_pose()

    def set_joint_velocity(self, req):
        self.moveit_control.set_joint_velocity(req.joint_velocity)
        return True

if __name__ == "__main__":
    rospy.init_node('moveit_adapter_node')
    moveit_adapter = MoveitAdapter()
    rospy.spin()