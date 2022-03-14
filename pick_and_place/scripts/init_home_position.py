#!/usr/bin/env python

import rospy
from pick_and_place_module.eef_control import MoveGroupControl
from std_msgs.msg import Float64
from math import pi
import tf

if __name__ == "__main__":
    # rospy.init_node("grasping")
    moveit_control = MoveGroupControl()

    finger1_pub = rospy.Publisher('/panda_finger1_controller/command', Float64, queue_size=10)
    finger2_pub = rospy.Publisher('/panda_finger2_controller/command', Float64, queue_size=10)
    rospy.sleep(2)
        
    finger1_data = Float64()
    finger1_data.data = 0.05
    finger2_data = Float64()
    finger2_data.data = 0.05

    current_states = moveit_control.get_current_joint_states()
    print(current_states)

    planner = moveit_control.move_group.get_planner_id()
    print(planner)

    print(moveit_control.move_group.get_goal_joint_tolerance())
    moveit_control.move_group.set_goal_joint_tolerance(0.1)
    moveit_control.go_to_joint_state(j1=0.329, j2=-0.959, j3=0.906, j4=-2.33, j5=0.701, j6=1.602, j7=1.72)    
    # moveit_control.go_to_joint_state(j4=-0.35)    
