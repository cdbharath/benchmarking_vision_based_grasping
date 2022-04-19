#!/usr/bin/env python3

import rospy
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.pick_and_place import PickAndPlace
from std_msgs.msg import Float64
from math import pi
import tf

if __name__ == "__main__":
    rospy.init_node("init_home_position")
    pick_and_place = PickAndPlace(gripper_offset=0.05, intermediate_z_stop=0.5)
    
    pick_and_place.setDropPose(x=0.0, y=0.4, z=0.4, roll=0, pitch=pi, yaw=0)
    pick_and_place.setGripperPose(width=0.1)
    
    pick_and_place.execute_place()
