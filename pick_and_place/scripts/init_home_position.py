#!/usr/bin/env python3

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from math import pi

if __name__ == "__main__":
    rospy.init_node("init_home_position")
    pick_and_place = PickAndPlace(gripper_offset=0.8, intermediate_z_stop=0.7)

    pick_and_place.setDropPose(x=0.45, y=0.0, z=0.7, roll=0, pitch=pi, yaw=pi)
    pick_and_place.setGripperPose(width=0.05)
        
    pick_and_place.execute_place()
