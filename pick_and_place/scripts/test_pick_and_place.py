#!/usr/bin/env python

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from std_msgs.msg import Float64
from math import pi

def pick_and_place():
    pick_and_place = PickAndPlace(gripper_offset=0.05, intermediate_z_stop=0.5)
    
    pick_and_place.setDropPose(x=0.4, y=0.0, z=0.5, roll=0, pitch=pi, yaw=0)
    pick_and_place.setPickPose(x=0.40, y=0.00, z=0.1, roll=0, pitch=pi, yaw=0)
    pick_and_place.setGripperPose(width=0.0)
    
    pick_and_place.execute_pick_and_place()
    # pick_and_place.execute_pick_up()
    # pick_and_place.execute_place()
    # pick_and_place.execute_cartesian_pick_and_place()
    # pick_and_place.execute_cartesian_pick_up()
    # pick_and_place.execute_cartesian_place()

if __name__ == "__main__":
    rospy.init_node("test_pnp")
    pick_and_place()