#!/usr/bin/env python

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace

if __name__ == "__main__":
    rospy.init_node('planner_node')
    pick_and_place = PickAndPlace()
    rospy.spin()