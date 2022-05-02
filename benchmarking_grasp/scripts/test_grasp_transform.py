#!/usr/bin/env python

import rospy
from grasp_transform_module.grasp_transform import GraspTransform

if __name__ == "__main__":
    rospy.init_node('grasp_transform')
    grasp_Service = GraspTransform(sim_mode=False, crop=False)
    rospy.spin()
