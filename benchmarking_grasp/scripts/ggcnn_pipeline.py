#!/usr/bin/env python

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place.srv import GraspPrediction, GraspPredictionResponse
from pick_and_place.srv import ProcessAndExecute
from std_msgs.msg import Float64
from math import pi
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pick_and_place = PickAndPlace(0.05, 0.5)

def process_and_execute(req):
    global pick_and_place

    rospy.loginfo("waiting for service: ggcnn_service/predict")
    rospy.wait_for_service("ggcnn_service/predict")
    rospy.loginfo("Service call successful")

    srv_handle = rospy.ServiceProxy("ggcnn_service/predict", GraspPrediction)
    response = srv_handle()
    rospy.loginfo("response: " + str(response))
    x = response.best_grasp.pose.position.x
    y = response.best_grasp.pose.position.y
    z = response.best_grasp.pose.position.z 
    (rx, ry, rz) = euler_from_quaternion([response.best_grasp.pose.orientation.w, response.best_grasp.pose.orientation.x, response.best_grasp.pose.orientation.y, response.best_grasp.pose.orientation.z])

    pick_and_place.setPickPose(x, y, z + 0.02, rx, ry, rz)
    pick_and_place.setDropPose(0.0, 0.4, 0.5, 0, pi, 0)
    pick_and_place.setGripperPose(0.01, 0.01)
    
    # pick_and_place.execute_pick_and_place()
    pick_and_place.execute_cartesian_pick_up()

    return True


if __name__ == "__main__":
    rospy.Service("ggcnn_pipeline/process_and_execute", ProcessAndExecute, process_and_execute)
    rospy.spin()
    