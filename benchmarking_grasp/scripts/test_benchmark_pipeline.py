#!/usr/bin/env python

import os
import yaml 
from math import pi
import numpy as np

import rospy
import rospkg
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

from pick_and_place_module.pick_and_place import PickAndPlace
from pick_and_place_module.eef_control import MoveGroupControl
from benchmarking_msgs.srv import GraspPrediction, GraspPredictionResponse
from benchmarking_msgs.srv import ProcessAndExecute
from gazebo_grasp_plugin_ros.msg import GazeboGraspEvent

class BenchmarkTest:
    def __init__(self):
        self.urdf_package_name = "pick_and_place"
        self.yaml_package_name = "benchmarking_grasp"
        self.center_coord = np.array([0.5, 0.00, 0.0, 0, 0, 0])
        
        self.rospack = rospkg.RosPack()
        self.urdf_package_path = os.path.join(self.rospack.get_path(self.urdf_package_name), "urdf/objects")
        self.yaml_package_path = os.path.join(self.rospack.get_path(self.yaml_package_name), "config/benchmarking.yaml")
        
        self.models_paths, self.r, self.alpha, self.n = self.parse_yaml(self.yaml_package_path)

        # Generate object pose for spawning
        self.poses = [self.center_coord, 
                      self.center_coord + np.array([self.r, 0, 0, 0, 0, 0]),
                      self.center_coord + np.array([0, self.r, 0, 0, 0, 0]),
                      self.center_coord + np.array([0, -self.r, 0, 0, 0, 0]),
                      self.center_coord + np.array([0, self.r, 0, 0, 0, self.alpha]),
                      self.center_coord + np.array([0, -self.r, 0, 0, 0, -self.alpha])]
        
        # Mirroring
        # self.poses.extend([self.center_coord, 
        #               self.center_coord + [self.r, 0, pi, 0, 0, 0],
        #               self.center_coord + [0, self.r, 0, pi, 0, 0],
        #               self.center_coord + [0, -self.r, 0, pi, 0, 0],
        #               self.center_coord + [0, self.r, 0, self.alpha + pi, 0, 0],
        #               self.center_coord + [0, -self.r, 0, -self.alpha + pi, 0, 0]]
        # )
    
        # Variables to track inside Timer
        self.pose_idx = 0
        self.object_idx = 0
        self.n_ = 0
        self.testing_in_process = False
        self.attached = False
        self.positive_grasps = []
        self.negative_grasps = []

        rospy.Subscriber("/gazebo_grasp_plugin_event_republisher/grasp_events", GazeboGraspEvent, self.on_grasp_event)
        rospy.Timer(rospy.Duration(nsecs=1000000), self.execute_benchmark_test)
    
    def execute_benchmark_test(self, event):
        object = self.models_paths[self.object_idx]
        pose = self.poses[self.pose_idx]
        
        self.spawn_model(object, pose)
        self.testing_in_process = True
        self.process_rgbd_and_execute_pickup()
        self.test_benchmark()
        self.place()
        self.testing_in_process = False

        self.pose_idx = self.pose_idx + 1 
        if self.pose_idx >= len(self.poses):
            self.pose_idx = 0
            self.n_ = self.n_ + 1
            if self.n_ >= self.n:
                self.n_ = 0
                self.object_idx = self.object_idx + 1
                if self.object_idx >= len(self.models_paths):
                    rospy.loginfo(len(self.positive_grasps)/(len(self.positive_grasps) + len(self.negative_grasps)))
                    rospy.loginfo("Benchmarking test completed successfully")
                    rospy.signal_shutdown("Benchmarking test completed successfully")

        rospy.sleep(0.5)
        self.delete_model(object)
        rospy.sleep(0.5)
    
    def on_grasp_event(self, data):
        object = data.object
        attached = data.attached
        
        rospy.logerr(self.positive_grasps)
        rospy.logerr(self.negative_grasps)
                
        if attached and self.testing_in_process:
            self.attached = True
        
        if not attached and self.testing_in_process:
            self.negative_grasps.append(object)
            self.attached = False
        
        if not attached and not self.testing_in_process and self.attached:
            self.positive_grasps.append(object)
            self.attached = False
            
    
    def parse_yaml(self, yaml_package_path):
        
        model_file = open(yaml_package_path, 'r')
        config = yaml.load(model_file, Loader=yaml.FullLoader)
        models = config["objects"]
        model_paths = [os.path.join(self.urdf_package_path, model, model + ".urdf") for model in models]
        
        r = config["config"]["r"]
        alpha = config["config"]["alpha"]
        n = config["config"]["n"]
        return model_paths, r, alpha, n
    
    def spawn_model(self, model_path, pose):
        spawn_pose = Pose()
        spawn_pose.position.x = pose[0]
        spawn_pose.position.y = pose[1]
        spawn_pose.position.z = pose[2]

        quaternion = quaternion_from_euler (pose[3], pose[4], pose[5])
        spawn_pose.orientation.x = quaternion[0]
        spawn_pose.orientation.y = quaternion[1]
        spawn_pose.orientation.z = quaternion[2]
        spawn_pose.orientation.w = quaternion[3]

        rospy.wait_for_service('gazebo/spawn_urdf_model')
        spawn_model_handle = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        
        file_xml = open(model_path, 'r')
        model_xml = file_xml.read()
        
        res = spawn_model_handle(model_path.split("/")[-1].split(".")[0], model_xml, '', spawn_pose, 'world')
        return res

    def delete_model(self, model_path):
        
        rospy.wait_for_service('gazebo/delete_model')
        delete_model_handle = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
        res = delete_model_handle(model_path.split("/")[-1].split(".")[0])
        return res

    def process_rgbd_and_execute_pickup(self):

        rospy.loginfo("waiting for service: ggcnn_service/predict")
        rospy.wait_for_service("ggcnn_service/predict")
        rospy.loginfo("Service call successful")

        srv_handle = rospy.ServiceProxy("ggcnn_service/predict", GraspPrediction)
        response = srv_handle()

        # rospy.loginfo("response: " + str(response))
        x = response.best_grasp.pose.position.x
        y = response.best_grasp.pose.position.y
        z = response.best_grasp.pose.position.z 
        (rx, ry, rz) = euler_from_quaternion([response.best_grasp.pose.orientation.w, response.best_grasp.pose.orientation.x, response.best_grasp.pose.orientation.y, response.best_grasp.pose.orientation.z])

        self.pick_and_place = PickAndPlace(0.07, 0.5)

        self.pick_and_place.setPickPose(x, y, z, rx, ry, rz)
        self.pick_and_place.setDropPose(0.0, 0.4, 0.5, 0, pi, 0)
        self.pick_and_place.setGripperPose(0.01, 0.01)
        
        self.pick_and_place.execute_cartesian_pick_up()

        return True
    
    def place(self):
        self.pick_and_place.execute_cartesian_place()


    def test_benchmark(self):
        moveit_control = MoveGroupControl()

        # Rotate the object
        pose = moveit_control.get_current_pose()
        (x, y, z) = (pose.position.x, pose.position.y, pose.position.z) 
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
        
        # Yawing    
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z, roll + pi/4, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z, roll - pi/4, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

        # Pitching
        roll = roll - pi/4
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch + pi/4, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch - pi/4, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

        # Rolling
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw + pi/4]])
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw - pi/4]])
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
        roll = roll + pi/4
        
        # Shaking
        moveit_control.follow_cartesian_path([[x, y, z + 0.1, roll, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z - 0.1, roll, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z + 0.1, roll, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z - 0.1, roll, pitch, yaw]])
        moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

if __name__ == "__main__":
    rospy.init_node("benchmark_test", log_level=rospy.INFO)
    
    benchmark_test = BenchmarkTest()
    rospy.spin()
    