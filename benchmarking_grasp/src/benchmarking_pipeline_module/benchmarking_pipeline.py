import os
from turtle import width
import yaml 
from math import pi
import numpy as np
import enum
import csv
import cv2
from datetime import date, datetime
import six

import rospy
import rospkg
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from pick_and_place_module.pick_and_place import PickAndPlace
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper
from benchmarking_msgs.srv import GraspPrediction, GraspPredictionResponse
from benchmarking_msgs.srv import ProcessAndExecute
from gazebo_grasp_plugin_ros.msg import GazeboGraspEvent

class BenchmarkTestStates(enum.Enum):
    """
    States to keep track of the scores for evaluation in the simulation
    Not sure how these will help duing realtime tests
    """
    FREE = 0
    PICK_UP = 1
    ROTATE = 2
    SHAKE = 3

class BenchmarkTest:
    """
    Spawns object based on the objects listed in the yaml file
    Calls the grasp_tramsform/predict service which inturn calls service_server/predict to get the detected grasps
    Runs the benchmarking procedure for the spawned objects in Gazebo
    Keeps track of the scores of the detected grasp for eacch object (makes sense for only in simulation)  
    """
    def __init__(self, use_cartesian=True, over_head=True, sim_mode=True):
        """
        1. Initializes pick and place module
        2. Initializrs move_group module
        3. Inputs:
            use_cartesian - cartesian planning or joint space planning
            over_head - camera over head or eye in hand
            sim_mode - simulation or real robot

        TODO add parameters into a yaml file

        4. Set urdf, yaml package path for spawning objects in gazebo. Enter the requried objects to be tested in the yaml file
        5. Create log file to log the scores for each object
        6. Parse yaml file for object list and benchmarking parameters
        7. Initialize required ROS topics
        """
        self.pick_and_place = PickAndPlace(gripper_offset=0.1, intermediate_z_stop=0.5)
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()
        self.use_cartesian = use_cartesian
        self.over_head = over_head
        self.sim_mode = sim_mode

        # Reach scan pose if eye in hand
        if not self.over_head:
            self.pick_and_place.setScanPose(x=0.3, y=0.0, z=0.560, roll=0.0, pitch=3.14, yaw=0.0)
            if self.use_cartesian:
                self.pick_and_place.reach_cartesian_scanpose()
            else:
                self.pick_and_place.reach_scanpose()

        # Package names to look for urdf and log files
        self.urdf_package_name = "pick_and_place"
        self.yaml_package_name = "benchmarking_grasp"

        self.start_time = str(datetime.now())

        self.rospack = rospkg.RosPack()
        self.urdf_package_path = os.path.join(self.rospack.get_path(self.urdf_package_name), "urdf/objects")
        self.yaml_package_path = os.path.join(self.rospack.get_path(self.yaml_package_name), "config/benchmarking.yaml")
        self.log_folder = os.path.join(self.rospack.get_path(self.yaml_package_name), "logs") 
        self.log_file_path = os.path.join(self.log_folder, "log_" +  self.start_time + ".csv") 

        # Create log folder if not exists
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)

        # Write the header to the log file
        with open(self.log_file_path, 'w') as file:
            header = ['Experiment', 'Trial', 'Object', 'Pose', 'Score']
            writer = csv.writer(file)
            writer.writerow(header)

        # Parse object list from yaml and benchmarking parameters
        parsed_experiments = self.parse_yaml(self.yaml_package_path)

        self.experiments = []

        # Generate object list and benchmarking parameters for spawning objects
        for experiment_idx in range(len(parsed_experiments)):
            model_paths = parsed_experiments[experiment_idx][0] 
            center = parsed_experiments[experiment_idx][1]
            r = parsed_experiments[experiment_idx][2]
            alpha = parsed_experiments[experiment_idx][3]
            n = parsed_experiments[experiment_idx][4]

            center_coord = np.array([center, 0.00, 0.0, 0, 0, 0])

            # Generate object pose for spawning
            poses = [center_coord, 
                     center_coord + np.array([r, 0, 0, 0, 0, 0]),
                     center_coord + np.array([0, r, 0, 0, 0, 0]),
                     center_coord + np.array([0, -r, 0, 0, 0, 0]),
                     center_coord + np.array([0, r, 0, 0, 0, alpha]),
                     center_coord + np.array([0, -r, 0, 0, 0, -alpha])]
            
            # Mirroring
            # poses.extend([center_coord, 
            #               center_coord + [r, 0, pi, 0, 0, 0],
            #               center_coord + [0, r, 0, pi, 0, 0],
            #               center_coord + [0, -r, 0, pi, 0, 0],
            #               center_coord + [0, r, 0, alpha + pi, 0, 0],
            #               center_coord + [0, -r, 0, -alpha + pi, 0, 0]]
            # )

            self.experiments.append([model_paths, poses, n])
    
        # Variables to track inside Timer
        self.experiment_idx = 0
        self.pose_idx = 0
        self.object_idx = 0
        self.n_ = 0
        self.testing_in_process = False
        self.benchmark_state = BenchmarkTestStates.FREE
        self.attached = False
        self.positive_grasps = []
        self.negative_grasps = []
        self.finger1_state = 0.05
        self.finger2_state = 0.05

        # Initialize grasp plugin and joint states topic
        if self.sim_mode:
            rospy.Subscriber("/gazebo_grasp_plugin_event_republisher/grasp_events", GazeboGraspEvent, self.on_grasp_event)
        rospy.Subscriber("/joint_states", JointState, self.joint_cb)
        rospy.Timer(rospy.Duration(nsecs=1000000), self.execute_benchmark_test)
    
    def execute_benchmark_test(self, event):
        """
        Executes the entire benchmarking procedure
        1. Spawns an object in Gazebo / waits for user to place the object as requested
        2. Calls predict service to get the 3D coordinates of the grasp
        3. Picks up the object
        4. Performs the benchmarking tests
        5. Places the object (tracks the state all the way)
        6. Updates the score log file (Only useful for simulator)
        7. Deletes the object from environment (Only useful for simulator)
        """

        skip = False
        experiment = self.experiments[self.experiment_idx]
        object = experiment[0][self.object_idx]
	# object = experiment[0][1]
        pose = experiment[1][self.pose_idx]
        
        if self.sim_mode:
            self.spawn_model(object, pose)
        else:
            try:
                six.moves.input("[Benchmarking Pipeline] Place the {} at ({}, {}, {}) meters with respect to the robot base and press ENTER".format(str(object.split("/")[-1].split(".")[0]), pose[0], pose[1], pose[2]))
                # print("Place the {} at ({}, {}, {}) meters with respect to the robot base and press ENTER".format(str(object.split("/")[-1].split(".")[0]), pose[0], pose[1], pose[2]))
                # while True:
                #     k = cv2.waitKey(33)
                #     if k == 32:
                #         break
            except SyntaxError:
                pass

        rospy.sleep(1)

        # Execute the benchmark test
        self.testing_in_process = True
        
        try:
            self.process_rgbd_and_execute_pickup()
            # self.test_benchmark()
            score = self.benchmark_state
            self.place()
        except Exception as e:
            rospy.logerr("[Benchmarking Pipeline] skipping this turn %s", e)
            skip = True

        self.testing_in_process = False

        if not self.over_head:
            if self.use_cartesian:
                self.pick_and_place.reach_cartesian_scanpose()
            else:
                self.pick_and_place.reach_scanpose()

        if not skip:
            with open(self.log_file_path, 'a') as file:
                update = [str(self.experiment_idx), str(self.n_), str(object.split("/")[-1].split(".")[0]), str(self.pose_idx), score.value]
                writer = csv.writer(file)
                writer.writerow(update)

            # Track the status of the test
            self.pose_idx = self.pose_idx + 1 
            if self.pose_idx >= len(experiment[1]):
                self.pose_idx = 0
                self.n_ = self.n_ + 1
                if self.n_ >= experiment[2]:
                    self.n_ = 0
                    self.object_idx = self.object_idx + 1
                    if self.object_idx >= len(experiment[0]):
                        self.object_idx = 0
                        self.experiment_idx = self.experiment_idx + 1
                        rospy.loginfo("[Benchmarking Pipeline] Success rate for experiment %s: %s", self.experiment_idx, len(self.positive_grasps)/(len(self.positive_grasps) + len(self.negative_grasps)))
                        if self.experiment_idx >= len(self.experiments): 
                            rospy.loginfo("[Benchmarking Pipeline] Benchmarking test completed successfully")
                            rospy.signal_shutdown("[Benchmarking Pipeline] Benchmarking test completed successfully")
        
        if self.sim_mode:
            try:
                rospy.sleep(0.5)
                self.delete_model(object)
                rospy.sleep(0.5)
            except Exception as e:
                rospy.logerr("[Benchmarking Pipeline] Object deleted while still attached to hand %s", e)

    def on_grasp_event(self, data):
        """
        If a grasp is detected by the grasp plugin
        Do stuff
        """
        object = data.object
        attached = data.attached

        if attached:
            self.gripper.grasp(self.finger1_state, self.finger2_state)
                        
        if attached and self.testing_in_process:
            self.attached = True
        
        if not attached and self.testing_in_process:
            self.negative_grasps.append(object)
            self.attached = False
        
        if not attached and not self.testing_in_process and self.attached:
            self.positive_grasps.append(object)
            self.attached = False
            
    def joint_cb(self, data):
        """
        Callback for joint states
        """
        position = data.position
        self.finger1_state = position[0]
        self.finger2_state = position[1]
    
    def parse_yaml(self, yaml_package_path):
        """
        Parse information from the yaml file
        """
        model_file = open(yaml_package_path, 'r')
        config = yaml.load(model_file, Loader=yaml.FullLoader)

        if config['only_first']:
            experiments_n = 1
        else:
            experiments_n = len(config) - 1

        experiments = []
        for experiment_idx in range(1, experiments_n + 1):
            models = config["experiment_" + str(experiment_idx)]["objects"]
            model_paths = [os.path.join(self.urdf_package_path, model, model[4:] + ".sdf") for model in models]

            center = config["experiment_" + str(experiment_idx)]["config"]["center"]            
            r = config["experiment_" + str(experiment_idx)]["config"]["r"]
            alpha = config["experiment_" + str(experiment_idx)]["config"]["alpha"]
            n = config["experiment_" + str(experiment_idx)]["config"]["n"]
            experiments.append([model_paths, center, r, alpha, n])
        return experiments
    
    def spawn_model(self, model_path, pose):
        """
        Spawns model in the required pose in Gazebo
        """
        spawn_pose = Pose()
        spawn_pose.position.x = pose[0]
        spawn_pose.position.y = pose[1]
        spawn_pose.position.z = pose[2]

        quaternion = quaternion_from_euler (pose[3], pose[4], pose[5])
        spawn_pose.orientation.x = quaternion[0]
        spawn_pose.orientation.y = quaternion[1]
        spawn_pose.orientation.z = quaternion[2]
        spawn_pose.orientation.w = quaternion[3]

        # rospy.wait_for_service('gazebo/spawn_urdf_model')
        # spawn_model_handle = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_handle = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        file_xml = open(model_path, 'r')
        model_xml = file_xml.read()
        
        res = spawn_model_handle(model_path.split("/")[-1].split(".")[0], model_xml, '', spawn_pose, 'world')
        return res

    def delete_model(self, model_path):
        """
        Deletes the model from the Gazebo environment
        """

        rospy.wait_for_service('gazebo/delete_model')
        delete_model_handle = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
        res = delete_model_handle(model_path.split("/")[-1].split(".")[0])
        return res

    def process_rgbd_and_execute_pickup(self):
        """
        1. Calls the predict service and gets a 3D grasp coordinate as response
        2. Picks up the object from the received coordinate
        3. Benchmarking state is updated to PICK_UP
        """
        rospy.loginfo("[Benchmarking Pipeline] waiting for service: grasp_transform/predict")
        rospy.wait_for_service("grasp_transform/predict")
        rospy.loginfo("[Benchmarking Pipeline] Service call successful")

        srv_handle = rospy.ServiceProxy("grasp_transform/predict", GraspPrediction)
        response = srv_handle()

        x = response.best_grasp.pose.position.x
        y = response.best_grasp.pose.position.y
        z = response.best_grasp.pose.position.z
        (rx, ry, rz) = euler_from_quaternion([response.best_grasp.pose.orientation.w, response.best_grasp.pose.orientation.x, response.best_grasp.pose.orientation.y, response.best_grasp.pose.orientation.z])

        self.pick_and_place.setPickPose(x=x, y=y, z=z, roll=rx, pitch=ry, yaw=rz)
        self.pick_and_place.setDropPose(x=0.4, y=0.0, z=1.5, roll=0, pitch=pi, yaw=0)
        self.pick_and_place.setGripperPose(width=0.00)

        if self.use_cartesian:
            self.pick_and_place.execute_cartesian_pick_up()
        else:
            self.pick_and_place.execute_pick_up()
        if self.attached:
            self.benchmark_state = BenchmarkTestStates.PICK_UP

        return True
    
    def place(self):
        """
        1. Places the object 
        2. Benchmarking state is updated to FREE
        """
        self.benchmark_state = BenchmarkTestStates.FREE
        
        if self.use_cartesian:
            self.pick_and_place.execute_cartesian_place()
        else:
            self.pick_and_place.execute_place()

    def test_benchmark(self):
        """
        1. Performs benchmarking tests
        2. Updates state as applicable
        3. Two tests
            1. Roll pitch yaw rotation
            2. Shake test
        """
        # self.moveit_control = MoveGroupControl()

        # Rotate the object
        pose = self.moveit_control.get_current_pose()
        (x, y, z) = (pose.position.x, pose.position.y, pose.position.z) 
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
        
        # Yawing    
        # self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
        # self.moveit_control.follow_cartesian_path([[x, y, z, roll + pi/4, pitch, yaw]])
        # self.moveit_control.follow_cartesian_path([[x, y, z, roll - pi/4, pitch, yaw]])
        # self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

        # Pitching
        # self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
        # self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch + pi/4, yaw]])
        # self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch - pi/4, yaw]])
        # self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

        # Rolling
        self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
        self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw + pi/4]])
        self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw - pi/4]])
        self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

        if self.attached:
            self.benchmark_state = BenchmarkTestStates.ROTATE

        # Shaking
        self.moveit_control.follow_cartesian_path([[x, y, z + 0.1, roll, pitch, yaw]])
        self.moveit_control.follow_cartesian_path([[x, y, z - 0.1, roll, pitch, yaw]])
        # self.moveit_control.follow_cartesian_path([[x, y, z + 0.1, roll, pitch, yaw]])
        # self.moveit_control.follow_cartesian_path([[x, y, z - 0.1, roll, pitch, yaw]])
        self.moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

        if self.attached:
            self.benchmark_state = BenchmarkTestStates.SHAKE
