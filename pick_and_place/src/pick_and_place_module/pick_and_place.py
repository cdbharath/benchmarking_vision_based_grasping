from copy import deepcopy
from math import pi

import rospy
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper
from benchmarking_msgs.srv import EndEffectorWaypoint, EndEffectorWaypointRequest, GripperCommand, GripperCommandRequest

class PickAndPlace:
    def __init__(self, gripper_offset=0.0, intermediate_z_stop=0.5, gripper_as_eef=True):
        self.gripper_offset = gripper_offset
        self.intermediate_z_stop = intermediate_z_stop
        self.scan_pose = [0.0, 0.3, 0.6, 0.0, pi, 0.0] 
        self.home_pose = [0.0, 0.3, 0.6, 0.0, pi, 0.0]
        self.pick_pose = None
        self.place_pose = None
        self.gripper_pose = None
        self.moveit_control = MoveGroupControl(gripper_as_eef=gripper_as_eef)
        self.gripper = Gripper()
        self.angle_offset = 0.0
            
    def setPickPose(self, x, y, z, roll, pitch, yaw):
        self.pick_pose = [x, y, z, roll + self.angle_offset, pitch, yaw]
    
    def setDropPose(self, x, y, z, roll, pitch, yaw):
        self.drop_pose = [x, y, z, roll + self.angle_offset, pitch, yaw]
    
    def setHomePose(self, x=0.0, y=0.3, z=0.6, roll=0.0, pitch=pi, yaw=0.0):
        self.home_pose = [x, y, z, roll + self.angle_offset, pitch, yaw]

    def setScanPose(self, x=0.0, y=0.3, z=0.6, roll=0.0, pitch=pi, yaw=0.0):
        self.scan_pose = [x, y, z, roll + self.angle_offset, pitch, yaw]

    def setGripperPose(self, width=0.0):
        self.gripper_pose = width
        
    def call_cartesian_service(self, waypoint):
        rospy.wait_for_service('moveit_adapter/cartesian_path')
        try:
            cartesian_service = rospy.ServiceProxy('moveit_adapter/cartesian_path', EndEffectorWaypoint)
            
            waypoint_request = EndEffectorWaypointRequest()
            waypoint_request.x = waypoint[0]
            waypoint_request.y = waypoint[1]
            waypoint_request.z = waypoint[2]
            waypoint_request.roll = waypoint[3]
            waypoint_request.pitch = waypoint[4]
            waypoint_request.yaw = waypoint[5]
            
            cartesian_service(waypoint)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)

    def call_vanilla_service(self, waypoint):
        rospy.wait_for_service('moveit_adapter/vanilla')
        try:
            vanilla_service = rospy.ServiceProxy('moveit_adapter/vanilla', EndEffectorWaypoint)
            
            waypoint_request = EndEffectorWaypointRequest()
            waypoint_request.x = waypoint[0]
            waypoint_request.y = waypoint[1]
            waypoint_request.z = waypoint[2]
            waypoint_request.roll = waypoint[3]
            waypoint_request.pitch = waypoint[4]
            waypoint_request.yaw = waypoint[5]
            
            vanilla_service(waypoint)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)

    def call_gripper_service(self, width):
        rospy.wait_for_service('moveit_adapter/grasp')
        try:
            gripper_service = rospy.ServiceProxy('moveit_adapter/grasp', GripperCommand)
            
            gripper_request = GripperCommandRequest()
            gripper_request.width = width
            
            gripper_service(gripper_request)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)

    def generate_waypoints(self, destination_pose, action):
        '''
        Generated waypoints are for a particular application
        This is to be changed based on the application it is being used
        0. Generate for scan 
        1. Generate for pick
        2. Generate for intermediate stop
        3. Generate for drop
        '''
        move_group = self.moveit_control

        waypoints = []

        if action == 0:
            # Scanning pose waypoint   
            current_pose = move_group.get_current_pose()
            current_pose_ = deepcopy(destination_pose)
            current_pose_[0] = self.scan_pose[0]
            current_pose_[1] = self.scan_pose[1]
            current_pose_[2] = self.scan_pose[2]
            current_pose_[3] = self.scan_pose[3]
            current_pose_[4] = self.scan_pose[4]
            current_pose_[5] = self.scan_pose[5]
            waypoints.append(current_pose_)

        if action == 1:
            # Intermediate vertical stop waypoint
            current_pose = move_group.get_current_pose()
            current_pose_ = deepcopy(destination_pose)
            current_pose_[2] = self.intermediate_z_stop
            waypoints.append(current_pose_)

            # 10 cm above pick pose waypoint 
            destination_pose_ = deepcopy(destination_pose)
            destination_pose_[2] = destination_pose_[2]  + 0.1 
            waypoints.append(destination_pose_)

            # Pick pose waypoint with gripper height offset
            destination_pose_ = deepcopy(destination_pose)
            destination_pose_[2] = destination_pose_[2]  + self.gripper_offset 
            waypoints.append(destination_pose_)

        if action == 2:
            # Intermediate vertical stop waypoint
            intermediate_pose = deepcopy(destination_pose)
            intermediate_pose[2] = self.intermediate_z_stop
            waypoints.append(intermediate_pose)

        if action == 3:
            # Intermediate vertical stop waypoint
            current_pose = move_group.get_current_pose()
            current_pose_ = deepcopy(destination_pose)
            current_pose_[0] = current_pose.position.x
            current_pose_[1] = current_pose.position.y
            current_pose_[2] = self.intermediate_z_stop
            waypoints.append(current_pose_)

            # Drop pose waypoint
            intermediate_pose = deepcopy(destination_pose)
            intermediate_pose[2] = self.intermediate_z_stop
            waypoints.append(intermediate_pose)

        return waypoints
    
    def execute_cartesian_pick_and_place(self):
        self.execute_cartesian_pick_up()
        self.execute_cartesian_place()

    def execute_pick_and_place(self):
        self.execute_pick_up()
        self.execute_place()

    def execute_cartesian_pick_up(self):
        move_group = self.moveit_control
        
        self.gripper.grasp(0.1)
        rospy.sleep(2)        
        
        waypoints = self.generate_waypoints(self.pick_pose, 1)
        for waypoint in waypoints:
            # self.moveit_control.follow_cartesian_path([waypoint])
            self.call_cartesian_service(waypoint)

        # self.gripper.grasp(self.gripper_pose)
        self.call_gripper_service(self.gripper_pose)
        rospy.sleep(3)
        
        waypoints = self.generate_waypoints(self.pick_pose, 2)
        for waypoint in waypoints:
            # move_group.follow_cartesian_path([waypoint])
            self.call_cartesian_service(waypoint)
    
        # rospy.sleep(2)        

    def execute_pick_up(self):
        move_group = self.moveit_control
        self.gripper.grasp(0.1)
        rospy.sleep(2)        

        waypoints = self.generate_waypoints(self.pick_pose, 1)        
        for waypoint in waypoints:
            # move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])
            self.call_vanilla_service(waypoint)

        # self.gripper.grasp(self.gripper_pose)
        self.call_gripper_service(self.gripper_pose)
        rospy.sleep(3)
            
        waypoints = self.generate_waypoints(self.pick_pose, 2)        
        for waypoint in waypoints:
            # move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])
            self.call_vanilla_service(waypoint)
        
        # rospy.sleep(2)        

    def execute_place(self):
        move_group = self.moveit_control

        waypoints = self.generate_waypoints(self.drop_pose, 3)        
        for waypoint in waypoints:
            # move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])
            self.call_vanilla_service(waypoint)

        # self.gripper.grasp(0.1)
        self.call_gripper_service(0.1)

        rospy.sleep(3)        
    
    def execute_cartesian_place(self):
        move_group = self.moveit_control

        waypoints = self.generate_waypoints(self.drop_pose, 3)
        for waypoint in waypoints:
            # move_group.follow_cartesian_path([waypoint])
            self.call_cartesian_service(waypoint)

        # self.gripper.grasp(0.1)
        self.call_gripper_service(0.1)

        rospy.sleep(3)        

    def reach_scanpose(self):
        move_group = self.moveit_control
        waypoints = self.generate_waypoints(self.scan_pose, 0)

        for waypoint in waypoints:
            # move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])
            self.call_vanilla_service(waypoint)

    def reach_cartesian_scanpose(self):
        move_group = self.moveit_control
        waypoints = self.generate_waypoints(self.scan_pose, 0)
 
        for waypoint in waypoints:
            # move_group.follow_cartesian_path([waypoint])
            self.call_cartesian_service(waypoint)
