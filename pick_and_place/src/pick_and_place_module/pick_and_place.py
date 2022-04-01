import rospy
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper
from copy import deepcopy
from math import pi

class PickAndPlace:
    def __init__(self, gripper_offset, intermediate_z_stop, scan_pose_x=0.0, scan_pose_y=0.3, scan_pose_z=0.6):
        self.gripper_offset = gripper_offset
        self.intermediate_z_stop = intermediate_z_stop
        self.scan_pose_x = scan_pose_x
        self.scan_pose_y = scan_pose_y
        self.scan_pose_z = scan_pose_z
        self.pick_pose = None
        self.place_pose = None
        self.gripper_pose = None
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()
    
    def setPickPose(self, x, y, z, roll, pitch, yaw):
        self.pick_pose = [x, y, z, roll + pi/4, pitch, yaw]
    
    def setDropPose(self, x, y, z, roll, pitch, yaw):
        self.drop_pose = [x, y, z, roll + pi/4, pitch, yaw]
    
    def setGripperPose(self, finger1, finger2):
        self.gripper_pose = [finger1, finger2]
    
    def generate_waypoints(self, destination_pose, action):
        '''
        Generated waypoints are for a particular application
        This is to be changed based on the application it is being used
        '''
        move_group = self.moveit_control

        waypoints = []

        if action:
            current_pose = move_group.get_current_pose()
            current_pose_ = deepcopy(destination_pose)
            current_pose_[0] = current_pose.position.x
            current_pose_[1] = current_pose.position.y
            current_pose_[2] = self.intermediate_z_stop
            waypoints.append(current_pose_)
        
        intermediate_pose = deepcopy(destination_pose)
        intermediate_pose[2] = self.intermediate_z_stop
        waypoints.append(intermediate_pose)

        if not action:
            destination_pose_ = deepcopy(destination_pose)
            destination_pose_[2] = destination_pose_[2]  + 0.1 
            waypoints.append(destination_pose_)

            destination_pose_ = deepcopy(destination_pose)
            destination_pose_[2] = destination_pose_[2]  + self.gripper_offset 
            waypoints.append(destination_pose_)
        
        return waypoints
    
    def execute_cartesian_pick_and_place(self):
        self.execute_cartesian_pick_up()
        self.execute_cartesian_place()

    def execute_pick_and_place(self):
        self.execute_pick_up()
        self.execute_place()

    def execute_cartesian_pick_up(self):
        move_group = self.moveit_control
        
        self.gripper.grasp(0.05, 0.05)
        rospy.sleep(2)        
        
        waypoints = self.generate_waypoints(self.pick_pose, 0)
        for waypoint in waypoints:
            self.moveit_control.follow_cartesian_path([waypoint])

        self.gripper.grasp(self.gripper_pose[0], self.gripper_pose[1])
        rospy.sleep(3)
        
        waypoints = []
        current_pose_ = deepcopy(self.pick_pose)
        current_pose_[2] = self.intermediate_z_stop
        waypoints.append(current_pose_)

        for waypoint in waypoints:
            move_group.follow_cartesian_path([waypoint])

        # rospy.sleep(2)        

    def execute_pick_up(self):
        move_group = self.moveit_control

        self.gripper.grasp(0.05, 0.05)
        rospy.sleep(2)        

        waypoints = self.generate_waypoints(self.pick_pose, 0)        
        for waypoint in waypoints:
            move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])

        self.gripper.grasp(self.gripper_pose[0], self.gripper_pose[1])
        rospy.sleep(3)
            
        waypoints = []
        current_pose = move_group.get_current_pose()
        current_pose[2] = self.intermediate_z_stop
        waypoints.append(deepcopy(current_pose))
        
        for waypoint in waypoints:
            move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])
                        
        # rospy.sleep(2)        

    def execute_place(self):
        move_group = self.moveit_control
        waypoints = self.generate_waypoints(self.drop_pose, 1)
        
        for waypoint in waypoints:
            move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])
                        
        self.gripper.grasp(0.05, 0.05)
        rospy.sleep(3)        

    
    def execute_cartesian_place(self):
        move_group = self.moveit_control
        waypoints = self.generate_waypoints(self.drop_pose, 1)

        for waypoint in waypoints:
            move_group.follow_cartesian_path([waypoint])

        self.gripper.grasp(0.05, 0.05)
        rospy.sleep(3)        

    def reach_scanpose(self):
        move_group = self.moveit_control

        move_group.follow_cartesian_path([])
        current_pose = move_group.get_current_pose()
        current_pose[0] = self.scan_pose_x
        current_pose[1] = self.scan_pose_y
        current_pose[2] = self.scan_pose_z
        waypoints.append(deepcopy(current_pose))

        for waypoint in waypoints:
            move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])

    def reach_cartesian_scanpose(self):
        move_group = self.moveit_control

        move_group.follow_cartesian_path([])
        current_pose = move_group.get_current_pose()
        current_pose[0] = self.scan_pose_x
        current_pose[1] = self.scan_pose_y
        current_pose[2] = self.scan_pose_z
        waypoints.append(deepcopy(current_pose))

        for waypoint in waypoints:
            move_group.follow_cartesian_path([waypoint])
