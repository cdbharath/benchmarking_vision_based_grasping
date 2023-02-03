import rospy
from std_msgs.msg import Float64
from franka_gripper.msg import GraspActionGoal

class Gripper:
    def __init__(self):
        self.grasp_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10) 
        rospy.sleep(1)
        
    def grasp(self, width):
        grasp_data = GraspActionGoal()
        grasp_data.goal.width = width
        grasp_data.goal.force = 0.7
        grasp_data.goal.speed = 0.2
        grasp_data.goal.epsilon.inner = 0.5
        grasp_data.goal.epsilon.outer = 0.5

        self.grasp_pub.publish(grasp_data)        
        
