#AR3 Env
# Robot Environment 

import numpy as np
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
#from nav_msgs.msg import Odometry
#from fetch_moveit_config.fetch_commander import FetchCommander
import geometry_msgs.msg



class AR3Env():

    def __init__(self):
        print('Entered Ar3 Env')
    
        # We Start all the ROS related Subscribers and publishers

        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joints_callback)
        self.joints = JointState()

        # Variables that we give through the constructor.

        self.controllers_list = ['joint_state_controller','position_controllers','fake_controller_joint_states']

        #self.robot_name_space = "ar3"

        print("Entered AR3 Env END")


    def joints_callback(self, data):
        self.joints = data

    def get_joints(self):
        return self.joints

    def set_trajectory_ee(self,action):
        """
        Sets the enf effector position and orientation
        """
        ee_pose = geometry_msgs.msg.Pose()
        ee_pose.position.x = action[0]
        ee_pose.position.y = action[1]
        ee_pose.position.z = action[2]
        ee_pose.orientation.x = 0.0
        ee_pose.orientation.y = 0.0
        ee_pose.orientation.z = 0.0
        ee_pose.orientation.w = 1.0

        return True
    
    def set_trajectory_joints(self,initial_qpos):
        
