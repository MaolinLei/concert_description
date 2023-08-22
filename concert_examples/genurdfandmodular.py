#!/usr/bin/env python3

from modular.URDF_writer import *
import numpy as np



from numpy.linalg import norm, solve
import pinocchio

from scipy.spatial.transform import Rotation

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import sys
from std_msgs.msg import Float64MultiArray
from FCL_TEST.srv import state_service, state_serviceRequest ,  state_serviceResponse


from trac_ik_python.trac_ik import IK
import math 
# import PyKDL as kdl



class urdfgen: 

   # homing_joint_map = {}
   # urdf_writer = UrdfWriter(speedup=True)
    
    def __init__(self, index_sequence):
        self.homing_joint_map = {}
        self.urdf_writer = UrdfWriter(speedup=True)
        self.index_sequence = index_sequence
        self.angle = -0.30
        self.index = 0

    def urdf_generate(self):
        self.urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

        # print("self.index_sequence = ")
        # print(self.index_sequence)
        for i in range(len(self.index_sequence)):
            self.index = self.index_sequence[i]
            # print(self.index)
            self.homing_joint_map = self.robot_assemble()

        self.urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)


           # write_file_to_stdout(urdf_writer, homing_joint_map)

        # self.urdf_writer.remove_connectors()

        # self.urdf_writer.write_urdf()
        # self.urdf_writer.write_lowlevel_config()
        # self.urdf_writer.write_problem_description_multi()
        # self.urdf_writer.write_srdf(self.homing_joint_map)
        # self.urdf_writer.write_joint_map()
         
        # self.urdf_writer.deploy_robot("modularbot" ,"/home/mlei/concert_ws/ros_src")

        urdf_string = self.urdf_writer.process_urdf()

        return urdf_string
    
    def get_urdf(self):
        self.urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

        # print("self.index_sequence = ")
        # print(self.index_sequence)
        for i in range(len(self.index_sequence)):
            self.index = self.index_sequence[i]
            # print(self.index)
            self.homing_joint_map = self.robot_assemble()

        self.urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)


        # write_file_to_stdout(urdf_writer, homing_joint_map)
        self.urdf_writer.remove_connectors()

        urdf_string = self.urdf_writer.process_urdf()

        return urdf_string
        
    def Table_Link(self):
        Table_ = ['module_joint_yaw_ORANGE.yaml','module_joint_double_elbow_ORANGE.yaml',
                   'module_link_elbow_90.yaml', 'module_link_straight_140.yaml', 
                    'concert/module_link_straight_10_concert.json', 'concert/module_link_straight_20_concert.json',
                    'concert/module_link_straight_50_concert.json']
        Name = Table_[self.index]
        return Name

    def robot_assemble(self):

        modular =  self.Table_Link()
        data =  self. urdf_writer.add_module(modular, 0, False)
        self.homing_joint_map[str(data['lastModule_name'])] = {'angle': self.angle}

        return self.homing_joint_map

 
# if __name__ == "__main__":
    urdf_map = [1, 0, 1, 0, 1 ]
    # urdf_map = [1, 1, 3, 2, 1, 0, 1  , 1]
    s = urdfgen(urdf_map)
    urdf_string = s.urdf_generate()
