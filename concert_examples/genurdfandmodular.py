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

class encodegraph: 
    n = 4
    # Number of modules to take
    N = 8
    def __init__(self, m):
        self.state = np.zeros(self.n, dtype=np.uint64)
        self.m = 0

    def take_action(self):
        """Add module id m to `state`.
        `state` contains n integers representing the modular assignment sequence in the following way:
        e.g., a 2D array [12, 3] implies that there are two possible modules; the order of modules to take can be interpreted by converting the decimal numbers into the binary form and read the bits from left to right.
        i.e, [12, 3] corresponds to [[1, 1, 0, 0], [0, 0, 1, 1]] and hence the sequence of modular assignment is [Module 1, Module 1, Module 2, Module 2].
        To get the interpreted sequence of assignment, call method get_sequence().

        Args:
            m (int): Id of the module to take, starting from 0.

        Raises:
            ValueError: when m >= n (available number of modules).
        """
        print(self.m)
        if m < len(self.state):
            nbits = len(np.binary_repr(self.state.max())) if self.state.any() else 0
            if nbits > 0:
                binary_repr = np.binary_repr(self.state[self.m])
                padded_binary = "0" * (nbits - len(binary_repr)) + binary_repr
                self.state[self.m] = int("1" + padded_binary, 2)
            else:
                self.state[self.m] += 1
        else:
            raise ValueError("Action dimension is higher than the state dimension.")


    def get_sequence(self, from_root=False):
        """Output the sequence of modules to take. 

        Args:
            from_root (bool, optional): If True output sequence starts from the base link. Defaults to False.

        Returns:
            np.ndarray: Sequence of the module ids.
        """    
        nbits = len(np.binary_repr(self.state.max())) if self.state.any() else 0
        binary_strings = [np.binary_repr(s, nbits) for s in self.state]
        matrix = [[int(char) for char in bstr] for bstr in binary_strings]
        sequence = np.argmax(np.array(matrix), axis=0)
        return np.flip(sequence) if from_root else sequence



class solveIK: 
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


        self.Rx = 0
        self.Ry = 0
        self.Rz = 0

    def solution_IK(self, urdf_string, pose):
        rospy.wait_for_service('IK')
        client = rospy.ServiceProxy('IK', state_service)
        
        # self.x = float(pose[0])
        # self.y = float(pose[1])
        # self.z = float(pose[2])
         
        # print(pose)

        # self.Rx = float(pose[3])
        # self.Ry = float(pose[4])
        # self.Rz = float(pose[5]) 

        client.wait_for_service()
        req = state_serviceRequest()
        req.x = pose[0]
        req.y = pose[1]  
        req.z = pose[2]

        req.path = urdf_string
        
        resp = client.call(req)
        # print(resp.x_)  
        # print(resp.y_)  
        # print(resp.z_)         
        return resp.x_, resp.y_, resp.z_, resp.man_
    
    def solution_IK_traj(self, urdf_string, traj_pointposs):
        rospy.wait_for_service('IK')
        client = rospy.ServiceProxy('IK', state_service)
    
        client.wait_for_service()
        req = state_serviceRequest()
        response_msg = Float64MultiArray()  

        x_traj = []
        y_traj = []
        z_traj = []
        # x_traj = Float64MultiArray()  
        # y_traj = Float64MultiArray()  
        # z_traj = Float64MultiArray()  

        # print(traj_pointposs.shape[0])

        for i in range(traj_pointposs.shape[0]):
           x_traj.append(traj_pointposs[i][0])
           y_traj.append(traj_pointposs[i][1])
           z_traj.append(traj_pointposs[i][2])


        response_msg = x_traj
        req.x_traj = response_msg
        response_msg = y_traj
        req.y_traj = y_traj 
        response_msg = z_traj 
        req.z_traj = z_traj


        Rx_traj = []
        Ry_traj = []
        Rz_traj = []
        
        for i in range(traj_pointposs.shape[0]):
           Rx_traj.append(traj_pointposs[i][3])
           Ry_traj.append(traj_pointposs[i][4])
           Rz_traj.append(traj_pointposs[i][5])

        response_msg = Rx_traj
        req.Rx_traj = response_msg
        response_msg = Ry_traj
        req.Ry_traj = response_msg 
        response_msg = Rz_traj 
        req.Rz_traj = response_msg
   
        # req.Rx_traj = Rx_traj
        # req.Ry_traj = Ry_traj
        # req.Rz_traj = Rz_traj
                
        req.path = urdf_string
        
        # print(x_traj)
        # print(y_traj)
        # print(z_traj)
        # print(Rx_traj)
        # print(Ry_traj)
        # print(Rz_traj)
        # print(traj_pointposs)

        resp = client.call(req)
        # print(resp.x_)  
        # print(resp.y_)  
        # print(resp.z_)         
        return resp.x_, resp.y_, resp.z_, resp.man_, resp.Rx_,  resp.Ry_,  resp.Rz_
   
    def solution_IK_ori(self, urdf_string, pose):
        rospy.wait_for_service('IK')
        client = rospy.ServiceProxy('IK', state_service)
    
        client.wait_for_service()
        req = state_serviceRequest()

        
        req.x = pose[0]
        req.y = pose[1]  
        req.z = pose[2]

        req.Rx = pose[3]
        req.Ry = pose[4]
        req.Rz = pose[5]
                
        req.path = urdf_string
        
        resp = client.call(req)
        # print(resp.x_)  
        # print(resp.y_)  
        # print(resp.z_)         
        return resp.x_, resp.y_, resp.z_, resp.man_, resp.Rx_,  resp.Ry_,  resp.Rz_
   
    def TRAC_IK(self, urdf_string):

        ik_solver = IK("base_link",
                    "ee_A", urdf_string)

        lower_bound, upper_bound = ik_solver.get_joint_limits()

        print(lower_bound)
        print(upper_bound)

        seed_state = [0.0] * ik_solver.number_of_joints
        
        qw,qx,qy,qz = self.EulerAndQuaternionTransform( [self.Rx, self.Ry, self.Rz])
        solution = ik_solver.get_ik(seed_state,
                        0.45, 0.5, 0.3,
                        qw,qx,qy,qz)  
        print(solution)

        fk_pose = ik_solver.forward(solution) 

    def IK_Pino(self):
        euler_angles = [self.Rx, self.Ry, self.Rz]
        # r = Rotation.from_euler('ZYX', euler_angles, degrees=False)
        # print(r.as_matrix)
        # rotation_matrix = r.as_matrix()
        rotation_matrix = self.euler_to_matrix()
        model_path = "/home/mlei/concert_ws/ros_src/modularbot/urdf/ModularBot.urdf"
        model = pinocchio.buildModelFromUrdf(model_path)
        frame_name = "ee_A"
        # pinocchio
        # frame_id = pinocchio.findFrameID(model, frame_name)
        data  = model.createData()
        
        JOINT_number = int(model.njoints)
        print(JOINT_number)
        JOINT_ID = JOINT_number - 1
        print(JOINT_ID)
        JOINT_ID = int(JOINT_ID)
        rotation = np.eye(3,dtype=np.float64)
        for i in range(3):
           for j in range(3):
               rotation[i][j] = rotation_matrix[i][j]

        oMdes = pinocchio.SE3(np.eye(3), np.array([1., 1., 1.]))
        
        q      = pinocchio.neutral(model)
        eps    = 1e-4
        IT_MAX = 3000
        DT     = 1e-1
        damp   = 1e-12
        
        i=0
        dMi = oMdes.actInv(data.oMi[JOINT_ID])

        q_min = -2 * np.pi * np.ones(model.njoints)
        q_max = 2 * np.pi * np.ones(model.njoints)
        # print(data)
        err = pinocchio.log(dMi).vector
        while True:
            pinocchio.forwardKinematics(model,data,q)
            #  print(data)
            dMi = oMdes.actInv(data.oMi[JOINT_ID])
            #  print(data.oMi[JOINT_ID],"2b")
            err = pinocchio.log(dMi).vector
            if norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break
            J = pinocchio.computeJointJacobian(model,data,q,JOINT_ID)
            v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pinocchio.integrate(model,q,v*DT)
            # man_ =J * J.T
            # if not i % 100:
                # print('%d: error = %s' % (i, err.T))
            i += 1
        # print((err.T))     
        manipulate = J.dot(J.T)
        det = np.linalg.det(manipulate)
        q = ((q+np.pi) % (2 * np.pi)) - np.pi
        pinocchio.forwardKinematics(model,data,q)
            #  print(data)
        dMi = oMdes.actInv(data.oMi[JOINT_ID])
            #  print(data.oMi[JOINT_ID],"2b")
        err = pinocchio.log(dMi).vector
        print(err,"err")
        return err[0], err[1], err[2], det, err[3],  err[4],  err[5]


    def euler_to_matrix(self):
        R_x = np.array([[1,          0,           0],
                        [0, np.cos(self.Rz), -np.sin(self.Rz)],
                        [0, np.sin(self.Rz),  np.cos(self.Rz)]])

        R_y = np.array([[ np.cos(self.Ry), 0, np.sin(self.Ry)],
                        [0,           1,          0],
                        [-np.sin(self.Ry), 0, np.cos(self.Ry)]])

        R_z = np.array([[np.cos(self.Rx), -np.sin(self.Rx), 0],
                        [np.sin(self.Rx),  np.cos(self.Rx), 0],
                        [          0,           0,          1]])

        return R_z @ R_y @ R_x

    def quaternion_error(q_desired, q_current):
        # Calculate the inverse of q_current
        q_inv = np.array([q_current[0], -q_current[1], -q_current[2], -q_current[3]])

        # Calculate the quaternion error
        q_err = np.matmul(q_desired, q_inv)

        return q_err



    def EulerAndQuaternionTransform( intput_data):
        """
            四元素与欧拉角互换
        """
        data_len = len(intput_data)
        angle_is_not_rad = False
    
        if data_len == 3:
            r = 0
            p = 0
            y = 0
            if angle_is_not_rad: # 180 ->pi
                r = math.radians(intput_data[0]) 
                p = math.radians(intput_data[1])
                y = math.radians(intput_data[2])
            else:
                r = intput_data[0] 
                p = intput_data[1]
                y = intput_data[2]
    
            sinp = math.sin(p/2)
            siny = math.sin(y/2)
            sinr = math.sin(r/2)
    
            cosp = math.cos(p/2)
            cosy = math.cos(y/2)
            cosr = math.cos(r/2)
    
            w = cosr*cosp*cosy + sinr*sinp*siny
            x = sinr*cosp*cosy - cosr*sinp*siny
            y = cosr*sinp*cosy + sinr*cosp*siny
            z = cosr*cosp*siny - sinr*sinp*cosy
            return [w,x,y,z]
    
        elif data_len == 4:
    
            w = intput_data[0] 
            x = intput_data[1]
            y = intput_data[2]
            z = intput_data[3]
    
            r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            p = math.asin(2 * (w * y - z * x))
            y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    
            if angle_is_not_rad : # pi -> 180
                r = math.degrees(r)
                p = math.degrees(p)
                y = math.degrees(y)
            return [r,p,y]


    def visualize(self,pose):
        # rospy.init_node('rviz_marker')

        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2      

        # Set the scale of the marker
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.20
        marker.color.b = 1.00
        marker.color.a = 0.8

        # Set the pose of the marker
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker_pub.publish(marker)
        rospy.rostime.wallsleep(5.0)


    def talker(self, joint, pub, number_joint, pose2):

        rate = rospy.Rate(10) # 10hz
        hello_str = JointState()
        hello_str.position = joint 

        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2      

        # Set the scale of the marker
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.00
        marker.color.b = 1.00
        marker.color.a = 0.8

        # Set the pose of the marker
        marker.pose.position.x = float(pose2[0])
        marker.pose.position.y = float(pose2[1])
        marker.pose.position.z = float(pose2[2])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        
        # rospy.rostime.wallsleep(5.0)


        Table_ = ['J1_A', 'J2_A', 'J3_A', 'J4_A','J5_A','J6_A', 'J7_A', 'J8_A']   
        joint_name = []
        for i in range(number_joint):
            joint_name.append(Table_[i])
        hello_str.name = joint_name
        done = False
        loop = 0
        while not done:
            hello_str.header.stamp = rospy.Time.now()            
            pub.publish(hello_str)
            marker_pub.publish(marker)
            loop = loop +1
            if loop >= 20 : done = True
            rate.sleep()


# #   rospy.rostime.wallsleep(1.0)


class results_show: 
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


        self.Rx = 0
        self.Ry = 0
        self.Rz = 0



    def visualize(self,pose):
        # rospy.init_node('rviz_marker')

        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2      

        # Set the scale of the marker
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.20
        marker.color.b = 1.00
        marker.color.a = 0.8

        # Set the pose of the marker
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = float(pose[2])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker_pub.publish(marker)
        rospy.rostime.wallsleep(5.0)


    def talker(self, joint, pub, number_joint, pose2):

        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

        rate = rospy.Rate(10) # 10hz
        hello_str = JointState()
        hello_str.position = joint 

        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2      

        # Set the scale of the marker
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.00
        marker.color.b = 1.00
        marker.color.a = 0.8

        # Set the pose of the marker
        marker.pose.position.x = float(pose2[0])
        marker.pose.position.y = float(pose2[1])
        marker.pose.position.z = float(pose2[2])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker_pub.publish(marker)
        rospy.rostime.wallsleep(5.0)



        Table_ = ['J1_A', 'J2_A', 'J3_A', 'J4_A','J5_A','J6_A', 'J7_A', 'J8_A']   
        joint_name = []
        for i in range(number_joint):
            joint_name.append(Table_[i])
        hello_str.name = joint_name
        done = False
        loop = 0
        while not done:
            hello_str.header.stamp = rospy.Time.now()            
            pub.publish(hello_str)
            marker_pub.publish(marker)
            loop = loop +1
            if loop >= 20 : done = True
            rate.sleep()


# #   rospy.rostime.wallsleep(1.0)


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

class showresult: 
    def __init__(self, inputpath):
        # POS_data1 = []
        # POS_data2 = []
        # POS_data3 = []
        self.path = inputpath

    def read_txt(self):
        with open(self.path, 'r', encoding='utf-8') as infile:

            datamiddle = []
            data = []
            lines=infile.readline()
            # print(lines)
            lines = 0
            for line in infile:
                data_line = line.strip("\n").split()  # 去除首尾换行符，并按空格划分
                # print(float(data_line))
                # data2.append([int(data_line-3) for i in data_line])
                lines = lines +1
                # print(data_line)
                datamiddle.append(data_line)
                # print(datamiddle)
                # 输出：[[1, 2, 3, 4, 5], [6, 7, 8, 9, 10]]

                # 输出：[['1', '2', '3', '4', '5'], ['6', '7', '8', '9', '10']]

            # for line in infile:
            #     if line >= lines-4:
            #       data_line = line.strip("\n").split()  # 去除首尾换行符，并按空格划分
            #       datamiddle.append(data_line)
            #     print("data")
            #     print(datamiddle)  
        POS_data1 = datamiddle[lines-3]
        POS_data2 = datamiddle[lines-2]
        POS_data3 = datamiddle[lines-1]
        print(POS_data1)
        print(POS_data2)  
        print(POS_data3)          
        # print(data)        
        return POS_data1, POS_data2, POS_data3
 
# if __name__ == "__main__":
#     urdf_map = [5, 1, 4, 0 ,1 ,0 ,1 ]

#     urdf_map = [2, 1, 3, 2, 1, 0, 1, 0, 1] 
#     s = urdfgen(urdf_map)
#     urdf_string = s.urdf_generate()

# if __name__ == "__main__":
#     input_path_point = '/home/mlei/catkin_ws/src/mpc/fcl_test/src/traning_MPC/desirpoint.txt'
#     input_path_joint = '/home/mlei/catkin_ws/src/mpc/fcl_test/src/traning_MPC/Joint.txt'

#     rospy.init_node('joint_state_publisher')
#     pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

#     showresult_point = showresult(input_path_point)
#     pose1, pose2, pose3 = showresult_point.read_txt()

#     showresult_joint = showresult(input_path_joint)
#     joint1, joint2, joint3 = showresult_joint.read_txt()
    
#     marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

#     ik1 = solveIK()
#     # ik1.visualize(pose1)

#     for i in range(len(joint1)):
#         joint1[i] = float(joint1[i])
  
#     ik1.talker(joint1, pub,len(joint1),pose1)

#     ik1 = solveIK()
#     # ik1.visualize(pose2)

#     for i in range(len(joint2)):
#         joint2[i] = float(joint2[i])
  
#     ik1.talker(joint2, pub,len(joint2),pose2)
  
#     # ik2.talker(joint2,pub,len(joint2), pose2)


#     ik3 = solveIK()
#     # ik3.visualize(pose3)

#     for i in range(len(joint3)):
#         joint3[i] = float(joint3[i])
  
#     ik1.talker(joint3, pub,len(joint3),pose3)





urdf_map = [1, 0, 1, 0, 1 ]
# urdf_map = [1, 1, 3, 2, 1, 0, 1  , 1]
s = urdfgen(urdf_map)
urdf_string = s.urdf_generate()




# des_pose = [-0.5, -0.3, 0.5,  0, -math.pi/2 , 0]
# # # des_pose = [0.2, -0.2, 0.6, 1.2092, 1.2092 , -1.2092]
# ik = solveIK(des_pose)

# # R = ik.euler_to_matrix()
# # print(R)
# x_,y_,z_, man ,Rx_,Ry_,Rz_ = ik.IK_Pino()

# x_,y_,z_ ,man, Rx_,Ry_,Rz_ = ik.solution_IK_ori(urdf_string)

# print(x_,y_,z_ ,Rx_,Ry_,Rz_ )




# rospy.set_param('/robot_description', urdf_string)

# pi = 3.1415926

# des_pose = [0.25, 0.55, 0.2, 0.8, 0.2, 3.14/3]

# des_pose = np.array([0.4, 0.5, 0.8, 0, 0.2, 3.14/3])
# print(des_pose)
# ik = solveIK(des_pose)
# x_,y_,z_, man ,Rx_,Ry_,Rz_ = ik.solution_IK_ori(urdf_string)

# print(Rx_)








# rospy.init_node('rviz_marker')

# marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

# marker = Marker()

# marker.header.frame_id = "ci/world"
# marker.header.stamp = rospy.Time.now()

# # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
# marker.type = 2
# marker.id = 0

# # Set the scale of the marker
# marker.scale.x = 1.0
# marker.scale.y = 1.0
# marker.scale.z = 1.0

# # Set the color
# marker.color.r = 0.0
# marker.color.g = 1.0
# marker.color.b = 0.0
# marker.color.a = 1.0

# # Set the pose of the marker
# marker.pose.position.x = 0
# marker.pose.position.y = 0
# marker.pose.position.z = 0
# marker.pose.orientation.x = 0.0
# marker.pose.orientation.y = 0.0
# marker.pose.orientation.z = 0.0
# marker.pose.orientation.w = 1.0

# while not rospy.is_shutdown():
#   marker_pub.publish(marker)
#   rospy.rostime.wallsleep(1.0)