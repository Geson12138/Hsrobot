import copy
from typing import List
import sys

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

import pandas as pd
import numpy as np
import cv2
import time
import numpy as np
import math
import spatialmath.base as spatialmathbase
# from src.hsrobot import HSROBOT as hs_robot_arm
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32


from matplotlib import pyplot as plt



class HSRobotdes:
    def __init__(self):
        d1 = 0.26
        d4 = 0.52
        d6 = 0.192
    

        a3 = 0.48

        self.dof = 6
        self.q0 = [0.0, 0.0, 0.0, 0.0, 0.0]

        alpha_array = [0, np.pi/2, np.pi, np.pi/2, -np.pi/2, np.pi/2]
        a_array = [0.0, 0, a3, 0, 0, 0.0]
        d_array = [d1, 0.0, 0, d4, 0, d6]
        theta_array = [0,  np.pi/2, np.pi/2, 0, 0, 0]
        self.tool = SE3.Trans(0.0, 0.0, 0.192)
        self.bias = SE3.Trans(0.0, 0.0, -0.192)

        links = []
        for i in range(6):
            links.append(rtb.DHLink(d=d_array[i], alpha=alpha_array[i], a=a_array[i], offset=theta_array[i], mdh=True))
        self.robot = rtb.DHRobot(links)

        self.alpha_array = alpha_array
        self.a_array = a_array
        self.d_array = d_array
        self.theta_array = theta_array

    def fkine(self, q) -> SE3:
        return self.robot.fkine(q)

    def ikine(self, Tep):
        result = self.robot.ikine_NR(Tep, q0=self.q0)
        if result.success:
            return result.q
        return []

    def move_cartesian(self, T: SE3):
        q = self.ikine(T)

        assert len(q)  # inverse kinematics failure
        self.set_joint(q)

    def set_joint(self, q):
        self.q0 = q[:]

    def get_joint(self):
        return copy.deepcopy(self.q0)

    def get_cartesian(self):
        return self.fkine(self.q0)
    
    def get_wrist2base(self):
        return self.fkine(self.q0).A
    
    def fkine_to_joint5(self, q):
        q5 = q[:5]
        # 创建一个只包含前5个关节的新机器人模型
        robot5 = rtb.DHRobot(self.robot.links[:5])
        # 计算前向运动学
        return robot5.fkine(q5)
        
        

    # #构建机械臂的包围盒
    # def get_geometries(self) -> List[Geometry]:
    #     Ts = []
    #     T = SE3()
    #     for i in range(self.dof):
    #         T = T * HSRobotdes.transform_mdh(self.alpha_array[i], self.a_array[i], self.d_array[i], self.theta_array[i],
    #                                     self.q0[i])
    #         Ts.append(T)

    #     # 机械臂
    #     T1 = Ts[0] * SE3.Trans(0, 0, -0.26)
    #     geometry1 = Capsule(T1, 0.105, 0.343)

    #     T2 = Ts[1] * SE3.Trans(-0.075, 0, 0.174) * SE3.Ry(np.pi / 2)
    #     geometry2 = Capsule(T2, 0.0725, 0.615)

    #     T3 = Ts[2] * SE3.Trans(0, -0.0676, 0) * SE3.Rx(-np.pi / 2)
    #     geometry3 = Capsule(T3, 0.059, 0.212)

    #     T4 = Ts[3] * SE3.Trans(0.0, -0.105, -0.374)
    #     geometry4 = Capsule(T4, 0.06, 0.434)

    #     T5 = Ts[4] * SE3.Trans(0.0, 0.0, -0.0647)
    #     geometry5 = Capsule(T5, 0.046, 0.14)

    #     T6 = Ts[5] * SE3.Trans(0.0, 0.0, -0.149)
    #     geometry6 = Capsule(T6, 0.0535, 0.149)

    #     #相机
    #     T7 = Ts[5] *SE3.Trans(0.0, 0.0, -0.012)
    #     geometry7 = Capsule(T7, 0.1, 0.006)

    #     #添加末端工具

    #     return [geometry1, geometry2, geometry3, geometry4, geometry5, geometry6, geometry7]

    @staticmethod
    def transform_mdh(alpha, a, d, theta, q) -> SE3:
        return SE3.Rx(alpha) * SE3.Trans(a, 0, d) * SE3.Rz(theta + q)

# def get_image_frome_ros(i):
        
#     time_count=0
#     while True:
#         color_img = rospy.wait_for_message("/hs_camera/color/image_raw", Image, timeout=None)
#         depth_img = rospy.wait_for_message("/hs_camera/aligned_depth_to_color/image_raw", Image, timeout=None)
#         time_count = time_count +1
#         if time_count>= 10:
#             break

#     color_image = bridge.imgmsg_to_cv2(color_img, 'bgr8')
#     cv2.imwrite(f'./calib/calib_image_{i}.jpg',color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])


# if __name__ == '__main__':
#     hs_robot = HSRobotdes()
    
#     bridge = CvBridge() 
#     rospy.init_node('get_images', anonymous=True)
#     # 定义一个ros话题，把self.color_image发布出去
#     image_pub = rospy.Publisher('image_topic', Image, queue_size=10)

#     # def get_image_frome_ros(i):
            
#     #     time_count=0
#     #     while True:
#     #         color_img = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=None)
#     #         depth_img = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=None)
#     #         time_count = time_count +1
#     #         if time_count>= 10:
#     #             break

#     #     color_image = bridge.imgmsg_to_cv2(color_img, 'bgr8')
#     #     cv2.imwrite(f'./calib/calib_image_{i}.jpg',color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])

#     # -------------------------------连接机器人--------------------------------
#     hsrobot = hs_robot_arm()

#     # ------------------------------定义标定位姿--------------------------------
#     i_tcp_pose = np.array([-454, -50 , 540, 170, 0, 153]) # 初始标定位姿
#     x_plus = 40
#     y_plus = 40
#     z_plus = 100
#     theta_x_plus = 5
#     theta_y_plus = 8
#     theta_z_plus = 8
#     # 定义关节目标空间位置
#     tcp_pose = {
#         0 : [0, 0, 0, 0, 0, 0], # unchanged

#         1 : [0, 0, 0, theta_x_plus, 0, 0], # +rx
#         2 : [x_plus, y_plus, 0, 0, 0, 0], # +x+y

#         3 : [0, 0, 0, -theta_x_plus, 0, 0], # -rx
#         4 : [-x_plus, -y_plus, 0, 0, 0, 0], # -x-y

#         5 : [0, 0, 0, 0, theta_y_plus, 0], # +ry
#         6 : [x_plus, -y_plus, 0, 0, 0, 0], # +x-y

#         7 : [0, 0, 0, 0, -theta_y_plus, 0], # -ry
#         8 : [-x_plus, y_plus, 0, 0, 0, 0], # -x+y

#         9 : [0, 0, 0, 0, 0, theta_z_plus], # +rz
#         10 : [0, 0, 0, 0, 0, -theta_z_plus], # -rz

#         11 : [x_plus/2, 0, 0, theta_x_plus, theta_y_plus, 0], # +rx+ry
#         12 : [-x_plus/2, 0, 0, -theta_x_plus, -theta_y_plus, 0], # -rx-ry
#         13 : [0, y_plus/2, 0, -theta_x_plus, theta_y_plus, 0], # -rx+ry
#         14 : [0, -y_plus/2, 0, theta_x_plus, -theta_y_plus, 0], # +rx-ry

#         15 : [0, 0, -z_plus, theta_x_plus, 0, theta_z_plus], # +rx+rz
#         16 : [0, 0, 0, -theta_x_plus, 0, -theta_z_plus], # -rx-rz
#         17 : [0, 0, -z_plus, -theta_x_plus, 0, theta_z_plus], # -rx+rz
#         18 : [0, 0, 0, theta_x_plus, 0, -theta_z_plus], # +rx-rz

#         19 : [0, 0, -z_plus, 0, theta_y_plus, theta_z_plus], # +ry+rz
#         20 : [0, 0, 0, 0, -theta_y_plus, -theta_z_plus], # -ry-rz
#         21 : [0, 0, -z_plus, 0, -theta_y_plus, theta_z_plus], # -ry+rz
#         22 : [0, 0, 0, 0, theta_y_plus, -theta_z_plus], # +ry-rz


#     }
#     count = 23
#     pose_files = []
#     file_path = './calib/pose.xlsx'
#     df = pd.read_excel(file_path, engine='openpyxl') 
#     for i in list(range(count)):
#         print(f'当前位姿编号: {i}')
#         # ----------------------------- 机器人运动到标定位姿--------------------------
#         # d_tcp_pose = i_tcp_pose + tcp_pose[i]
#         input(f'按回车键继续')
#         # print(f'末端期望位姿: {d_tcp_pose}\n')
#         hsrobot.arm.HRIF_SetOverride(0,0,0.3) # 设置速度
#         # hsrobot.move_l(d_tcp_pose,30)
#         time.sleep(1)

#         # ----------------------------- 获取视觉图像 --------------------------
#         # get_image_frome_ros(i)

#         r_poselist = [] # 定义返回值空列表
#         hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)

#         # 读取实际笛卡尔空间位置变量
#         r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
#         r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
#         # print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')

#         c_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
#         q_cur = c_joint_pos[0:5]/180*np.pi # 机器人前五个关节位置.
        
#         temp_T = hs_robot.fkine_to_joint5(q_cur) #运用机器人工具箱得到第五个关节的位姿矩阵
#         # temp_T = np.array(temp_T)
#         temp_r = temp_T.A[:3,:3];pos = temp_T.A[:3,3]*1000

#         euler_angles = spatialmathbase.tr2rpy(temp_r)
#         ori = np.array([np.rad2deg(j) for j in euler_angles]) 

#         # pose = np.concatenate((r_tcp_ori,r_tcp_pos.T))
#         pose = np.concatenate((ori, pos))
#         print(f'工具箱pos: {pose}')
#         df.loc[i+2] = pose
#         df.to_excel(file_path, index=False)
#         # print(f'工具箱pos: {pos}, 直接读pos: {r_tcp_pos}')
#         # print(f'工具箱ori: {ori}, 直接读ori: {r_tcp_ori}')
#         get_image_frome_ros(i)

#         # pose_files.append(pose)
#         # print(f'当前位姿: {pose}\n')

#     # 读取Excel文件  
#     # df = pd.read_excel(file_path, engine='openpyxl') 
#     # pose_files = np.array(pose_files)
#     # print(pose_files)

#     # # 写入pose_files到DataFrame
#     # for i in list(range(count)):
#     #     df.loc[i+2] = pose_files[i,:]

#     # 将修改后的DataFrame保存回原Excel文件  
#     # df.to_excel(file_path, index=False)  # index=False表示不保存行索引到Excel文件  