'''
Date: 3.12.2024
Author: Shuai Gan
Copyright by Casia. Robotic Theory and Application Group
Decription: This is the main program for the robot control.
'''

import time
import roboticstoolbox as rtb
import numpy as np
import math
from vision import RealsenseD435
import spatialmath.base as spatialmathbase
from hsrobot import HSROBOT as hs_robot_arm

# -------------------------------连接机器人--------------------------------
hsrobot = hs_robot_arm()

# -----------------------------获取机器人状态-------------------------------
# 读取实际关节位置变量
r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
r_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
print(f'机器人当前各关节位置(in mm): [{ r_joint_pos[0], r_joint_pos[1], r_joint_pos[2], r_joint_pos[3], r_joint_pos[4], r_joint_pos[5]}]\n')
# 读取实际笛卡尔空间位置变量
r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
print(f'机器人当前TCP位姿为(in degree): [{ r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]}]\n')

# --------------------------------Sensor----------------------------------
# realsenseD435 = RealsenseD435()
# cam_grasp_point = realsenseD435.vision_module_output()
# # print(f'相机坐标系下四个角点的齐次坐标为: {cam_grasp_point}')
# [d_tcp_pos,d_tcp_ori] = hsrobot.get_TCP_targetPose(cam_grasp_point)
# print(f'末端期望pos: {d_tcp_pos}\n 末端期望ori: {d_tcp_ori}')

# 读取实际力传感器数据
result = []; hsrobot.arm.HRIF_ReadFTCabData(0,0,result)
r_force_data = np.array([float(i) for i in result[0:6]]) 
print(f'机器人当前六维力传感器数据为(in N/Nm): [{r_force_data[0], r_force_data[1], r_force_data[2], r_force_data[3], r_force_data[4], r_force_data[5]}]\n')

# hsrobot.arm.HRIF_SetForceZero(0,0) # 设置力传感器零点
# hsrobot.arm.HRIF_ReadFTCabData(0,0,result)
# r_force_data = np.array([float(i) for i in result[0:6]]) 
# print(f'机器人当前六维力传感器数据为(in N/Nm): [{r_force_data[0], r_force_data[1], r_force_data[2], r_force_data[3], r_force_data[4], r_force_data[5]}]\n')

# --------------------------------Control---------------------------------
'''
开启力控模式，确保以下步骤已执行：
(1) 负载辨识，示教器上运行<配置><设置TCP><负载辨识>
(2) 力传感器标定，

'''
# 设置力控状态 0：关闭力控 1：开启力控
# hsrobot.arm.HRIF_SetForceControlState(0,0,1)

# 定义力控状态码到描述的映射  
force_control_states = {  
    0: "未开启力控",  
    1: "力控探寻中",  
    2: "力控探寻完成,保持恒力",  
    3: "力控自由驱动"  
}  
  
# 读取当前力控状态  
result = []; hsrobot.arm.HRIF_ReadForceControlState(0, 0, result)  
nState = int(result[0])  
  
# 通过字典查找状态描述  
state_description = force_control_states.get(nState, "未知的力控状态")  
print(f"力控处于: {state_description}")

'''
# 定义关节目标位置
RawACSpoints = [0, 0, 90, 0, 90, 0]
# 定义工具坐标变量 & 定义用户坐标变量
sTcpName = "TCP_grasp"; sUcsName = "Base"
# 定义运动最大速度，单位[°/s] & 定义运动最大加速度，单位[°/s 2]
dVelocity = 5; dAcc = 10
# 定义过渡半径，单位[mm]
dRadius = 50
# 定义是否使用关节角度
nIsUseJoint= 0
# 定义是否使用检测 DI 停止 & 定义检测的 DI 索引 & 定义检测的 DI 状态 & 定义路点 ID
nIsSeek = 0; nIOBit = 0; nIOState = 0; stdCmdID = "0"
# 执行路点运动
nRet = cps.HRIF_MoveJ(0,0,tcp_tar_p,RawACSpoints,sTcpName,sUcsName,dVelocity,dAcc,dRadius,nIsUseJoint, nIsSeek, nIOBit, nIOState, stdCmdID)
'''

# 机器人使能
nRet = hsrobot.arm.HRIF_GrpEnable(0,0)
if nRet == 0:
    print('机器人使能成功')
else:
    print('机器人使能失败')
   
# 夹具打开
hsrobot.gripper_open()
hsrobot.gripper_init()
time.sleep(10)

hsrobot.gripper_close()
hsrobot.gripper_init()
time.sleep(6)   




# ------------------------断开连接---------------------
# 机器人去使能
# nRet = hsrobot.arm.HRIF_GrpDisable(0,0)

'''
# 机器人断电
nRet = cps.HRIF_Blackout(0)
# 控制器断电
nRet = cps.HRIF_ShutdownRobot(0)
# 断开连接机器人服务器
nRet = cps.HRIF_DisConnect(0)
'''
