from src.hsrobot import HSROBOT as hs_robot_arm
import numpy as np
import time

# -------------------------------连接机器人--------------------------------
hsrobot = hs_robot_arm()
hsrobot.arm.HRIF_GrpEnable(0,0) # 机器人使能
time.sleep(3.5)

#运动到初始位置
init_joint_pos = np.array([-10.956,17.118,-136.711,-7.59,67.931,38.165])
hsrobot.move_j(init_joint_pos,15,sTcpName="TCP_grasp")

# 读取实际关节位置变量
r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
r_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
print(f'机器人当前各关节位置(in degree): { [r_joint_pos[0], r_joint_pos[1], r_joint_pos[2], r_joint_pos[3], r_joint_pos[4], r_joint_pos[5]]}\n')

# 读取实际笛卡尔空间位置变量
r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')

#tcp向y（前）方向前进

#

#向左平移到预插接位置
