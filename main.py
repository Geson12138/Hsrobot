'''
Decription: Welcome, this is the main program to control the robot in BQ project!
Updated: 4.12.2024
Author: Shuai Gan
Mail: shuai.gan@ia.ac.cn
Copyright by CASIA. Robotic Theory and Application Group
'''
import time
import numpy as np
import math
from src.vision import RealsenseD435i
import spatialmath.base as spatialmathbase
from src.hsrobot import HSROBOT as hs_robot_arm

# -------------------------------连接机器人--------------------------------
hsrobot = hs_robot_arm()
nRet = hsrobot.arm.HRIF_GrpEnable(0,0) # 机器人使能
if nRet == 0:
    print('机器人使能成功')
else:
    print('机器人使能失败')
time.sleep(3.5)

'''
prefix description: i_ for initial; r_ for real; d_ for desired; f_ for fixed
'''
# ----------------------------- 运动到初始位姿-------------------------------
i_tcp_pose = np.array([-687,-11,333,-180,0,90]) # 偏航 俯仰 翻滚
# 定义笛卡尔空间目标初始位置
# hsrobot.move_l(i_tcp_pose,50,sTcpName="TCP_grasp")

# 定义关节目标空间初始位置
# i_joint_pos = np.array([0.757, -0.407, -147.771, -0.033, 39.295, 13.386])
# hsrobot.move_j(i_joint_pos,15,sTcpName="TCP_grasp")

#定义获取图片位置
joint_get_img = np.array([5.342, 1.389, -85.62, 0.907, -89.436, 180.585])
hsrobot.move_j(joint_get_img,15,sTcpName="TCP_grasp")

# ----------------------------- 获取机器人状态-------------------------------
# 读取实际关节位置变量
r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
r_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
print(f'机器人当前各关节位置(in degree): { [r_joint_pos[0], r_joint_pos[1], r_joint_pos[2], r_joint_pos[3], r_joint_pos[4], r_joint_pos[5]]}\n')
# 读取实际笛卡尔空间位置变量
r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')

# ---------------------------------Vision----------------------------------
#末端期望位姿: [-773.37, -152.216, 240.261, -170.025, 4.34, 94.751]
# [    -726.18     -171.36      235.54      179.42     0.13332      87.218]
# [    -721.68     -171.71      234.94     -178.66     -2.8654      75.704]
# [    -727.65     -172.42      235.82      179.31    -0.74764      83.941]

realsenseD435i = RealsenseD435i()
# cam_grasp_point = realsenseD435i.vision_module_output()
# print(f'相机坐标系下四个角点的齐次坐标为: {cam_grasp_point}')
# [d_tcp_pos,d_tcp_ori] = hsrobot.get_TCP_targetPose(cam_grasp_point)
[d_tcp_pos,d_tcp_ori] = hsrobot.get_TCP_targetPose2()#采用二维码识别
d_tcp_pose = np.concatenate((np.array(d_tcp_pos),np.array(d_tcp_ori)))
print(f'末端期望位姿: {d_tcp_pose}\n')

# if d_tcp_pose[3] > 0:
#     d_tcp_pose[3] = - d_tcp_pose[3]
f_tcp_pose = np.array([-27, 24, -47, 7, 0, 18])
# d_tcp_pose = d_tcp_pose + f_tcp_pose
print(f'末端期望位姿: {d_tcp_pose}\n')

# # ----------------------------- 机器人运动到期望位姿--------------------------
# 定义关节目标空间初始位置
i_joint_pos = np.array([-5.07, -22.428, -145.167, -3.661, 31.682, 179.395])
hsrobot.move_j(i_joint_pos,15,sTcpName="TCP_grasp")

d_tcp_pose_1 = d_tcp_pose.copy() # 创建一个副本，并不是直接引用, 如果直接等于就是引用，指向同一个数组对象
d_tcp_pose_1[2] = 370
d_tcp_pose_1[0:3] = d_tcp_pose_1[0:3] + np.array([0, 100, 0])
print('期望位姿: ',d_tcp_pose_1)
hsrobot.move_l(d_tcp_pose_1,15,sTcpName="TCP_grasp")

d_tcp_pose_2 = d_tcp_pose_1.copy()
d_tcp_pose_2[0:3] = d_tcp_pose_2[0:3] + np.array([0, -100, 0])
print('期望位姿: ',d_tcp_pose_2)
hsrobot.move_l(d_tcp_pose_2,15,sTcpName="TCP_grasp")

d_tcp_pose_3 = d_tcp_pose_2.copy()
d_tcp_pose_3[0:3] = d_tcp_pose_3[0:3] + np.array([0, 0, 0])
print('期望位姿: ',d_tcp_pose_3)
hsrobot.move_l(d_tcp_pose_3,15,sTcpName="TCP_grasp")

d_tcp_pose_4 = d_tcp_pose_3.copy()
d_tcp_pose_4[2] = d_tcp_pose[2] + 15
hsrobot.move_l(d_tcp_pose_4,30,sTcpName="TCP_grasp")

# ----------------------------- 开启力控-------------------------------
nState = 1

nRet = hsrobot.arm.HRIF_SetForceControlState(0, 0, nState)
try:
    # 设置力控坐标系状态
    result1 = []
    nRet = hsrobot.arm.HRIF_SetForceToolCoordinateMotion(0,0,nState,result1)
    dWrench = [0, 1, 0, 0, 0, 0]
    # 设置力控自由度状态
    nRet = hsrobot.arm.HRIF_SetControlFreedom(0,0,dWrench)
    # nRet = hsrobot.arm.HRIF_SetForceControlStrategy(0,0,nState)
    # print(nRet)
    # 设置开启力控自由驱动
    bEnable = True
    # 设置开启力控自由驱动
    # nRet = hsrobot.arm.HRIF_SetForceFreeDriveMode(0,0,bEnable)
    result = [] 
    nRet = hsrobot.arm.HRIF_ReadForceControlState(0, 0, result)
    nState = int(result[0])
    print(nState)
except Exception as e:
    print(e) 
finally: 
    input('Press Enter to continue...')
    hsrobot.arm.HRIF_SetForceControlState(0,0,0)
time.sleep(1)

# ----------------------------- 机器人运动到期望位姿--------------------------
# 读取实际关节位置变量
r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
# 读取实际笛卡尔空间位置变量,向上运动取出电连接器
r_tcp_pos = np.array([float(i) for i in r_poselist[6:12]])

d_tcp_pose_5 = r_tcp_pos.reshape(6) + np.array([0,0,30,0,0,0])
print('期望位姿: ',d_tcp_pose_5)
hsrobot.move_l(d_tcp_pose_5,15,sTcpName="TCP_grasp")


# ----------------------------- plan-------------------------------
if input('是否执行plan? y/n') == 'y':
    plan = True
else:
    plan = False
if plan:
    config = {
    'SPEED': 20,
    'step1' : np.array([-965.528, 123.739, 294.749, 0.0, 28, 90.0]),#伸进去
    'step2' : np.array([-965.529, 48.623, 294.747, -0.0, 28.0, 90.0]),#向左移动298.424
    'step3' : np.array([-975.529, 52.623, 310.747, -0.0, 15.186, 90.0]),#向前上移动
    'step4' : np.array([-975.973, 10.065, 310.751, 0.0, 15.186, 90.0]),#向左移动
    'step5' : np.array([-975.973, 10.062, 322.753, 0.0, 2.632, 90.0]),#绕tcp-yaw旋转，z向上移动
    # 'step6' : np.array([-988.044, -136.767, 338.246, -180.0, -37.824, 90.0]),#绕tcp-yaw旋转，z向上移动，向左移动
    }
    ready_tcp_pos = np.array([-729.235, 123.737, 294.749, 0, 0, 90])
    # hsrobot.move_j(init_joint_pos,15,sTcpName="TCP_grasp")
    hsrobot.move_l(ready_tcp_pos,35,sTcpName="TCP_grasp")

   
    #正着走
    for i in range(6):
        print(f'step{i+1}')
        hsrobot.move_l(config[f'step{i+1}'],config['SPEED'],sTcpName="TCP_5")
        time.sleep(1.5)

    #倒着走
    for i in range(6):
        print(f'step{i+1}')
        hsrobot.move_l(config[f'step{6-i}'],config['SPEED'],sTcpName="TCP_5")
        time.sleep(1.5)

    #回到初始位置
    hsrobot.move_l(ready_tcp_pos,15,sTcpName="TCP_grasp")


# --------------------------------Control---------------------------------
'''
开启力控模式，确保以下步骤已执行：
(1) 负载辨识，示教器上运行<配置><设置TCP><负载辨识>, 机械臂开始自动校和，结束后点击<应用>保存生效
(2) 力传感器标定，标定是进行传感器的初始力偏差（安装预紧力、零漂等）、传感器与工具的质量和质心、传感器安装角度等参数的标定
示教器<配置><力传感器><标定>设置八个标定点位:(0,0,90,0,90,0),(0,0,90,0,60,0),(0,0,90,0,120,0),(0,0,90,30,90,0),(0,0,90,30,60,0),(0,0,90,-30,90,0),
(0,0,90,-30,120,0),(0,0,90,-30,60,0)，点击<应用>保存生效
'''
# 设置力传感器状态 0：关闭力控 1：开启力控
# hsrobot.arm.HRIF_SetForceControlState(0,0,1)
# 定义力控状态码到描述的映射  
# force_control_states = {  
#     0: "未开启力控",  
#     1: "力控探寻中",  
#     2: "力控探寻完成,保持恒力控制",  
#     3: "力控自由驱动"  
# }  
# # 读取标定后力传感器数据
# result = []; hsrobot.arm.HRIF_ReadFTCabData(0,0,result)
# r_force_data = np.array([float(i) for i in result[0:6]]) 
# print(f'当前六维力传感器数据为(in N/Nm):\nx方向力:{r_force_data[0]}\ny方向力:{r_force_data[1]}\nz方向力:{r_force_data[2]}\nRx方向力矩:{r_force_data[3]}\nRy方向力矩:{r_force_data[4]}\nRz方向力矩:{r_force_data[5]}\n')
# # 读取当前力控状态  
# result = []; hsrobot.arm.HRIF_ReadForceControlState(0, 0, result)  
# nState = int(result[0])  
# state_description = force_control_states.get(nState, "未知的力控状态")  
# print(f"力控处于: {state_description}")
# # 设置力控制策略 1:柔顺模式/0:恒力模式
# nRet = hsrobot.arm.HRIF_SetForceControlStrategy(0,0,0) # 恒力模式
# # 设置力控坐标系方向为  Tool  坐标方向模式，这里为TCP_grasp
# nRet = hsrobot.arm.HRIF_SetForceToolCoordinateMotion(0,0,1) 
# 设置力控限制范围，力传感器超过此范围后控制器断电
# #  设置外部力最大值 
# dMax = [500, 500, 500, 500, 500, 500]   
# #  设置外部力最小值 
# dMin = [-500, -500, -500, -500, -500, -500]   
# #  设置力传感器数据限制范围 
# nRet = hsrobot.arm.HRIF_SetForceDataLimit(0,0, dMax, dMin)

'''
设置力控参数
'''
# #  设置惯量控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
# Mass = [0, 0, 0, 0, 0, 0] 
# nRet = hsrobot.arm.HRIF_SetMassParams(0,0,Mass) 
# #  设置阻尼控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
# Damp = [800, 800, 800, 40, 40, 40] 
# nRet = hsrobot.arm.HRIF_SetDampParams(0,0,Damp) 
# #  设置刚度控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
# Stiff = [1000, 1000, 1000, 100, 100, 100] 
# nRet = hsrobot.arm.HRIF_SetStiffParams(0,0,Stiff) 
# #  设置恒力控制目标力/力矩: dX：X方向，单位[N], dY：Y方向，单位[N], dZ：Z方向，单位[N], dRx：Rx方向，单位[NM], dRy：Ry方向，单位[NM], dRz：Rz方向，单位[NM] 
# Goal = [10, 0, 0, 0, 0, 0] 
# #  设置恒力控制目标力/力矩，Assembly Strategy Based on ARIE，装配方向施加恒定力，即X方向设置恒力10N, 其余方向为放松状态，设置目标力/力矩为0
# nRet = hsrobot.arm.HRIF_SetForceControlGoal(0,0,Goal) 

'''
开启恒力控制模式，机器人将保持恒力控制状态，执行装配动作规划任务
基于环境吸引域理论的“两步法”装配策略: Assembly Strategy Based on ARIE
Step 1: 主动偏转轴的姿态，并构建ARIE
'''
# # get_current_pose
# r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
# # 读取实际笛卡尔空间位置变量
# r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
# r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
# # print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')
# r_toolPose = hsrobot.pose_robot(r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2],r_tcp_pos[0],r_tcp_pos[1],r_tcp_pos[2])
# # 绕自身的Z轴旋转10度
# deflect_toolPose = hsrobot.pose_robot(0, 0, 10, 0, 0, 0)
# d_toolPose = r_toolPose @ deflect_toolPose
# print(f'期望的末端位姿为: {d_toolPose}\n')

# d_tcp_ori = hsrobot.rotation_matrix_to_rpy(d_toolPose[0:3,0:3])
# d_tcp_pos = d_toolPose[0:3,3]
# d_tcp_pose = np.concatenate(np.array(d_tcp_pos),np.array(d_tcp_ori))
# hsrobot.move_l(d_tcp_pose,30,sTcpName="TCP_grasp")


# 暂停力控运动，仅暂停力控功能，不暂停运动和脚本
# nRet = hsrobot.arm.HRIF_ForceControlinterrupt(0,0)

'''
夹具控制
'''
# 夹具打开
# hsrobot.gripper_open()
# hsrobot.gripper_init()
# time.sleep(10)

# 夹具闭合
# hsrobot.gripper_close()
# hsrobot.gripper_init()
# time.sleep(6)   



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
