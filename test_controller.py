import time
import numpy as np
import math
from src.vision import RealsenseD435i
import spatialmath.base as spatialmathbase
from src.hsrobot import HSROBOT as hs_robot_arm
import pykin.utils.transform_utils as pykin_utils_transform
import sys
import threading

hsrobot = hs_robot_arm()

def waitForceDone():
    while True:
        result = [] 
        nRet = hsrobot.arm.HRIF_ReadForceControlState(0, 0, result)
        print("waitForceDone HRIF_ReadForceControlState->%s" % nRet)
        if float(nRet) == 2:
            time.sleep(0.2)
            break
        time.sleep(0.2)

def setup_force():
    # 设置力控工具坐标
    result = []
    print(hsrobot.arm.HRIF_SetForceToolCoordinateMotion(0,0,1,result))
    mass = [10, 10, 10, 4, 4, 4]
    # 设置mass
    res_demo = hsrobot.arm.HRIF_SetMassParams(0,0,mass)
    print("cps.HRIF_SetMassParams = %s" % res_demo)

    # 设置探寻距离200mm,上下共200mm
    print(hsrobot.arm.HRIF_SetForceDistanceLimit(0,0,100, 2))
    # 设置切向力(X,Y方向力)大于20N时,机器人向Z方向抬起手臂,直到切向力小于Min(10N)后,力控恢复正常探寻
    print(hsrobot.arm.HRIF_SetTangentForceBounds(0,0,25, 10, 50))
    # 设置避障模式
    print(hsrobot.arm.HRIF_SetForceControlStrategy(0,0,2))
    # 探寻自由度：Z轴
    print(hsrobot.arm.HRIF_SetControlFreedom(0,0,[0, 1, 0, 0, 0, 0]))
    # 探寻力Z方向：10
    print(hsrobot.arm.HRIF_SetForceControlGoal(0,0,[0, 0, 10, 0, 0, 0]))
    # 设置力控探寻的最大速度
    print(hsrobot.arm.HRIF_SetMaxSearchVelocities(0,0,20, 10))
    # 力控清零
    print(hsrobot.arm.HRIF_SetForceZero(0,0))
    time.sleep(1)
    set_force_zero_res = hsrobot.arm.HRIF_SetForceZero(0,0)
    print("cps.HRIF_SetForceZero() %s" % set_force_zero_res)
    if set_force_zero_res != 0:
        sys.exit()

    # 力传感器开启
    print(hsrobot.arm.HRIF_SetForceControlState(0,0,1))
    waitForceDone()
    print("探寻力设置完毕")

keep_running = True 
def listen_for_input():
    global keep_running
    input("按下回车键停止程序")
    hsrobot.arm.HRIF_SetForceControlState(0,0,0)
    # hsrobot.arm.HRIF_ForceControlInterrupt(0,0)
    keep_running = False

# nState = 1

# nRet = hsrobot.arm.HRIF_SetForceControlState(0, 0, nState)
# try:
#     #定义mass大小
#     # Mass = [80, 80, 80, 80, 80, 80]
#     # nRet = hsrobot.arm.HRIF_SetMassParams(0,0,Mass)
#     # 定义力控自由度状态
#     # 设置力控坐标系状态
#     result1 = []
#     nRet = hsrobot.arm.HRIF_SetForceToolCoordinateMotion(0,0,nState,result1)
#     dWrench = [0, 1, 0, 0, 0, 0]
#     # 设置力控自由度状态
#     nRet = hsrobot.arm.HRIF_SetControlFreedom(0,0,dWrench)
#     # nRet = hsrobot.arm.HRIF_SetForceControlStrategy(0,0,nState)
#     # print(nRet)
#     # 设置开启力控自由驱动
#     bEnable = True
#     # 设置开启力控自由驱动
#     # nRet = hsrobot.arm.HRIF_SetForceFreeDriveMode(0,0,bEnable)
#     result = [] 
#     nRet = hsrobot.arm.HRIF_ReadForceControlState(0, 0, result)
#     nState = int(result[0])
#     print(nState)
# except Exception as e:
#     print(e) 
# finally: 
#     input('Press Enter to continue...')
#     hsrobot.arm.HRIF_SetForceControlState(0,0,0)


r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
r_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
print(f'机器人当前各关节位置(in degree): { [r_joint_pos[0], r_joint_pos[1], r_joint_pos[2], r_joint_pos[3], r_joint_pos[4], r_joint_pos[5]]}\n')
# 读取实际笛卡尔空间位置变量
r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')

threading.Thread(target=listen_for_input).start()

setup_force()
#向下移动10mm
# r_tcp_pos[2] -= 200
# tcp = np.vstack((r_tcp_pos, r_tcp_ori)).reshape(6)
# print(tcp)
# hsrobot.move_l(tcp, 20, sTcpName="TCP_grasp")
