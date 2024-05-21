import time
import sys
import numpy as np
import math
from src.vision import RealsenseD435i
import spatialmath.base as spatialmathbase
from src.hsrobot import HSROBOT as hs_robot_arm
import pykin.utils.transform_utils as pykin_utils_transform
import threading
import matplotlib.pyplot as plt
from collections import deque
import pandas as pd
import scipy.signal as signal
import json

# 定义队列长度
queue_length = 100
# 创建两个双端队列来存储位置和姿态的数据
postion_data = [deque(maxlen=queue_length) for _ in range(3)] # 100队列长度
oritation_data = [deque(maxlen=queue_length) for _ in range(3)]
# 创建两个双端队列来存储力和力矩的数据
force_data = [deque(maxlen=queue_length) for _ in range(3)] # 100队列长度
torque_data = [deque(maxlen=queue_length) for _ in range(3)]
t = deque(maxlen=queue_length)  # 创建一个双端队列来保存时间


mapping_pos = {
    0: 'x',
    1: 'y',
    2: 'z',
}
mapping_ori = {
    0: 'Rx',
    1: 'Ry',
    2: 'Rz',
}

'''
清空队列
'''
def clear_deques():
    global postion_data, oritation_data, force_data, torque_data, t

    # 清空位置和姿态的数据队列
    for deque in postion_data:
        deque.clear()
    for deque in oritation_data:
        deque.clear()

    # 清空力和力矩的数据队列
    for deque in force_data:
        deque.clear()
    for deque in torque_data:
        deque.clear()

    # 清空时间队列
    t.clear()

'''
实时读取机器人状态数据 1000HZ 1ms
'''
def realtime_read_robot_data():

    # 设计低通滤波器
    filter_order = 2
    b, a = signal.butter(N=filter_order, Wn=0.01, btype='low')  # N是滤波器的阶数，Wn是截止频率，btype指定滤波器类型
    t0 = time.time()  # 获取当前时间

    global queue_length
    global force_data
    global torque_data
    global postion_data
    global oritation_data
    global t

    slice_length = 10 # 取多少时刻的数据
    force_data_array = np.zeros((slice_length,3))
    torque_data_array = np.zeros((slice_length,3))

    # 力传感器数据清零
    hsrobot.arm.HRIF_SetForceZero(0,0)


    while True:
        
        t1 = time.time()  # 获取当前时间

        # 读取六维接触力数据
        result = []; hsrobot.arm.HRIF_ReadFTCabData(0,0,result)
        r_force_data = np.array([float(i) for i in result[0:6]]) 

        # 读取实际笛卡尔空间位置变量
        r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
        r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
        r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
        # print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')

        for i in range(3):

            postion_data[i].append(r_tcp_pos[i])  # 保存xyz
            oritation_data[i].append(r_tcp_ori[i])  # 保存rxryrz
        
            force_data[i].append(r_force_data[i])  # 保存xyz方向的力
            torque_data[i].append(r_force_data[i+3])  # 保存rxryrz方向的力矩
            '''
            filter
            '''
            if len(force_data[i]) >= filter_order*5:
                # 对数据进行滤波
                force_data[i] = deque(signal.filtfilt(b, a, force_data[i]), maxlen=queue_length)
                force_data_array[:,i] = np.array(list(force_data[i])[-slice_length:])
            if len(torque_data[i]) >= filter_order*5:
                # 对数据进行滤波  
                torque_data[i] = deque(signal.filtfilt(b, a, torque_data[i]), maxlen=queue_length)
                torque_data_array[:,i] = np.array(list(torque_data[i])[-slice_length:])

        t.append(time.time() - t0)  # 更新时间

        # time.sleep(0.1)

        # print(f'采集一次数据用时: {(time.time()-t1)*1000} ms')
        # print(f'X方向受到外部力: {force_data_array[:,0]}\n')
        # print(f'Y方向受到外部力: {force_data_array[:,1]}\n')
        # print(f'Z方向受到外部力: {force_data_array[:,2]}\n')
        # print(f'Rx方向受到外部力矩: {torque_data_array[:,0]}\n')
        # print(f'Ry方向受到外部力矩: {torque_data_array[:,1]}\n')
        # print(f'Rz方向受到外部力矩: {torque_data_array[:,2]}\n')
        
'''
读取机器人力传感数据并保存
'''
def read_and_save_force_data():

    global keyboard_interrupt
    t0 = time.time()  # 获取当前时间

    hsrobot.arm.HRIF_SetForceZero(0,0)

    # 打开文件以写入数据
    with open('force_data.json', 'w') as f:

        while True:

            # 读取六维接触力数据
            result = []; hsrobot.arm.HRIF_ReadFTCabData(0,0,result)
            r_force_data = np.array([float(i) for i in result[0:6]]) 

            # 获取力和力矩数据
            force__data_save = r_force_data[:3].tolist()
            torque_data_save = r_force_data[3:].tolist()

            # 获取当前时间
            current_time = time.time() - t0

            # 将数据和时间保存到一个字典中
            data = {
                'force_data': force__data_save,
                'torque_data': torque_data_save,
                'time': current_time,
            }

            # 将字典写入到JSON文件中
            json.dump(data, f)
            f.write('\n')  # 添加一个新行，使每个数据条目占据一行

            # 检查停止条件
            if keyboard_interrupt:
                break

            # 等待一段时间，以减少数据的采集频率
            time.sleep(0.1)

'''
读取机器人位姿数据并保存
'''
def read_and_save_pose_data():

    global keyboard_interrupt
    t0 = time.time()  # 获取当前时间

    # 打开文件以写入数据
    with open('pose_data.json', 'w') as f:

        while True:

            # 读取机器人位姿数据
            r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
            r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
            r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])

            # 获取当前时间
            current_time = time.time() - t0

            # 将数据和时间保存到一个字典中
            data = {
                'pos_data': r_tcp_pos.tolist(),
                'ori_data': r_tcp_ori.tolist(),
                'time': current_time,
            }

            # 将字典写入到JSON文件中
            json.dump(data, f)
            f.write('\n')  # 添加一个新行，使每个数据条目占据一行

            # 检查停止条件
            if keyboard_interrupt:
                break

            # 等待一段时间，以减少数据的采集频率
            time.sleep(0.1)

'''
绘制机器人状态数据
'''
def realtime_plot_robot_data():

    global force_data
    global torque_data
    global postion_data
    global oritation_data
    global mapping_pos
    global mapping_ori
    global t

    plt.ion()  # 开启交互模式

    # 创建第一个窗口，显示位姿数据
    fig1, axs = plt.subplots(4, figsize=(8, 9))
    # # 创建第二个窗口，显示力数据
    # fig2, axs2 = plt.subplots(2)

    label = ['x', 'y', 'z', 'Rx', 'Ry', 'Rz']  # 定义图例标签
    colors = ['r', 'g', 'b']  # 定义颜色


    while True:

        if not plt.fignum_exists(fig1.number):
            break  # 如果窗口已经关闭，退出循环
        
        axs[0].clear(); axs[1].clear(); axs[2].clear(); axs[3].clear()  # 清除图形

        axs[0].tick_params(axis='both', labelsize=6); axs[1].tick_params(axis='both', labelsize=6)  # 设置x轴和y轴的刻度标签字体大小为14
        axs[2].tick_params(axis='both', labelsize=6); axs[3].tick_params(axis='both', labelsize=6)  # 设置x轴和y轴的刻度标签字体大小为14
        # 设置x轴和y轴的标题
        axs[0].set_xlabel('t/s',  fontsize=10); axs[0].set_ylabel('F/N',  fontsize=10, rotation=0)
        axs[1].set_xlabel('t/s',  fontsize=10); axs[1].set_ylabel('T/Nm',  fontsize=10, rotation=0)
        axs[2].set_xlabel('t/s',  fontsize=10); axs[2].set_ylabel('P/mm',  fontsize=10, rotation=0)
        axs[3].set_xlabel('t/s',  fontsize=10); axs[3].set_ylabel('$\\theta/\\circ$',  fontsize=10, rotation=0)
        axs[0].yaxis.set_label_coords(-0.1,0.5)
        axs[1].yaxis.set_label_coords(-0.1,0.5)
        axs[2].yaxis.set_label_coords(-0.1,0.5)
        axs[3].yaxis.set_label_coords(-0.1,0.5)

        for i in range(3): 

            axs[0].plot(t, force_data[i],label=label[i], linewidth=1.2, color=colors[i])  # 绘制新的图形
            axs[1].plot(t, torque_data[i],label=label[i+3], linewidth=1.2, color=colors[i])  # 绘制新的图形
            axs[2].plot(t, postion_data[i],label=label[i], linewidth=1.2, color=colors[i])  # 绘制新的图形
            axs[3].plot(t, oritation_data[i],label=label[i+3], linewidth=1.2, color=colors[i])  # 绘制新的图形
            # 在图形上显示最后一个t对应的各个数值
            axs[0].text(0.97, 0.4 - i*0.15, f'{mapping_pos[i]}: {force_data[i][-1]:.3f}', verticalalignment='bottom', horizontalalignment='left', transform=axs[0].transAxes, color='black', fontsize=8)
            axs[1].text(0.97, 0.4 - i*0.15, f'{mapping_ori[i]}: {torque_data[i][-1]:.3f}', verticalalignment='bottom', horizontalalignment='left', transform=axs[1].transAxes, color='black', fontsize=8)
            axs[2].text(0.97, 0.4 - i*0.15, f'{mapping_pos[i]}: {postion_data[i][-1]:.3f}', verticalalignment='bottom', horizontalalignment='left', transform=axs[2].transAxes, color='black', fontsize=8)
            axs[3].text(0.97, 0.4 - i*0.15, f'{mapping_ori[i]}: {oritation_data[i][-1]:.3f}', verticalalignment='bottom', horizontalalignment='left', transform=axs[3].transAxes, color='black', fontsize=8)

        axs[0].legend(bbox_to_anchor=(0.97, 1), loc='upper left', fontsize=8)  # 显示图例，并设置位置
        axs[1].legend(bbox_to_anchor=(0.97, 1), loc='upper left', fontsize=8)  # 显示图例，并设置位置
        axs[2].legend(bbox_to_anchor=(0.97, 1), loc='upper left', fontsize=8)  # 显示图例，并设置位置
        axs[3].legend(bbox_to_anchor=(0.97, 1), loc='upper left', fontsize=8)  # 显示图例，并设置位置

        plt.draw()  # 更新图形
        plt.pause(0.001)  # 暂停0.01秒，在暂停代码执行的同时，更新图形并处理图形的事件，每次花费0.1s，100队列就是10s
        # print(f'{time.time()-time1}/s') # 统计代码执行时长
        
    plt.show()  # 阻塞程序，直到图形窗口被关闭
    # 循环结束后，关闭所有窗口
    plt.close('all')

'''
绘制力传感数据
'''
def plot_force_data():

    global mapping_pos
    global mapping_ori
    global start_time
    force_data_save = []
    torque_data_save = []
    time_save = []

    # 读取JSON文件
    with open('force_data.json', 'r') as f:
        data = [json.loads(line) for line in f]

    # 提取力数据和时间
    for item in data:

        if np.any(np.abs(np.array([item_ for item_ in item['force_data']])) > 50) or np.any(np.abs(np.array([item_ for item_ in item['torque_data']])) > 50):
            continue

        if len(item['force_data']) < 3 or len(item['torque_data']) < 3 or item['time'] <= start_time: # 去掉没有被记录的数据 和 开始的3s数据
            continue
        else:
            force_data_save.append(item['force_data'])
            torque_data_save.append(item['torque_data'])
            time_save.append(item['time']-start_time)


    # 创建3个子图
    fig, axs = plt.subplots(6)
    fig.suptitle('Realtime force Data')
    color = ['r','g','b']

    # 在每个子图上绘制一个方向的力
    for i in range(3):
        axs[i].plot(time_save, [item[i] for item in force_data_save],color=color[i],linewidth=1.2)
        axs[i+3].plot(time_save, [item[i] for item in torque_data_save],color=color[i],linewidth=1.2)
        axs[i].set_ylabel(f'Force {mapping_pos[i]}')
        axs[i+3].set_ylabel(f'Torque {mapping_ori[i]}')

    # 设置x轴标签
    axs[-1].set_xlabel('Time')

    # plt.show()

'''
绘制位姿数据
'''
def plot_pose_data():

    global mapping_pos
    global mapping_ori
    global start_time
    position_data_save = []
    orientation_data_save = []
    time_save = []

    # 读取JSON文件
    with open('pose_data.json', 'r') as f:
        data = [json.loads(line) for line in f]

    # 提取位姿数据和时间
    for item in data:

        if len(item['pos_data']) < 3 or len(item['ori_data']) < 3 or item['time'] <= start_time: # 去掉没有被记录的数据 和 开始的3s数据
            continue
        else:

            if np.abs(item['ori_data'][0]) <= 180.0 and np.abs(item['ori_data'][0]) >= 179.9:
                item['ori_data'][0] = 180.00

            position_data_save.append(item['pos_data'])
            orientation_data_save.append(item['ori_data'])
            time_save.append(item['time']-start_time)

    # 创建6个子图

    fig, axs = plt.subplots(6)
    fig.suptitle('Realtime Pose Data')
    color = ['r','g','b']

    # 在每个子图上绘制一个方向的位姿
    for i in range(3):
        axs[i].plot(time_save, [item[i] for item in position_data_save],color=color[i],linewidth=1.2)
        axs[i+3].plot(time_save, [item[i] for item in orientation_data_save],color=color[i],linewidth=1.2)
        axs[i].set_ylabel(f'Position {mapping_pos[i]}')
        axs[i+3].set_ylabel(f'Orientation {mapping_ori[i]}')

    # 设置x轴标签
    axs[-1].set_xlabel('Time')

    # plt.show()

'''
检测键盘按键输入
'''
def check_keyboard():

    global keyboard_interrupt
    input("按下键回车停止程序")
    hsrobot.arm.HRIF_SetForceControlState(0,0,0)
    time.sleep(0.2)
    keyboard_interrupt = True

'''
等待力探寻开启完成
'''
def waitForceDone():
    while True:
        nRet = hsrobot.arm.HRIF_ReadForceControlState()
        print("waitForceDone HRIF_ReadForceControlState->%s" % nRet)
        if float(nRet[1]) == 2:
            time.sleep(0.2)
            break
        time.sleep(0.2)

'''
等待力控关闭完成
'''
def wait_Force_OFF_Done():
    while True:
        nRet = hsrobot.arm.HRIF_ReadForceControlState()
        print(nRet)
        if float(nRet[1]) == 0:
            time.sleep(0.2)
            break
        time.sleep(0.2)


'''
导纳控制器
'''
def admittance_controller():

    global start_time

    '''
    motion limitation parameters
    '''
    #  设置最大关节速度 
    nRet = hsrobot.arm.HRIF_SetJointMaxVel(0,0,[45,45,75,75,90,90]) # default/2 deg/s
    #  设置最大关节加速度 
    nRet = hsrobot.arm.HRIF_SetJointMaxAcc(0,0,[180,180,180,180,180,180]) # default/2 deg/s^2
    #  设置最大直线速度 
    nRet = hsrobot.arm.HRIF_SetLinearMaxVel(0,0,500) # mm/s
    #  设置最大直线加速度 
    nRet = hsrobot.arm.HRIF_SetLinearMaxAcc(0,0,2500) # mm/s^2

    '''
    admittance control parameters
    '''
    # 设置力控工具坐标系方向模式
    result = []; nRet = hsrobot.arm.HRIF_SetForceToolCoordinateMotion(0,0,1,result) 
    # 设置惯量控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    Mass = [20, 20, 20, 1, 1, 1] # [10, 10, 10, 4, 4, 4]
    nRet = hsrobot.arm.HRIF_SetMassParams(0,0,Mass) 
    #  设置阻尼控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    Damp = [300, 300, 300, 0.5, 0.5, 0.5]
    nRet = hsrobot.arm.HRIF_SetDampParams(0,0,Damp) 
    #  设置刚度控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    Stiff = [60, 60, 60, 0.1, 0.1, 0.1] 
    nRet = hsrobot.arm.HRIF_SetStiffParams(0,0,Stiff) 
    # 设置力控限制范围，力传感器超过此范围后控制器断电
    dMax = [50, 50, 50, 50, 50, 50] # 设置外部力最大值 
    dMin = [-50, -50, -50, -50, -50, -50] # 设置外部力最小值 
    nRet = hsrobot.arm.HRIF_SetForceDataLimit(0,0, dMax, dMin) # 设置力传感器数据限制范围
    # 设置力控模式 1:柔顺模式/0:恒力模式
    nRet = hsrobot.arm.HRIF_SetForceControlStrategy(0,0,1) # 恒力模式
    #  设置恒力控制目标力/力矩: dX：X方向，单位[N], dY：Y方向，单位[N], dZ：Z方向，单位[N], dRx：Rx方向，单位[NM], dRy：Ry方向，单位[NM], dRz：Rz方向，单位[NM] 
    Goal = [0, 0, 0, 0, 0, 0] 
    nRet = hsrobot.arm.HRIF_SetForceControlGoal(0,0,Goal) 
    # 设置力控自由度, 0: 关闭 1: 开启
    freedom = [1, 1, 1, 1, 1, 1]
    nRet = hsrobot.arm.HRIF_SetControlFreedom(0,0,freedom)

    '''
    纠正位姿
    '''
    r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    d_tcp_ori = [180, 0, -90] # 纠正机器人姿态，先对齐
    d_tcp_pos = [-720, 18, 551] # 保持当前位置不变
    d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori)))
    hsrobot.move_l(d_tcp_pose,10,sTcpName="TCP_grasp")

    start_time = time.time() - start_time
    t1 = time.time()

   
    #=================================== motion planning ( Assembly Strategy Based on ARIE )===================================#
    

    '''
    Step 1: 主动偏转轴的姿态 Rx=0 Ry=60 Rz=0, 构建吸引域
    '''
    
    r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    r_toolPose = hsrobot.pose_robot(r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2],r_tcp_pos[0],r_tcp_pos[1],r_tcp_pos[2]) # print(r_toolPose)
    deflect_toolPose = hsrobot.pose_robot(0, 50, 0, 0, 0, 0) # print(deflect_toolPose)
    d_tcp_poseMatrix = r_toolPose @ deflect_toolPose # print(f'期望的末端位姿为: {d_tcp_poseMatrix}\n')
    d_tcp_ori = np.zeros(3)
    d_tcp_ori[0],d_tcp_ori[1],d_tcp_ori[2] = np.rad2deg(hsrobot.rotation_matrix_to_rpy(d_tcp_poseMatrix[0:3, 0],d_tcp_poseMatrix[0:3, 1],d_tcp_poseMatrix[0:3, 2]))
    d_tcp_pos = d_tcp_poseMatrix[0:3,3]
    d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori))) # print(f'期望的末端位姿为: {d_tcp_pose}\n')
    hsrobot.move_l(d_tcp_pose,10,sTcpName="TCP_grasp")
    print(f'step1 done, 完成时间: {time.time()-t1}\n')


    '''
    Step 2: 沿着+X方向前进, 保持Y、Z方向放松
    '''

    ## 开启力控

    hsrobot.arm.HRIF_SetMassParams(0,0,[13, 13, 13, 1, 1, 1]) # 设置惯量控制参数 
    hsrobot.arm.HRIF_SetDampParams(0,0,[141, 141, 141, 0.5, 0.5, 0.5]) # 设置阻尼控制参数
    hsrobot.arm.HRIF_SetStiffParams(0,0,[51, 51, 51, 0.1, 0.1, 0.1]) # 设置刚度控制参数 *
    hsrobot.arm.HRIF_SetForceZero(0,0)
    Goal = [0, 0, 0, 0, 0, 0] 
    nRet = hsrobot.arm.HRIF_SetForceControlGoal(0,0,Goal) 
    freedom = [0, 1, 1, 0, 0, 0]
    nRet = hsrobot.arm.HRIF_SetControlFreedom(0,0,freedom)
    nRet = hsrobot.arm.HRIF_SetForceControlStrategy(0,0,1)
    hsrobot.arm.HRIF_SetForceControlState(0,0,1)
    time.sleep(0.2)


    ## 到达吸引域最低点

    tcp_feed_dis = np.array([50,0,0]) # 末端执行器的偏移距离
    r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    rotMatrix = pykin_utils_transform.get_matrix_from_rpy(r_tcp_ori/180*np.pi)
    base_feed_dis = np.dot(rotMatrix, tcp_feed_dis)
    # print(f'base_feed_dis: {base_feed_dis}')
    d_tcp_pos = r_tcp_pos + base_feed_dis
    d_tcp_ori = r_tcp_ori
    d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori)))
    hsrobot.move_l_timeblock(d_tcp_pose,5,sTcpName="TCP_grasp")
    time.sleep(4)
    print(f'step2 done, 完成时间: {time.time()-t1}\n')
    

    '''
    Step 3: 主动偏转Ry姿态, 消除姿态误差
    '''

    ## 切换力控

    hsrobot.arm.HRIF_SetForceControlState(0,0,0) # 先关力控
    time.sleep(2)
    hsrobot.arm.HRIF_SetMassParams(0,0,[13, 13, 13, 1, 1, 1]) # 设置惯量控制参数 
    hsrobot.arm.HRIF_SetDampParams(0,0,[141, 141, 141, 0.8, 0.8, 0.8]) # 设置阻尼控制参数
    hsrobot.arm.HRIF_SetStiffParams(0,0,[51, 51, 51, 0.1, 0.1, 0.1]) # 设置刚度控制参数 *
    hsrobot.arm.HRIF_SetForceControlGoal(0,0,[0, 0, 0, 0, 0, 0]) 
    hsrobot.arm.HRIF_SetControlFreedom(0,0,[0, 0, 0, 0, 1, 0])
    hsrobot.arm.HRIF_SetForceControlStrategy(0,0,1)
    hsrobot.arm.HRIF_SetForceControlState(0,0,1) # 再开力控，避免振动

    ## 偏转回来Ry姿态

    r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    r_toolPose = hsrobot.pose_robot(r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2],r_tcp_pos[0],r_tcp_pos[1],r_tcp_pos[2]) # print(r_toolPose)
    transMatrix_top2tcp = hsrobot.pose_robot(0, 0, 0, 52, 0, 0) # print(transMatrix_top2tcp)
    transMatrix_top2tcp2 = hsrobot.pose_robot(0, 0, 0, -52, 0, 0)
    deflect_toolPose = hsrobot.pose_robot(0, -20, 0, 0, 0, 0) # print(deflect_toolPose)
    d_tcp_poseMatrix = r_toolPose @ transMatrix_top2tcp @ deflect_toolPose @ transMatrix_top2tcp2 # print(f'期望的末端位姿为: {d_tcp_poseMatrix}\n')
    d_tcp_ori = np.zeros(3)
    d_tcp_ori[0],d_tcp_ori[1],d_tcp_ori[2] = np.rad2deg(hsrobot.rotation_matrix_to_rpy(d_tcp_poseMatrix[0:3, 0],d_tcp_poseMatrix[0:3, 1],d_tcp_poseMatrix[0:3, 2]))
    d_tcp_pos = d_tcp_poseMatrix[0:3,3]
    d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori))) # print(f'期望的末端位姿为: {d_tcp_pose}\n')
    hsrobot.move_l_timeblock(d_tcp_pose,5,sTcpName="TCP_grasp")
    time.sleep(4)
    print(f'Step3 done, 完成时间: {time.time()-t1}\n')


    '''
    Step 4: 继续施加作用力, 完成装配
    '''
    ## 切换力控

    hsrobot.arm.HRIF_SetForceControlState(0,0,0) # 先关力控
    time.sleep(2)
    hsrobot.arm.HRIF_SetMassParams(0,0,[30, 13, 13, 1, 1, 1]) # 设置惯量控制参数 
    hsrobot.arm.HRIF_SetDampParams(0,0,[141, 141, 141, 0.8, 0.8, 0.8]) # 设置阻尼控制参数
    hsrobot.arm.HRIF_SetStiffParams(0,0,[51, 51, 51, 0.1, 0.1, 0.1]) # 设置刚度控制参数 *
    hsrobot.arm.HRIF_SetForceControlGoal(0,0,[10, 0, 0, 0, 0, 0]) 
    hsrobot.arm.HRIF_SetControlFreedom(0,0,[1, 0, 0, 0, 0, 0])
    hsrobot.arm.HRIF_SetForceControlStrategy(0,0,1)
    hsrobot.arm.HRIF_SetForceControlState(0,0,1) # 再开力控，避免振动

    ## 前进插入

    tcp_feed_dis = np.array([60,0,0]) # 末端执行器的偏移距离
    r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    rotMatrix = pykin_utils_transform.get_matrix_from_rpy(r_tcp_ori/180*np.pi)
    base_feed_dis = np.dot(rotMatrix, tcp_feed_dis)
    # print(f'base_feed_dis: {base_feed_dis}')
    d_tcp_pos = r_tcp_pos + base_feed_dis
    d_tcp_ori = r_tcp_ori
    d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori)))
    hsrobot.move_l_timeblock(d_tcp_pose,5,sTcpName="TCP_grasp")
    time.sleep(4)
    print(f'step4 done, 完成时间: {time.time()-t1}\n')
    

    '''
    关闭力控
    '''
    hsrobot.arm.HRIF_SetForceControlState(0,0,0)
    global keyboard_interrupt
    keyboard_interrupt = True





'''
测试运动
'''
def test_motion():

    # get_current_pose
    r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    # 读取实际笛卡尔空间位置变量
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    # print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')
    d_tcp_ori = [180,0,-90] # 纠正机器人姿态，先对齐
    d_tcp_pos = r_tcp_pos.copy() # 保持当前位置不变
    d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori)))
    # print(f'期望的末端位姿为: {d_tcp_pose}\n')
    hsrobot.move_l(d_tcp_pose,5,sTcpName="TCP_grasp")


'''
实时读取并绘制机器人的状态(Tcp位姿和外部接触力)
'''
def realtime_read_and_plot_robot_data():

    read_robotData_thread = threading.Thread(target=realtime_read_robot_data)
    read_robotData_thread.start()
    realtime_plot_robot_data() # 读取机器人本体状态数据并进行绘制



'''
执行力控
'''
def force_control():

    global keyboard_interrupt

    force_control_thread = threading.Thread(target=admittance_controller)
    force_control_thread.start()
    time.sleep(1)

    read_and_save_force_data_thread = threading.Thread(target=read_and_save_force_data)
    read_and_save_force_data_thread.start()
    time.sleep(0.2)

    read_and_save_pose_data_thread = threading.Thread(target=read_and_save_pose_data)
    read_and_save_pose_data_thread.start()
    time.sleep(0.2)


    check_keyboard_thread = threading.Thread(target=check_keyboard)
    check_keyboard_thread.start()
    time.sleep(0.2)


    while not keyboard_interrupt:
        time.sleep(0.1)

    time.sleep(5)
    plot_force_data() # 绘制力传感数据
    plot_pose_data() # 绘制位姿数据
    plt.show()

    # read_robotData_thread = threading.Thread(target=realtime_read_robot_data ,daemon=True) 
    # read_robotData_thread.start()
    # time.sleep(1)


    # robot_motion_thread = threading.Thread(target=test_motion)
    # robot_motion_thread.start()
    # time.sleep(1)

    # realtime_plot_robot_data() # 读取机器人本体状态数据并进行绘制



if __name__ == '__main__' :

    # -------------------------------连接机器人--------------------------------
    hsrobot = hs_robot_arm()

    # --------------------------------Control---------------------------------
    '''
    开启力控模式，确保以下步骤已执行：
    (1) 负载辨识，示教器上运行<配置><设置TCP><负载辨识>, 机械臂开始自动校和，结束后点击<应用>保存生效
    (2) 力传感器标定，标定是进行传感器的初始力偏差（安装预紧力、零漂等）、传感器与工具的质量和质心、传感器安装角度等参数的标定
    示教器<配置><力传感器><标定>设置15个标定点位:（标定点位均保存于点位列表, 命名为calib_forcesensor_1-15, 按照顺序长按到达）
    点击<应用>保存生效
    '''
    keyboard_interrupt = False
    start_time = time.time()

    force_control()

    # clear_deques()
    # read_and_plot_sensor_data()
    # test_motion()
    # admittance_controller()

    # plot_force_data() # 绘制力传感数据



