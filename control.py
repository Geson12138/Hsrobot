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

# 定义队列长度
queue_length = 100
# 创建两个双端队列来存储位置和姿态的数据
postion_data = [deque(maxlen=queue_length) for _ in range(3)] # 100队列长度
oritation_data = [deque(maxlen=queue_length) for _ in range(3)]
# 创建两个双端队列来存储力和力矩的数据
force_data = [deque(maxlen=queue_length) for _ in range(3)] # 100队列长度
torque_data = [deque(maxlen=queue_length) for _ in range(3)]
t = deque(maxlen=queue_length)  # 创建一个双端队列来保存时间
keyboard_interrupt = False

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

slice_length = 10 # 取多少时刻的数据
force_data_array = np.zeros((slice_length,3))
torque_data_array = np.zeros((slice_length,3))

'''
读取机器人状态数据 1000HZ 1ms
'''
def read_force_data():

    # 设计低通滤波器
    filter_order = 2
    b, a = signal.butter(N=filter_order, Wn=0.01, btype='low')  # N是滤波器的阶数，Wn是截止频率，btype指定滤波器类型
    t0 = time.time()  # 获取当前时间

    global queue_length
    global force_data
    global torque_data
    global t
    global force_data_array
    global torque_data_array


    # 力传感器数据清零
    hsrobot.arm.HRIF_SetForceZero(0,0)


    while True:
        
        t1 = time.time()  # 获取当前时间

        # 读取六维接触力数据
        result = []; hsrobot.arm.HRIF_ReadFTCabData(0,0,result)
        r_force_data = np.array([float(i) for i in result[0:6]]) 

        for i in range(3):
        
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

        # print(force_data_array)

        # time.sleep(0.1)

        # print(f'采集一次数据用时: {(time.time()-t1)*1000} ms')
        # print(f'X方向受到外部力: {force_data_array[:,0]}\n')
        # print(f'Y方向受到外部力: {force_data_array[:,1]}\n')
        # print(f'Z方向受到外部力: {force_data_array[:,2]}\n')
        # print(f'Rx方向受到外部力矩: {torque_data_array[:,0]}\n')
        # print(f'Ry方向受到外部力矩: {torque_data_array[:,1]}\n')
        # print(f'Rz方向受到外部力矩: {torque_data_array[:,2]}\n')
        

def plot_force_data():

    global force_data
    global torque_data
    global mapping_pos
    global mapping_ori
    global t

    plt.ion()  # 开启交互模式

    # 创建第一个窗口，显示位姿数据
    fig1, axs = plt.subplots(2, figsize=(8, 9))

    label = ['x_r', 'y', 'z', 'Rx', 'Ry', 'Rz']  # 定义图例标签
    colors = ['r', 'g', 'b']  # 定义颜色


    while True:

        if not plt.fignum_exists(fig1.number):
            break  # 如果窗口已经关闭，退出循环
        
        axs[0].clear(); axs[1].clear()

        axs[0].tick_params(axis='both', labelsize=6); axs[1].tick_params(axis='both', labelsize=6)  # 设置x轴和y轴的刻度标签字体大小为14
        # 设置x轴和y轴的标题
        axs[0].set_xlabel('t/s',  fontsize=10); axs[0].set_ylabel('F/N',  fontsize=10, rotation=0)
        axs[1].set_xlabel('t/s',  fontsize=10); axs[1].set_ylabel('T/Nm',  fontsize=10, rotation=0)
        axs[0].yaxis.set_label_coords(-0.1,0.5)
        axs[1].yaxis.set_label_coords(-0.1,0.5)

        for i in range(3): 

            axs[0].plot(t, force_data[i],label=label[i], linewidth=1.2, color=colors[i])  # 绘制新的图形
            axs[1].plot(t, torque_data[i],label=label[i+3], linewidth=1.2, color=colors[i])  # 绘制新的图形
            # 在图形上显示最后一个t对应的各个数值
            axs[0].text(0.97, 0.4 - i*0.15, f'{mapping_pos[i]}: {force_data[i][-1]:.3f}', verticalalignment='bottom', horizontalalignment='left', transform=axs[0].transAxes, color='black', fontsize=8)
            axs[1].text(0.97, 0.4 - i*0.15, f'{mapping_ori[i]}: {torque_data[i][-1]:.3f}', verticalalignment='bottom', horizontalalignment='left', transform=axs[1].transAxes, color='black', fontsize=8)

        axs[0].legend(bbox_to_anchor=(0.97, 1), loc='upper left', fontsize=8)  # 显示图例，并设置位置
        axs[1].legend(bbox_to_anchor=(0.97, 1), loc='upper left', fontsize=8)  # 显示图例，并设置位置

        plt.draw()  # 更新图形
        plt.pause(0.001)  # 暂停0.01秒，在暂停代码执行的同时，更新图形并处理图形的事件，每次花费0.1s，100队列就是10s
        # print(f'{time.time()-time1}/s') # 统计代码执行时长
        
    plt.show()  # 阻塞程序，直到图形窗口被关闭
    # 循环结束后，关闭所有窗口
    plt.close('all')


def plot_pose_data():

    global postion_data
    global oritation_data
    global mapping_pos
    global mapping_ori
    global t

    plt.ion()  # 开启交互模式

    # 创建第一个窗口，显示位姿数据
    fig2, axs = plt.subplots(2, figsize=(8, 9))

    label = ['x', 'y', 'z', 'Rx', 'Ry', 'Rz']  # 定义图例标签
    colors = ['r', 'g', 'b']  # 定义颜色


    while True:

        if not plt.fignum_exists(fig2.number):
            break  # 如果窗口已经关闭，退出循环
        
        axs[0].clear(); axs[1].clear() # 清除图形

        axs[0].tick_params(axis='both', labelsize=6); axs[1].tick_params(axis='both', labelsize=6)  # 设置x轴和y轴的刻度标签字体大小为14
        # 设置x轴和y轴的标题
        axs[0].set_xlabel('t/s',  fontsize=10); axs[0].set_ylabel('F/N',  fontsize=10, rotation=0)
        axs[1].set_xlabel('t/s',  fontsize=10); axs[1].set_ylabel('T/Nm',  fontsize=10, rotation=0)
        axs[0].yaxis.set_label_coords(-0.1,0.5)
        axs[1].yaxis.set_label_coords(-0.1,0.5)

        for i in range(3): 

            axs[0].plot(t, postion_data[i],label=label[i], linewidth=1.2, color=colors[i])  # 绘制新的图形
            axs[1].plot(t, oritation_data[i],label=label[i+3], linewidth=1.2, color=colors[i])  # 绘制新的图形
            # 在图形上显示最后一个t对应的各个数值
            axs[0].text(0.97, 0.4 - i*0.15, f'{mapping_pos[i]}: {postion_data[i][-1]:.3f}', verticalalignment='bottom', horizontalalignment='left', transform=axs[0].transAxes, color='black', fontsize=8)
            axs[1].text(0.97, 0.4 - i*0.15, f'{mapping_ori[i]}: {oritation_data[i][-1]:.3f}', verticalalignment='bottom', horizontalalignment='left', transform=axs[1].transAxes, color='black', fontsize=8)

        axs[0].legend(bbox_to_anchor=(0.97, 1), loc='upper left', fontsize=8)  # 显示图例，并设置位置
        axs[1].legend(bbox_to_anchor=(0.97, 1), loc='upper left', fontsize=8)  # 显示图例，并设置位置

        plt.draw()  # 更新图形
        plt.pause(0.001)  # 暂停0.01秒，在暂停代码执行的同时，更新图形并处理图形的事件，每次花费0.1s，100队列就是10s
        # print(f'{time.time()-time1}/s') # 统计代码执行时长
        
    plt.show()  # 阻塞程序，直到图形窗口被关闭
    # 循环结束后，关闭所有窗口
    plt.close('all')

'''
绘制机器人状态数据
'''
def plot_robot_data():

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
检测键盘按键输入
'''
def check_keyboard():
    global keyboard_interrupt
    input("按下键回车停止程序")
    hsrobot.arm.HRIF_SetForceControlState(0,0,0)
    time.sleep(0.2)
    keyboard_interrupt = True


'''
导纳控制器
'''
def force_controller():

    global force_data
    global torque_data
    global keyboard_interrupt

    '''
    motion limitation parameters
    '''

    #  设置最大关节速度 
    nRet = hsrobot.arm.HRIF_SetJointMaxVel(0,0,[30,30,50,50,60,60]) # default/3 deg/s
    #  设置最大关节加速度 
    nRet = hsrobot.arm.HRIF_SetJointMaxAcc(0,0,[120,120,120,120,120,120]) # default/3 deg/s^2

    #  设置最大直线速度 
    nRet = hsrobot.arm.HRIF_SetLinearMaxVel(0,0,500) # mm/s
    #  设置最大直线加速度 
    nRet = hsrobot.arm.HRIF_SetLinearMaxAcc(0,0,1250) # mm/s^2

    '''
    force control parameters
    '''

    # 设置力控工具坐标系方向模式
    result = []; nRet = hsrobot.arm.HRIF_SetForceToolCoordinateMotion(0,0,1,result) 

    # 期望惯性系数对角矩阵: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    # Mass = [40, 40, 40, 2, 2, 2] # [10, 10, 10, 4, 4, 4]
    Mass = [10,10,10]
    massMatrix = np.diag(Mass)

    # 期望阻尼系数对角矩阵: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    # Damp = [30, 30, 30, 1.5, 1.5, 1.5]
    Damp = [30, 30, 30]
    dampingMatrix = np.diag(Damp)

    # 期望刚度系数对角矩阵: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    # Stiff = [10, 10, 10, 0.5, 0.5, 0.5] 
    Stiff = [10, 10, 10]
    stiffnessMatrix = np.diag(Stiff)

    # compute error to desired equilibrium pose
    pos_error = np.zeros((3,1))

    print("力控参数设置完毕")

    '''
    motion planning
    '''

    # get_current_pose
    r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])

    # x0 = np.concatenate((r_tcp_pos, r_tcp_ori)).transpose() # desired equalibrium pose
    # x_d = np.zeros((6,1))
    # x_dd = np.zeros((6,1))

    x0 = r_tcp_pos.transpose()/1000 # desired equalibrium position
    # print(f'期望位置: {x0}\n')
    x_d = np.zeros((3,1))
    x_dd = np.zeros((3,1))

    T = 0.001 # 1ms
    x_last = x0

    # 设计低通滤波器
    filter_order = 2
    b, a = signal.butter(N=filter_order, Wn=0.01, btype='low')  # N是滤波器的阶数，Wn是截止频率，btype指定滤波器类型
    t0 = time.time()  # 获取当前时间

    while True:

        t1 = time.time()  # 获取当前时间
        if keyboard_interrupt:
            break

        # 读取六维接触力数据
        result = []; hsrobot.arm.HRIF_ReadFTCabData(0,0,result)
        r_force_data = np.array([float(i) for i in result[0:6]]) 

        for i in range(3):
        
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



        r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
        r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
        r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])


        rotMatrix = pykin_utils_transform.get_matrix_from_rpy(r_tcp_ori/180*np.pi)
        # x_r = np.concatenate((r_tcp_pos, r_tcp_ori)).transpose() # current pose
        x_r = r_tcp_pos.transpose()/1000 # current position
        # print(f'当前位置: {x_r}\n')

        x_dot = (x_r - x_last) / T # velocity 
        # print(f'当前速度: {x_dot}\n')
        pos_error = x_r - x0 # 当前值减去期望值
        # print(f'位置误差: {pos_error}\n')

        # get external force
        # F_ext = np.concatenate(force_data_array[-1,:],torque_data_array[-1,:]).transpose()
        # print(force_data_array)
        f_ext = force_data_array[-1,:].transpose()
        print(f'外部力: {f_ext}\n')
        f_base = rotMatrix @ f_ext

        x_dd = np.linalg.inv(massMatrix) @ ( f_base - dampingMatrix @ x_dot - stiffnessMatrix @ pos_error )# force control
        print(f'期望加速度: {x_dd}\n')
        x_d = x_dot + T * x_dd
        print(f'期望速度: {x_d}\n')
        x_r = x_r + T * x_d
        print(f'期望位置: {x_r}\n')

        d_tcp_pos = x_r.transpose()*1000
        d_tcp_ori = r_tcp_ori
        print(f'计算位置: {d_tcp_pos}\n')


        d_tcp_pose = np.concatenate((d_tcp_pos, d_tcp_ori))
        # move to the new position
        hsrobot.move_l(d_tcp_pose,10,sTcpName="TCP_grasp")

        for i in range(3):
            postion_data[i].append(d_tcp_pos[i])  # 保存xyz
            oritation_data[i].append(d_tcp_ori[i])  # 保存rxryrz
    
        x_last = x_r
        T = time.time() - t1
        t.append(time.time() - t0)  # 更新时间

        print(f'采集一次数据用时: {(time.time()-t1)*1000} ms')

        # time.sleep(T) # 1ms



'''
测试运动
'''
def test_motion():

    # get_current_pose
    r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    # print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')
    d_tcp_ori = [180,0,-90] # 纠正机器人姿态，先对齐
    d_tcp_pos = r_tcp_pos.copy() # 保持当前位置不变
    d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori)))
    # print(f'期望的末端位姿为: {d_tcp_pose}\n')
    hsrobot.move_l(d_tcp_pose,5,sTcpName="TCP_grasp")


'''
读取并绘制机器人的状态(Tcp位姿和外部接触力)
'''
def read_and_plot_force_data():

    read_robotData_thread = threading.Thread(target=read_force_data)
    read_robotData_thread.start()
    plot_force_data() # 读取机器人本体状态数据并进行绘制


'''
执行力控
'''
def force_control():

    # force_control_thread = threading.Thread(target=admittance_controller)
    # force_control_thread.start()
    # time.sleep(1)

    # check_keyboard_thread = threading.Thread(target=check_keyboard) #,daemon=True
    # check_keyboard_thread.start()
    # time.sleep(0.2)

    # read_robotData_thread = threading.Thread(target=read_force_data ,daemon=True) 
    # read_robotData_thread.start()
    # time.sleep(1)

    # force_control_thread = threading.Thread(target=force_controller ,daemon=True)
    # force_control_thread.start()
    # time.sleep(1)

    # robot_motion_thread = threading.Thread(target=test_motion)
    # robot_motion_thread.start()
    time.sleep(1)
    # plot_robot_data()




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

    clear_deques()
    # read_and_plot_sensor_data()

    # force_control()
    # read_and_plot_force_data()
    # test_motion()
    # admittance_controller()

    force_controller()



