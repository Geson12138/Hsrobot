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
读取机器人状态数据 1000HZ 1ms
'''
def read_robot_data():

    # 设计低通滤波器
    filter_order = 2
    b, a = signal.butter(N=filter_order, Wn=0.02, btype='low')  # N是滤波器的阶数，Wn是截止频率，btype指定滤波器类型
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

        # print(f'采集一次数据用时: {(time.time()-t1)*1000} ms')
        # print(f'X方向受到外部力: {force_data_array[:,0]}\n')
        # print(f'Y方向受到外部力: {force_data_array[:,1]}\n')
        # print(f'Z方向受到外部力: {force_data_array[:,2]}\n')
        # print(f'Rx方向受到外部力矩: {torque_data_array[:,0]}\n')
        # print(f'Ry方向受到外部力矩: {torque_data_array[:,1]}\n')
        # print(f'Rz方向受到外部力矩: {torque_data_array[:,2]}\n')
        

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

    global force_data
    global torque_data
    global postion_data
    global oritation_data

    '''
    admittance control parameters
    '''

    # 设置力控工具坐标系方向模式
    result = []; nRet = hsrobot.arm.HRIF_SetForceToolCoordinateMotion(0,0,1,result) 
    # 设置惯量控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    Mass = [10, 10, 10, 4, 4, 4] # [10, 10, 10, 4, 4, 4]
    nRet = hsrobot.arm.HRIF_SetMassParams(0,0,Mass) 

    #  设置阻尼控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    Damp = [200, 200, 200, 20, 20, 20]
    # nRet = hsrobot.arm.HRIF_SetDampParams(0,0,Damp) 

    #  设置刚度控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    Stiff = [100, 100, 100, 100, 100, 100] 
    nRet = hsrobot.arm.HRIF_SetStiffParams(0,0,Stiff) 

    # 设置力控限制范围，力传感器超过此范围后控制器断电
    dMax = [50, 50, 50, 50, 50, 50] # 设置外部力最大值 
    dMin = [-50, -50, -50, -50, -50, -50] # 设置外部力最小值 
    nRet = hsrobot.arm.HRIF_SetForceDataLimit(0,0, dMax, dMin) # 设置力传感器数据限制范围

    # 设置力控模式 1:柔顺模式/0:恒力模式
    nRet = hsrobot.arm.HRIF_SetForceControlStrategy(0,0,0) # 恒力模式

    #  设置恒力控制目标力/力矩: dX：X方向，单位[N], dY：Y方向，单位[N], dZ：Z方向，单位[N], dRx：Rx方向，单位[NM], dRy：Ry方向，单位[NM], dRz：Rz方向，单位[NM] 
    Goal = [0, 0, 0, 0, 0, 0] 
    nRet = hsrobot.arm.HRIF_SetForceControlGoal(0,0,Goal) 

    # 设置力控自由度, 0: 关闭 1: 开启
    freedom = [1, 1, 1, 0, 0, 0]
    nRet = hsrobot.arm.HRIF_SetControlFreedom(0,0,freedom)

    # 力传感器数据清零
    set_force_zero_res = hsrobot.arm.HRIF_SetForceZero(0,0)

    # 开启力控 0：关闭力控 1：开启力控
    hsrobot.arm.HRIF_SetForceControlState(0,0,1)

    # waitForceDone()
    print("恒力控制设置完毕")


'''
读取并绘制机器人的状态(Tcp位姿和外部接触力)
'''
def read_and_plot_robot_data():

    read_robotData_thread = threading.Thread(target=read_robot_data)
    read_robotData_thread.start()
    plot_robot_data() # 读取机器人本体状态数据并进行绘制


'''
执行力控
'''
def force_control():

    force_control_thread = threading.Thread(target=admittance_controller)
    force_control_thread.start()
    time.sleep(1)

    read_robotData_thread = threading.Thread(target=read_robot_data,daemon=True)
    read_robotData_thread.start()
    time.sleep(1)

    plot_robot_data() # 读取机器人本体状态数据并进行绘制

    # 开启力控 0：关闭力控 1：开启力控
    hsrobot.arm.HRIF_SetForceControlState(0,0,0)
    print('关闭力控')


if __name__ == '__main__' :

    # -------------------------------连接机器人--------------------------------
    hsrobot = hs_robot_arm()
    # nRet = hsrobot.arm.HRIF_GrpEnable(0,0) # 机器人使能
    # if nRet == 0:
    #     print('机器人使能成功')
    # else:
    #     print('机器人使能失败')
    # time.sleep(3.5)

    '''
    prefix description: i_ for initial; r_ for real; d_ for desired; f_ for fixed
    '''
    # --------------------------------Control---------------------------------
    '''
    开启力控模式，确保以下步骤已执行：
    (1) 负载辨识，示教器上运行<配置><设置TCP><负载辨识>, 机械臂开始自动校和，结束后点击<应用>保存生效
    (2) 力传感器标定，标定是进行传感器的初始力偏差（安装预紧力、零漂等）、传感器与工具的质量和质心、传感器安装角度等参数的标定
    示教器<配置><力传感器><标定>设置14个标定点位:（以下标定点位均保存于点位列表，长按到达）
    (0,-10,-90,0,-90,0),(0,-10,-90,0,-60,0),(0,-10,-90,0,-110,0),
    (0,-10,-90,30,-90,0),(0,-10,-90,30,-60,0),(0,-10,-90,-30,-90,0),
    (0,-10,-90,-30,-110,0),(0,-10,-90,-30,-60,0),(0,-10,-90,0,-30,0),
    (0,-10,-90,0,0,0),(0,-10,-90,0,30,0),(0,-10,-90,30,-30,0),
    (0,-10,-90,30,-0,0),(0,-10,-90,30,30,0)
    点击<应用>保存生效
    '''
    # # # 设置力传感器状态 0：关闭力控 1：开启力控
    # hsrobot.arm.HRIF_SetForceControlState(0,0,1)
    # # # 定义力控状态码到描述的映射  
    # force_control_states = {  
    #     0: "未开启力控",  
    #     1: "力控探寻中",  
    #     2: "力控探寻完成,保持恒力控制",  
    #     3: "力控自由驱动"
    # }  


    clear_deques()
    # read_and_plot_sensor_data()
    # read_and_plot_robot_pose()
    force_control()


    '''
    力控功能设置
    '''
    # 读取力探寻状态状态  
    # result = []; hsrobot.arm.HRIF_ReadForceControlState(0, 0, result)  
    # nState = int(result[0])  
    # state_description = force_control_states.get(nState, "未知的力控状态")  
    # print(f"力控处于: {state_description}")
    # 设置力控坐标系方向为 Tool 坐标方向模式，这里为TCP_grasp



    # result = []; nRet = hsrobot.arm.HRIF_SetForceToolCoordinateMotion(0,0,1,result) 
    # # 设置力控限制范围，力传感器超过此范围后控制器断电
    # dMax = [50, 50, 50, 50, 50, 50] # 设置外部力最大值 
    # dMin = [-50, -50, -50, -50, -50, -50] # 设置外部力最小值 
    # nRet = hsrobot.arm.HRIF_SetForceDataLimit(0,0, dMax, dMin) # 设置力传感器数据限制范围

    '''
    力控参数设置
    '''

    # #  设置惯量控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    # Mass = [1, 1, 1, 1, 1, 1] 
    # nRet = hsrobot.arm.HRIF_SetMassParams(0,0,Mass) 
    # #  设置阻尼控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    # Damp = [400, 400, 400, 400, 400, 400] 
    # nRet = hsrobot.arm.HRIF_SetDampParams(0,0,Damp) 
    # #  设置刚度控制参数: dX：X方向，dY：Y方向，dZ：Z方向，dRx：Rx方向，dRy：Ry方向，dRz：Rz方向
    # Stiff = [500, 500, 500, 500, 500, 500] 
    # nRet = hsrobot.arm.HRIF_SetStiffParams(0,0,Stiff) 
    # #  设置恒力控制目标力/力矩: dX：X方向，单位[N], dY：Y方向，单位[N], dZ：Z方向，单位[N], dRx：Rx方向，单位[NM], dRy：Ry方向，单位[NM], dRz：Rz方向，单位[NM] 
    # Goal = [1, 1, 1, 1, 1, 1] 
    # #  设置恒力控制目标力/力矩，Assembly Strategy Based on ARIE，装配方向施加恒定力，即X方向设置恒力10N, 其余方向为放松状态，设置目标力/力矩为0
    # nRet = hsrobot.arm.HRIF_SetForceControlGoal(0,0,Goal) 


    '''
    恒力控制参数

    模式：恒力控制
    设置TCP: TCP or TCP_grasp
    坐标系特征: 选择感受接触力的坐标系 
    目标力: [X,Y,Z,RX,RY,RZ] = [0,0,0,0,0,0]
    PID: [FP,FI,FD,TP,TI,TD] = [1.5,0.3,0.5,0.5,0.2,0.1]
    质量: [MX,MY,Z,MRX,MRY,MRZ] = [35,35,35,5,5,5]
    阻尼: [DX,DY,DZ,DRX,DRY,DRZ] = [700,700,700,20,20,20]
    刚度: [KX,KY,KZ,KRX,KRY,KRZ] = [10,10,10,1,1,1]
    速度: 直线速度: 5mm/s; 角速度: 1度/s
    '''
    # *	@index : 
    # *	@param brief:开启关闭力传感器_脚本带配置
    # *	@param boxID:电箱ID
    # *	@param rbtID:机器人ID,一般为0
    # *	@param state : 设置力传感器状态
    # *	@param FTMode : 控制模式
    #                     0 : 恒力模式
    #                     1 : 柔顺模式
    # *	@param UCS : Tool
    # *	@param vel : vel[0]线速度
    #                     vel[1]角度速度
    # *	@param forces : 目标探寻力x、y、z、Rx、Ry、Rz
    # *	@param freedom : 力控探寻自由度X,Y,Z,Rx,Ry,Rz
    # *	@param PID : fP,fI,fD,tP,tI,tD
    # *	@param Mass : 惯量控制参数
    # *	@param Damp : 阻尼控制参数
    # *	@param Stiff : 刚度参数x、y、z、Rx、Ry、Rz
    # *	@param return: 是否开启力控成功(0成功，1失败)

    '''
    恒力控制参数

    模式：恒力控制
    设置TCP: TCP or TCP_grasp
    坐标系特征: 选择感受接触力的坐标系 
    目标力: [X,Y,Z,RX,RY,RZ] = [0,0,0,0,0,0]
    PID: [FP,FI,FD,TP,TI,TD] = [1.5,0.3,0.5,0.5,0.2,0.1]
    质量: [MX,MY,Z,MRX,MRY,MRZ] = [35,35,35,5,5,5]
    阻尼: [DX,DY,DZ,DRX,DRY,DRZ] = [700,700,700,20,20,20]
    刚度: [KX,KY,KZ,KRX,KRY,KRZ] = [10,10,10,1,1,1]
    速度: 直线速度: 5mm/s; 角速度: 1度/s
    '''

    # Mass = [8,8,8,5,5,5] # [50,50,50,10,10,10] # 设置惯量控制参数
    # Damp = [700,700,700,20,20,20] #[2000,2000,2000, 40, 40, 40] # 设置阻尼控制参数
    # Stiff = [100,100,100,1,1,1] #[10,10,10,100,100,100] # 设置刚度控制参数

    # vel = [5, 1] # 设置线速度和角速度
    # Tool = 'Tool' # 设置TCP
    # forces = [0,0,0,0,0,0] # 设置目标探寻力
    # freedom = [1,1,1,0,0,0] # 力控探寻自由度X,Y,Z,Rx,Ry,Rz
    # PID = [1,0.1,0,1,0.1,0] # PID参数
    # hsrobot.arm.HRIF_SetScriptForceControlState(0,0,1,0,Tool,vel,forces,freedom,PID,Mass,Damp,Stiff)

    # input('Press Enter to continue...')
    # hsrobot.arm.HRIF_SetForceControlState(0,0,0)



    '''
    开启恒力控制模式后，机器人将保持恒力控制状态，执行装配动作规划任务
    '''
    # 设置力控模式 1:柔顺模式/0:恒力模式
    # nRet = hsrobot.arm.HRIF_SetForceControlStrategy(0,0,0) # 恒力模式

    # # get_current_pose
    # r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    # # 读取实际笛卡尔空间位置变量
    # r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    # r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    # print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')
    # d_tcp_ori = [180,0,90] # 纠正机器人姿态，先对齐
    # d_tcp_pos = r_tcp_pos.copy() # 保持当前位置不变
    # d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori)))
    # print(f'期望的末端位姿为: {d_tcp_pose}\n')
    # hsrobot.move_l(d_tcp_pose,20,sTcpName="TCP_grasp")

    # temp_rotMatrix = pykin_utils_transform.get_matrix_from_rpy(r_tcp_ori/180*np.pi)
    # print(temp_rotMatrix)
    # temp_rpy = np.rad2deg(pykin_utils_transform.get_rpy_from_matrix(temp_rotMatrix))
    # print(temp_rpy)

    '''
    基于环境吸引域理论的装配策略: Assembly Strategy Based on ARIE
    Step 1: 主动偏转轴的姿态, 并构建ARIE
    '''
    # # get_current_pose
    # r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
    # # 读取实际笛卡尔空间位置变量
    # r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    # r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    # r_toolPose = hsrobot.pose_robot(r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2],r_tcp_pos[0],r_tcp_pos[1],r_tcp_pos[2]) # print(r_toolPose)
    # # 绕自身的Z轴旋转5度
    # deflect_toolPose = hsrobot.pose_robot(0, 0, 5, 0, 0, 0) # print(deflect_toolPose)
    # d_tcp_poseMatrix = r_toolPose @ deflect_toolPose # print(f'期望的末端位姿为: {d_tcp_poseMatrix}\n')
    # d_tcp_ori = np.zeros(3)
    # d_tcp_ori[0],d_tcp_ori[1],d_tcp_ori[2] = np.rad2deg(hsrobot.rotation_matrix_to_rpy(d_tcp_poseMatrix[0:3, 0],d_tcp_poseMatrix[0:3, 1],d_tcp_poseMatrix[0:3, 2]))
    # d_tcp_pos = d_tcp_poseMatrix[0:3,3]
    # d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori))) # print(f'期望的末端位姿为: {d_tcp_pose}\n')
    # hsrobot.move_l(d_tcp_pose,20,sTcpName="TCP_grasp")

    '''
    Step 2: 设计运动轨迹(沿着插接方向)到达吸引域最低点, 在X方向上施加主动力, Y、Z方向上放松, Rx、Ry、Rz方向上施加力矩
    '''
    # d_tcp_pos = d_tcp_pos + np.array([0,50,50]) # yz方向上移动50mm，斜45度向上
    # d_tcp_pose = np.concatenate((np.array(d_tcp_pos), np.array(d_tcp_ori)))
    # hsrobot.move_l(d_tcp_pose,10,sTcpName="TCP_grasp")

    # 暂停力控运动，仅暂停力控功能，不暂停运动和脚本
    # nRet = hsrobot.arm.HRIF_ForceControlinterrupt(0,0)