from src.hsrobot import HSROBOT as hs_robot_arm
import numpy as np
import time

config = {
    'SPEED': 20,
    'step1' : np.array([-965.528, 123.739, 294.749, 0.0, 28, 90.0]),#伸进去
    'step2' : np.array([-965.529, 48.623, 294.747, -0.0, 28.0, 90.0]),#向左移动298.424
    'step3' : np.array([-975.529, 52.623, 310.747, -0.0, 15.186, 90.0]),#向前上移动
    'step4' : np.array([-975.973, 10.065, 310.751, 0.0, 15.186, 90.0]),#向左移动
    'step5' : np.array([-975.973, 10.062, 322.753, 0.0, 2.632, 90.0]),#绕tcp-yaw旋转，z向上移动
    # 'step6' : np.array([-988.044, -136.767, 338.246, -180.0, -37.824, 90.0]),#绕tcp-yaw旋转，z向上移动，向左移动
}

# -------------------------------连接机器人--------------------------------
hsrobot = hs_robot_arm()
hsrobot.arm.HRIF_GrpEnable(0,0) # 机器人使能
time.sleep(3.5)

#======================运动到初始位置==============================
# init_joint_pos = np.array([-10.956,17.118,-136.711,-7.59,67.931,38.165])
init_tcp_pos = np.array([-729.235, 123.737, 294.749, 0, 0, 90])
# hsrobot.move_j(init_joint_pos,15,sTcpName="TCP_grasp")
hsrobot.move_l(init_tcp_pos,15,sTcpName="TCP_grasp")

# 读取实际关节位置变量
r_poselist = []; hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)
r_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
print(f'机器人当前各关节位置(in degree): { [r_joint_pos[0], r_joint_pos[1], r_joint_pos[2], r_joint_pos[3], r_joint_pos[4], r_joint_pos[5]]}\n')

# 读取实际笛卡尔空间位置变量
r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')

test_bias = np.array([0,0,0,0,0,0])

#正着走
for i in range(5):
    print(f'step{i+1}')
    hsrobot.move_l(config[f'step{i+1}']+test_bias,config['SPEED'],sTcpName="TCP_grasp")
    time.sleep(1.5)

#倒着走
for i in range(5):
    print(f'step{i+1}')
    hsrobot.move_l(config[f'step{5-i}']+test_bias,config['SPEED'],sTcpName="TCP_grasp")
    time.sleep(1.5)

# #回到初始位置
hsrobot.move_l(init_tcp_pos,15,sTcpName="TCP_grasp")