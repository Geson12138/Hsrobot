import pandas as pd
import numpy as np
import cv2
import time
import numpy as np
import math
import spatialmath.base as spatialmathbase
from src.hsrobot import HSROBOT as hs_robot_arm
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
import roboticstoolbox as rtb

bridge = CvBridge() 
rospy.init_node('get_images', anonymous=True)
# 定义一个ros话题，把self.color_image发布出去
image_pub = rospy.Publisher('image_topic', Image, queue_size=10)

def get_image_frome_ros(i):
        
    time_count=0
    while True:
        color_img = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=None)
        depth_img = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=None)
        time_count = time_count +1
        if time_count>= 10:
            break

    color_image = bridge.imgmsg_to_cv2(color_img, 'bgr8')
    cv2.imwrite(f'./calib/calib_image_{i}.jpg',color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])

# -------------------------------连接机器人--------------------------------
hsrobot = hs_robot_arm()

# ------------------------------定义标定位姿--------------------------------
i_tcp_pose = np.array([-454, -50 , 540, 170, 0, 153]) # 初始标定位姿
x_plus = 40
y_plus = 40
z_plus = 100
theta_x_plus = 5
theta_y_plus = 8
theta_z_plus = 8
# 定义关节目标空间位置
tcp_pose = {
    0 : [0, 0, 0, 0, 0, 0], # unchanged

    1 : [0, 0, 0, theta_x_plus, 0, 0], # +rx
    2 : [x_plus, y_plus, 0, 0, 0, 0], # +x+y

    3 : [0, 0, 0, -theta_x_plus, 0, 0], # -rx
    4 : [-x_plus, -y_plus, 0, 0, 0, 0], # -x-y

    5 : [0, 0, 0, 0, theta_y_plus, 0], # +ry
    6 : [x_plus, -y_plus, 0, 0, 0, 0], # +x-y

    7 : [0, 0, 0, 0, -theta_y_plus, 0], # -ry
    8 : [-x_plus, y_plus, 0, 0, 0, 0], # -x+y

    9 : [0, 0, 0, 0, 0, theta_z_plus], # +rz
    10 : [0, 0, 0, 0, 0, -theta_z_plus], # -rz

    11 : [x_plus/2, 0, 0, theta_x_plus, theta_y_plus, 0], # +rx+ry
    12 : [-x_plus/2, 0, 0, -theta_x_plus, -theta_y_plus, 0], # -rx-ry
    13 : [0, y_plus/2, 0, -theta_x_plus, theta_y_plus, 0], # -rx+ry
    14 : [0, -y_plus/2, 0, theta_x_plus, -theta_y_plus, 0], # +rx-ry

    15 : [0, 0, -z_plus, theta_x_plus, 0, theta_z_plus], # +rx+rz
    16 : [0, 0, 0, -theta_x_plus, 0, -theta_z_plus], # -rx-rz
    17 : [0, 0, -z_plus, -theta_x_plus, 0, theta_z_plus], # -rx+rz
    18 : [0, 0, 0, theta_x_plus, 0, -theta_z_plus], # +rx-rz

    19 : [0, 0, -z_plus, 0, theta_y_plus, theta_z_plus], # +ry+rz
    20 : [0, 0, 0, 0, -theta_y_plus, -theta_z_plus], # -ry-rz
    21 : [0, 0, -z_plus, 0, -theta_y_plus, theta_z_plus], # -ry+rz
    22 : [0, 0, 0, 0, theta_y_plus, -theta_z_plus], # +ry-rz


}
count = 23
pose_files = []
for i in list(range(count)):
    print(f'当前位姿编号: {i}')
    # ----------------------------- 机器人运动到标定位姿--------------------------
    d_tcp_pose = i_tcp_pose + tcp_pose[i]
    print(f'末端期望位姿: {d_tcp_pose}\n')
    hsrobot.arm.HRIF_SetOverride(0,0,0.3) # 设置速度
    hsrobot.move_l(d_tcp_pose,30)
    time.sleep(1)

    # ----------------------------- 获取视觉图像 --------------------------
    get_image_frome_ros(i)

    r_poselist = [] # 定义返回值空列表
    hsrobot.arm.HRIF_ReadActPos(0,0, r_poselist)

    # 读取实际笛卡尔空间位置变量
    r_tcp_pos = np.array([float(i) for i in r_poselist[6:9]])
    r_tcp_ori = np.array([float(i) for i in r_poselist[9:12]])
    # print(f'机器人当前TCP位姿为(in mm/degree): { [r_tcp_pos[0], r_tcp_pos[1], r_tcp_pos[2],r_tcp_ori[0],r_tcp_ori[1],r_tcp_ori[2]]}\n')

    c_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
    # -------------------------------------利用机器人工具箱和DH参数建立运动学模型-----------------------------
    rtb_hsrobot = rtb.DHRobot([rtb.RevoluteDH(d=0.26,alpha=np.pi/2),
                        rtb.RevoluteDH(a=0.48,alpha=np.pi),
                        rtb.RevoluteDH(alpha=np.pi/2),
                        rtb.RevoluteDH(d=0.52, alpha=-np.pi/2),
                        rtb.RevoluteDH(alpha=np.pi/2),
                        rtb.RevoluteDH(d=0.192)
                        ], name="hsrobot")
    q_init = np.array([0,np.pi/2,np.pi/2,0,0,0])
    # q_cur = c_joint_pos[0:5]/180*np.pi # 机器人前五个关节位置.
    q_cur = c_joint_pos/180*np.pi # 机器人前五个关节位置
    q = np.add(q_init,q_cur)
    temp_T = rtb_hsrobot.fkine(q) #运用机器人工具箱得到第五个关节的位姿矩阵
    temp_r = temp_T.A[:3,:3];pos = temp_T.A[:3,3]*1000

    euler_angles = spatialmathbase.tr2rpy(temp_r)
    ori = np.array([np.rad2deg(j) for j in euler_angles]) 

    pose = np.concatenate((r_tcp_ori,r_tcp_pos.T))

    print(f'工具箱pos: {pos}, 直接读pos: {r_tcp_pos}')
    print(f'工具箱ori: {ori}, 直接读ori: {r_tcp_ori}')

    pose_files.append(pose)
    # print(f'当前位姿: {pose}\n')

file_path = './calib/pose.xlsx'
# 读取Excel文件  
df = pd.read_excel(file_path, engine='openpyxl') 
pose_files = np.array(pose_files)

# 写入pose_files到DataFrame
for i in list(range(count)):
    df.loc[i+2] = pose_files[i,:]

# 将修改后的DataFrame保存回原Excel文件  
df.to_excel(file_path, index=False)  # index=False表示不保存行索引到Excel文件  