'''
Description: Interface functions to hsrobot controller
Update: 3.20.2024
Author: Shuai Gan
Mail: shuai.gan@ia.ac.cn
Copyright by Casia, Robotic Theory and Application Group
'''

# 导入 Python SDK，需要和CPS.py在同一个文件夹下
from src.CPS import CPSClient
import roboticstoolbox as rtb
import numpy as np
import math
import spatialmath.base as spatialmathbase
import time
from hsrobot_des import HSRobotdes
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from spatialmath import SE3
#读取图片
bridge = CvBridge() 

# self.trans_cam2wrist = np.array([[ 5.28869282e-02, -9.98584258e-01,  5.69673203e-03,  7.77746026e+01],
#                                          [ 9.98043816e-01,  5.30470434e-02,  3.30840084e-02, -5.14321849e+01],
#                                          [-3.33393647e-02,  3.93587660e-03,  9.99436339e-01,  2.76224141e+01],
#                                          [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])


class HSROBOT(object):

    def __init__(self):
        #------------------------------------开始连接-------------------------------------
        # print('准备工作开始, 连接robot')
        # 设置机器人的IP 和端口号，注意本机IPv4设置需要和机器人IP在同一个网段
        IP = '192.168.31.102'
        nPort = 10003
        # 创建一个机器人对象
        self.arm = CPSClient()
        # 连接机器人的服务器
        nRet = self.arm.HRIF_Connect(0,IP,nPort)
        # 检测控制器是否连接
        nRet = self.arm.HRIF_IsConnected(0)
        print(f'Connecting to Robot Controller: {nRet}')
        # 相机相对于tcp(第五个关节)的变换矩阵,运行calib/calibration.py得到
        # self.trans_cam2wrist = np.array([[ 4.79173933e-02, -9.93733916e-01,  1.00979344e-01,  6.89458508e+01],
        #                                  [ 9.98850997e-01,  4.77509031e-02, -4.06661492e-03, -4.71596805e+01],
        #                                  [-7.80721685e-04,  1.01058180e-01,  9.94880211e-01,  3.61076252e+01],
        #                                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        self.trans_cam2wrist=np.array([[ 4.34533477e-02, -9.98988685e-01 ,-1.15504612e-02,  9.79193679e+01],
                                        [-3.12009268e-03 , 1.14256275e-02 ,-9.99929858e-01, -7.70839322e+01],
                                        [ 9.99050585e-01 , 4.34863382e-02 ,-2.62045552e-03 ,-3.47568309e+01],
                                        [ 0.00000000e+00 , 0.00000000e+00  ,0.00000000e+00 , 1.00000000e+00]])
        

    '''
    Function: 根据RPY角计算旋转矩阵:
    Input: RPY Angles, in radians
    Output: Rotation Matrix
    RPY角是欧拉角的一种, 是绕固定轴旋转的欧拉角, 按照x、y、z的顺序依次旋转roll/theta_x、pitch/theta_y、yaw/theta_z, 旋转矩阵左乘得到Rz@Ry@Rx
    (1) 绕固定坐标系(外旋)按照x、y、z的顺序旋转,为左乘, 也叫RPY角
    (2) 绕自身坐标系(内旋)按照z、y、x的顺序旋转,为右乘
    上述(1)和(2)等价，且由欧拉角计算旋转矩阵具有唯一性
    '''
    def RPY2RotMatrix(self,theta_x, theta_y, theta_z):

        Rx = np.array([[1, 0, 0], [0, math.cos(theta_x), -math.sin(theta_x)], [0, math.sin(theta_x), math.cos(theta_x)]])
        Ry = np.array([[math.cos(theta_y), 0, math.sin(theta_y)], [0, 1, 0], [-math.sin(theta_y), 0, math.cos(theta_y)]])
        Rz = np.array([[math.cos(theta_z), -math.sin(theta_z), 0], [math.sin(theta_z), math.cos(theta_z), 0], [0, 0, 1]])
        R = Rz@Ry@Rx

        return R

    '''
    Function: 从旋转矩阵的n、o和a分量计算旋转的rpy角
    Input: n, o, a
    Output: roll, pitch, yaw
    RPY角是绕固定轴旋转的欧拉角, 按照绕x、y、z的顺序依次旋转(roll/theta_x)、(pitch/theta_y)、(yaw/theta_z)
    旋转矩阵为左乘, R = Rz(yaw/theta_z)*Ry(pitch/theta_y)*Rx(roll/theta_x)
    '''
    def rotation_matrix_to_rpy(self,n, o, a): 
        
        roll = np.arctan2(o[2], a[2])  
        pitch = np.arctan2(-n[2], np.sqrt(o[2]**2 + a[2]**2))  
        # yaw = np.arctan2(o[2], o[0]) - np.pi/2  
        yaw = np.arctan2(n[1], n[0])
        
        return roll, pitch, yaw

    '''
    Function: 用于根据位姿pose(位置+姿态)计算变换矩阵:
    Input:(1) RPY Angles, theta_x/yaw, theta_y/pitch, theta_z/roll, in degrees (2) Position, Tx, Ty, Tz, in mm
    Output: Transformation Matrix
    '''
    def pose_robot(self,x, y, z, Tx, Ty, Tz):
        thetaX = x / 180 * math.pi
        thetaY = y / 180 * math.pi
        thetaZ = z / 180 * math.pi
        R = self.RPY2RotMatrix(thetaX, thetaY, thetaZ)
        t = np.array([[Tx], [Ty], [Tz]])
        RT1 = np.column_stack([R, t])  # 列合并
        RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
        # RT1=np.linalg.inv(RT1)
        return RT1

    '''
    Function: 根据旋转矩阵反解得到RPY角(欧拉角的一种), 旋转矩阵计算欧拉角不具有唯一性！
    Input: Rotation Matrix (3x3)
    Ouput: RPY Angles, [x: pitch y: yaw z: roll], in radians        
    '''
    # 判断是否是旋转矩阵，根据旋转矩阵和其转置乘积为单位矩阵的原则
    def isRotationMatrix(self,R):
        Rt = np.transpose(R) # 旋转矩阵的转置为其逆矩阵
        shouldBeIdentity = np.dot(Rt, R) # 互为逆矩阵的两个矩阵点乘为单位矩阵
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
    
    def rot2euler(self,R):
        assert (self.isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2]) * 180 / np.pi
            y = math.atan2(-R[2, 0], sy) * 180 / np.pi
            z = math.atan2(R[1, 0], R[0, 0]) * 180 / np.pi
        else:
            x = math.atan2(-R[1, 2], R[1, 1]) * 180 / np.pi
            y = math.atan2(-R[2, 0], sy) * 180 / np.pi
            z = 0
        return np.array([x, y, z])
    
    '''
    Function: 获取腕关节到基座的变换矩阵
    Input: None
    Output: Transformation Matrix from Wrist to Base
    '''
    
    def get_T_wrist2base(self):

        r_poselist = [] # 定义返回值空列表
        self.arm.HRIF_ReadActPos(0,0, r_poselist)
        c_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
        # -------------------------------------利用机器人工具箱和DH参数建立运动学模型-----------------------------
        rtb_hsrobot = rtb.DHRobot([rtb.RevoluteDH(d=0.26,alpha=np.pi/2),
                            rtb.RevoluteDH(a=0.48,alpha=np.pi),
                            rtb.RevoluteDH(alpha=np.pi/2),
                            rtb.RevoluteDH(d=0.52, alpha=-np.pi/2),
                            rtb.RevoluteDH(alpha=np.pi/2)
                            ], name="hsrobot")#rtb.RevoluteDH(d=0.192)第六关节
        q_init = np.array([0,np.pi/2,np.pi/2,0,0])
        q_cur = c_joint_pos[0:5]/180*np.pi # 机器人前五个关节位置
        q = np.add(q_init,q_cur)
        temp_T = rtb_hsrobot.fkine(q) #运用机器人工具箱得到第五个关节的位姿矩阵
        temp_r = temp_T.A[:3,:3];temp_t = temp_T.A[:3,3]*1000
        # euler_angles = spatialmathbase.tr2rpy(temp_r)  
        # print("欧拉角:", np.degrees(euler_angles[0]), np.degrees(euler_angles[1]), np.degrees(euler_angles[2]))
        # temp_r1 = self.RPY2RotMatrix(euler_angles[0],euler_angles[1],euler_angles[2])
        temp_rt = np.column_stack([temp_r, temp_t])  # 列合并
        trans_wrist2base = np.row_stack((temp_rt, np.array([0,0,0,1]))) # 行合并
        return trans_wrist2base
    
    def get_T_wrist2base2(self):
        hs_robot = HSRobotdes()
        r_poselist = [] # 定义返回值空列表
        self.arm.HRIF_ReadActPos(0,0, r_poselist)
        c_joint_pos = np.array([float(i) for i in r_poselist[0:6]])
        tem_T = hs_robot.fkine_to_joint5(c_joint_pos[0:5]/180*np.pi)
        temp_r = tem_T.A[:3,:3];temp_t = tem_T.A[:3,3]*1000
        temp_rt = np.column_stack([temp_r, temp_t])  # 列合并
        trans_wrist2base = np.row_stack((temp_rt, np.array([0,0,0,1]))) # 行合并
        return trans_wrist2base
        

    '''
    Function: 根据视觉感认知输出，计算机械臂末端的期望位姿
    Input: 感认知模块的输出，相机坐标系下的插头四个角点的坐标 3xN
    Output: 机械臂末端TCP的期望运动位姿 d_tcp_pos : in mm, d_tcp_ori: RPY Angles
    '''
    def get_TCP_targetPose(self,cam_grasp_point):

        cam_center_point = np.mean(cam_grasp_point,axis=1) # 相机坐标系下的抓取点中心的齐次坐标表示
        plug_xaxis_p = np.mean(cam_grasp_point[:,2:4],axis=1) # 相机坐标系下的插头x轴上的点
        plug_yaxis_p = (cam_grasp_point[:,0]+cam_grasp_point[:,3])/2 # 相机坐标系下的插头y轴上的点
        plug_xaxis_vec = plug_xaxis_p - cam_center_point # 相机坐标系下的插头x轴的方向向量
        plug_yaxis_vec = plug_yaxis_p - cam_center_point # 相机坐标系下的插头y轴的方向向量
        # 归一化，得到表示插头坐标系{plug_frame}主轴方向的单位矢量
        plug_xaxis_vec = plug_xaxis_vec / np.linalg.norm(plug_xaxis_vec) # 归一化，相机坐标系下的插头x轴的单位方向向量
        plug_yaxis_vec = plug_yaxis_vec / np.linalg.norm(plug_yaxis_vec) # 归一化，相机坐标系下的插头y轴的单位方向向量
        plug_zaxis_vec = np.cross(plug_xaxis_vec,plug_yaxis_vec)
        plug_zaxis_vec = plug_zaxis_vec / np.linalg.norm(plug_zaxis_vec) # 归一化，相机坐标系下的插头z轴的单位方向向量
        plug_yaxis_vec = np.cross(plug_zaxis_vec,plug_xaxis_vec)
        plug_yaxis_vec = plug_yaxis_vec / np.linalg.norm(plug_yaxis_vec) # 归一化，相机坐标系下的插头y轴的单位方向向量

        RT = np.zeros((4,4)) # 向量表示为齐次坐标形式
        RT[0:3,0] = plug_xaxis_vec; RT[0:3,1] = plug_yaxis_vec; RT[0:3,2] = plug_zaxis_vec; RT[0:3,3] = cam_center_point.T; RT[3,3] = 1
        # 得到腕关节到基座的变换矩阵
        trans_wrist2base = self.get_T_wrist2base2()
        grasp_target_pose = trans_wrist2base @ self.trans_cam2wrist @ RT


        # print(f'腕关节到基座的变换矩阵为: {trans_wrist2base}')
        # base_axis = trans_wrist2base @ self.trans_cam2wrist @ RT # 将表示相机坐标系的坐标轴单位方向向量转换到机器人坐标系下
        # print(base_axis)
        # vector_x1 = base_axis[0:3,0].T; vector_y1 = base_axis[0:3,1].T; vector_z1 = base_axis[0:3,2].T
        # 单位向量，表示坐标轴方向
        # unit_x = np.array([1.0, 0.0, 0.0]); unit_y = np.array([0.0, 1.0, 0.0]); unit_z = np.array([0.0, 0.0, 1.0])
        # 计算投影分量
        # n_x = np.dot(vector_x1, unit_x); n_y = np.dot(vector_x1, unit_y); n_z = np.dot(vector_x1, unit_z)
        # o_x = np.dot(vector_y1, unit_x); o_y = np.dot(vector_y1, unit_y); o_z = np.dot(vector_y1, unit_z)
        # a_x = np.dot(vector_z1, unit_x); a_y = np.dot(vector_z1, unit_y); a_z = np.dot(vector_z1, unit_z)
        # print("X 轴上的投影分量:", n_x,o_x,a_x); print("Y 轴上的投影分量:", n_y,o_y,a_y); print("Z 轴上的投影分量:", n_z,o_z,a_z)
        # print(n_x**2+o_x**2+a_x**2); print(n_y**2+o_y**2+a_y**2); print(n_z**2+o_z**2+a_z**2)
        # target_RotMatrix = np.array(
        #     [[n_x,o_x,a_x],
        #     [n_y,o_y,a_y],
        #     [n_z,o_z,a_z]
        #     ])

        # temp_RT = np.ones(4)
        # temp_RT[0:3] = cam_center_point.T
        # grasp_target_point = trans_wrist2base @ self.trans_cam2wrist @ temp_RT 
        d_tcp_pos = grasp_target_pose[0:3,3] # 末端的期望位置
        d_tcp_ori = self.rot2euler(grasp_target_pose[0:3,0:3]) # 末端的期望姿态
        return d_tcp_pos,d_tcp_ori
    
    def get_TCP_obs_Pose(self,cam_obs_point):
        #根据平面三个点算出法向量
        vec1 = (cam_obs_point[1]-cam_obs_point[0])/np.linalg.norm(cam_obs_point[1]-cam_obs_point[0])
        vec2 = (cam_obs_point[2]-cam_obs_point[0])/np.linalg.norm(cam_obs_point[2]-cam_obs_point[0])

        #y方向
        y_axis = np.cross(vec1,vec2)
        y_axis = y_axis/np.linalg.norm(y_axis)

        #z方向固定向下
        z_axis = vec1/np.linalg.norm(vec1)

        #x方向
        x_axis = np.cross(y_axis,z_axis)
        x_axis = x_axis/np.linalg.norm(x_axis)

        RT = np.zeros((4,4)) # 向量表示为齐次坐标形式
        RT[0:3,0] = x_axis; RT[0:3,1] = y_axis; RT[0:3,2] = z_axis; RT[0:3,3] = cam_obs_point.T[1]; RT[3,3] = 1
        print('RT',RT)
        # 得到腕关节到基座的变换矩阵
        trans_wrist2base = self.get_T_wrist2base2()
        obs_target_pose = trans_wrist2base @ self.trans_cam2wrist @ RT

        obs_tcp_pos = obs_target_pose[0:3,3] # 末端的期望位置
        obs_tcp_ori = self.rot2euler(obs_target_pose[0:3,0:3]) # 末端的期望姿态

        return obs_tcp_pos,obs_tcp_ori

    
    #二维码识别
    def get_TCP_targetPose2(self):
        dist=np.array(([[0.0,0.0,0.0,0.0,0.0]]))
        newcameramtx=np.array([[189.076828   ,  0.    ,     361.20126638]
        ,[  0 ,2.01627296e+04 ,4.52759577e+02]
        ,[0, 0, 1]])

        mtx=np.array([[606.941  , 0.      ,   316.836],
        [  0.       ,  607.06, 253.232],
        [  0.,           0.,           1.        ]])

        bridge = CvBridge() 
        rospy.init_node('get_images', anonymous=True)
        # 定义一个ros话题，把self.color_image发布出去
        image_pub = rospy.Publisher('image_topic', Image, queue_size=10)
        font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

        trans_wrist2base = self.get_T_wrist2base2()
        while True:
            time_out = 0
            while time_out < 2:
                color_img = rospy.wait_for_message("/hs_camera/color/image_raw", Image, timeout=None)
                depth_img = rospy.wait_for_message("/hs_camera/aligned_depth_to_color/image_raw", Image, timeout=None)
                time_out = time_out +1
            # if time_count>= 10:
            #     break
            color_image = bridge.imgmsg_to_cv2(color_img, 'bgr8')
            cv2.imwrite(f'save_images/IMG_3739.jpg',color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])
            
            frame=cv2.imread('save_images/IMG_3739.jpg')

        # cap = cv2.VideoCapture(0)


            # ret, frame = cap.read()
            h1, w1 = frame.shape[:2]
            # 读取摄像头画面
            # 纠正畸变
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
            dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            x, y, w1, h1 = roi
            dst1 = dst1[y:y + h1, x:x + w1]
            frame=dst1
        
        
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
            parameters =  aruco.DetectorParameters_create()
            dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            '''
            detectMarkers(...)
                detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
                mgPoints]]]]) -> corners, ids, rejectedImgPoints
            '''
        
            #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        
        #    如果找不打id
            if ids is not None:
        
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.04, mtx, dist)
                # 估计每个标记的姿态并返回值rvet和tvec ---不同
                # from camera coeficcients
                (rvec-tvec).any() # get rid of that nasty numpy value array error
        
        #        aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1) #绘制轴
        #        aruco.drawDetectedMarkers(frame, corners) #在标记周围画一个正方形
        
                for i in range(rvec.shape[0]):

                    #转换为位姿矩阵
                    rvec = np.array(rvec)
                    tvec = np.array(tvec)*1000
                    R,_ = cv2.Rodrigues(rvec)
                    RT = np.zeros((4,4))
                    RT[:3,:3] = R
                    RT[:3,3] = tvec
                    RT[3,3] = 1
                    RT = RT*SE3.Rx(np.pi)*SE3.Rz(np.pi/2)*SE3.Trans(-90,3,0)*SE3.Ry(-np.pi*15/180)
                    print('RT',RT)
                    tvec = RT[:3,3].reshape(1,1,3)
                    grasp_target_pose = trans_wrist2base @ self.trans_cam2wrist @ RT
                    d_tcp_pos = grasp_target_pose[0:3,3] # 末端的期望位置
                    d_tcp_ori = self.rot2euler(grasp_target_pose[0:3,0:3]) # 末端的期望姿态

                    rvec = (self.rot2euler(RT[:3,:3])/180*np.pi).reshape(1,1,3)
                    print('rvec',rvec)
                    print('tvec',tvec)
                    aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :]/1000, 0.03)
                    aruco.drawDetectedMarkers(frame, corners)
                ###### DRAW ID #####
                cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
        
        
            else:
                ##### DRAW "NO IDS" #####
                cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
        
        
            # 显示结果框架
            cv2.imshow("frame",frame)
        
            key = cv2.waitKey(1)
        
            if key == 27:         # 按esc键退出
                print('esc break...')
                # cap.release()
                cv2.destroyAllWindows()
                break
        
            if key == ord(' '):   # 按空格键保存
        #        num = num + 1
        #        filename = "frames_%s.jpg" % num  # 保存一张图像
                filename = str(time.time())[:10] + ".jpg"
                cv2.imwrite(filename, frame)
        return d_tcp_pos, d_tcp_ori
    

    '''
    Function Group: 控制电动夹爪, 配置电箱通用数字输出DO0 & DO7高低电平, 0低电平 1高电平, 1&0关 0&1开
    Input:  None
    Output: None
    '''
    def gripper_close(self):

        # --------------夹具闭合, 设置DO0电平状态为1---------------
        nVal_1 = 1;  nVal_2 = 0 # 0: 低电平 1: 高电平
        # 定义需要设置的位
        nBit_1 = 0;  nBit_2 = 7
        # 设置电箱第 nBit 位 DO 状态为 nVal
        self.arm.HRIF_SetBoxDO(0, nBit_1, nVal_1)   
        self.arm.HRIF_SetBoxDO(0, nBit_2, nVal_2)
        print('夹具闭合') 

    def gripper_open(self):

        # ----------------夹具打开, 设置DO7电平状态为1----------------
        nVal_1 = 0;  nVal_2 = 1 # 0: 低电平 1: 高电平
        # 定义需要设置的位
        nBit_1 = 0;  nBit_2 = 7
        # 设置电箱第 nBit 位 DO 状态为 nVal
        self.arm.HRIF_SetBoxDO(0, nBit_1, nVal_1)   
        self.arm.HRIF_SetBoxDO(0, nBit_2, nVal_2) 
        print('夹具打开') 

    def gripper_init(self):

        # 夹爪初始化,两位都输出为0: 低电平
        nVal_1 = 0;  nVal_2 = 0
        # 定义需要设置的位
        nBit_1 = 0;  nBit_2 = 7
        # 设置电箱第 nBit 位 DO 状态为 nVal
        self.arm.HRIF_SetBoxDO(0, nBit_1, nVal_1)   
        self.arm.HRIF_SetBoxDO(0, nBit_2, nVal_2) 
        print('夹具初始化')

    '''
    Function: 机器人关节空间运动，增加了判断是否运动完成
    Input: 关节空间目标位置, 关节运动速度 0~100, 工具坐标系
    Output: None
    '''
    def move_j(self,joint_pos,vel,sTcpName = "TCP"): 

        # -------------------------------预定义参数--------------------------------
        self.arm.HRIF_SetOverride(0,0,vel/100) # 设置速度
        # 定义笛卡尔空间目标位置
        tcp_pose = np.array([0,0,0,0,0,0])
        # 定义用户坐标变量：目标空间坐标所处用户坐标系，与示教器页面的名称对应, nIsUseJoint=1 时无效，可使用默认名称"Base"
        sUcsName = "Base"
        # 关节角度最大运动速度 单位[°/s]
        dVelocity = 30  
        # 关节角度最大运动加速度单位[°/s^2]
        dAcc = 20
        dRadius = 5 # 定义过渡半径: 过渡半径，单位[mm]
        nIsUseJoint= 1 # 定义是否使用关节角度: 是否使用关节角度作为目标点，1 使用 0 不使用
        # 定义是否使用检测 DI 停止: 如果 nIsSeek 为 1，则开启检测 DI 停止，路点运动过程中如果电箱的 nIOBit 位索引的DI 的状态=nIOState 时，机器人停止运动，否则运动到目标点完成运动
        nIsSeek = 0
        # 定义检测的 DI 索引: 电箱对应 DI 索引，nIsSeek=0 时无效
        nIOBit = 0
        # 定义检测的 DI 状态: 检测的 DI 状态，，nIsSeek=0 时无效
        nIOState = 0
        # 定义路点 ID: 当前路点 ID，可以自定义，也可以按顺序设置为"1"，"2"，"3"
        stdCmdID = "0"
        self.arm.HRIF_MoveJ(0,0,tcp_pose,joint_pos,sTcpName,sUcsName,dVelocity,dAcc,dRadius,nIsUseJoint, nIsSeek, nIOBit, nIOState, stdCmdID)
        result = []
        self.arm.waitMovementDone(0,0,result) # 等待运动完成

    '''
    Function: 机器人笛卡尔空间直线运动，增加了判断是否运动完成
    Input: 笛卡尔空间目标位姿, 笛卡尔空间运动速度 0~100, 工具坐标系
    Output: None
    '''
    def move_l(self,d_tcp_pose,vel,sTcpName = "TCP"):

        # -------------------------------预定义参数--------------------------------    
        self.arm.HRIF_SetOverride(0,0,vel/100) # 设置速度
        # 定义关节空间目标位置
        RawACSpoints = [ 0, 0, 90, 0, 90, 0]
        # 定义用户坐标变量
        sUcsName = "Base"
        # 笛卡尔空间运动最大速度, 单位[mm/s], [°/s]
        dVelocity = 50
        # 笛卡尔空间运动最大加速度，单位[mm/s^2], [°/s^2]
        dAcc = 50
        # 定义过渡半径
        dRadius = 5
        # 定义是否使用检测 DI 停止
        nIsSeek = 0
        # 定义检测的 DI 索引
        nIOBit = 0
        # 定义检测的 DI 状态
        nIOState = 0
        # 定义路点 ID
        strCmdID = "0"
        # 执行路点运动
        nRet = self.arm.HRIF_MoveL(0,0, d_tcp_pose, RawACSpoints, sTcpName, sUcsName, dVelocity, dAcc, dRadius,nIsSeek,nIOBit, nIOState, strCmdID)
        result = []
        self.arm.waitMovementDone(0,0,result) # 等待运动完成

    '''
    Function: 机器人笛卡尔空间圆弧轨迹运动，增加了判断是否运动完成
    Input: 圆弧运动起点、圆弧运动中点、圆弧运动终点, 笛卡尔空间圆弧运动速度 0~100
    Output: None
    '''
    def move_c(self,dStartPoint,dAuxPoint,dEndPoint,vel):

        # -------------------------------预定义参数--------------------------------    
        self.arm.HRIF_SetOverride(0,0,vel/100) # 设置速度
        # 是否固定姿态, 圆弧整个运动过程中是否保持姿态不变：0 固定姿态 1 不固定姿态
        nFixedPosure = 0
        # 圆弧类型 0：整圆 1：圆弧段
        nMoveCType = 1
        # 整圆圈数，当使用圆弧运动时无效，通过三个点位确定圆弧路径，当使用整圆运动时表示整圆的圈数，小数部分无效。
        dRadLen = 1
        # 定义笛卡尔空间运动最大速度, 单位[mm/s], [°/s]
        dVelocity = 50
        # 定义笛卡尔空间运动最大加速度，单位[mm/s^2], [°/s^2]
        dAcc = 50
        # 定义过渡半径
        dRadius = 5
        # 定义工具坐标变量
        sTcpName = "TCP_grasp"
        # 定义用户坐标变量
        sUcsName = "Base"
        # 定义路点 ID
        strCmdID = "0"
        # 执行路点运动
        self.arm.HRIF_MoveC(0,0,dStartPoint , dAuxPoint, dEndPoint,
                                   nFixedPosure, nMoveCType, dRadLen, dVelocity, dAcc, dRadius,sTcpName , sUcsName, strCmdID)
        result = []
        self.arm.waitMovementDone(0,0,result) # 等待运动完成