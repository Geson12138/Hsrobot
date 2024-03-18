import cv2
import numpy as np
import glob
from math import *
import pandas as pd
import os
import time

# p[318.683 259.04]  f[607.013 607.321]
K=np.array([[606.941,0,316.836],
            [0,607.06,253.232],
            [0,0,1]],dtype=np.float64) # realsense相机内参
chess_board_x_num = 10 #棋盘格x方向格子数
chess_board_y_num = 7 #棋盘格y方向格子数
chess_board_len = 20 #单位棋盘格长度,mm


#用于根据欧拉角计算旋转矩阵
def myRPY2R_robot(x, y, z):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx
    return R

#用于根据位姿计算变换矩阵
def pose_robot(x, y, z, Tx, Ty, Tz):
    thetaX = x / 180 * pi
    thetaY = y / 180 * pi
    thetaZ = z / 180 * pi
    R = myRPY2R_robot(thetaX, thetaY, thetaZ)
    t = np.array([[Tx], [Ty], [Tz]])
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
    # RT1=np.linalg.inv(RT1)
    return RT1

#用来从棋盘格图片得到相机外参
def get_RT_from_chessboard(img_path,chess_board_x_num,chess_board_y_num,K,chess_board_len):
    '''
    :param img_path: 读取图片路径
    :param chess_board_x_num: 棋盘格x方向格子数
    :param chess_board_y_num: 棋盘格y方向格子数
    :param K: 相机内参
    :param chess_board_len: 单位棋盘格长度,mm
    :return: 相机外参
    '''
   
    # 读取图像  
    # img_path = '123.png'
    img = cv2.imread(img_path)  
    # print(image_path)
    # print(img.shape)

    # # 创建自定义窗口  
    # cv2.namedWindow('image', cv2.WINDOW_NORMAL)   
    # # # 在窗口中显示图像  
    # cv2.imshow('image', img)  
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    # cv2.namedWindow('image', cv2.WINDOW_NORMAL)   
    # # # 在窗口中显示图像  
    # cv2.imshow('image', gray)  
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, (chess_board_x_num, chess_board_y_num), cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)

    # print(corners)

    corner_points=np.zeros((2,corners.shape[0]),dtype=np.float64)
    for i in range(corners.shape[0]):
        corner_points[:,i]=corners[i,0,:]
    # print(corner_points)
    object_points=np.zeros((3,chess_board_x_num*chess_board_y_num),dtype=np.float64)
    flag=0
    for i in range(chess_board_y_num):
        for j in range(chess_board_x_num):
            object_points[:2,flag]=np.array([(10-j-1)*chess_board_len,(7-i-1)*chess_board_len])
            flag+=1
    # print(object_points)

    retval,rvec,tvec  = cv2.solvePnP(object_points.T,corner_points.T, K, distCoeffs=None)
    # print(rvec.reshape((1,3)))
    # RT=np.column_stack((rvec,tvec))
    RT=np.column_stack(((cv2.Rodrigues(rvec))[0],tvec))
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    # RT=pose(rvec[0,0],rvec[1,0],rvec[2,0],tvec[0,0],tvec[1,0],tvec[2,0])
    # print(RT)

    # print(retval, rvec, tvec)
    # print(RT)
    # print('')
    return RT


# files = os.listdir(folder)
# file_num=len(files)
# RT_all=np.zeros((4,4,file_num))

# print(get_RT_from_chessboard('calib/2.bmp', chess_board_x_num, chess_board_y_num, K, chess_board_len))

good_picture=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17]#存放可以检测出棋盘格角点的图片
file_num=len(good_picture)

#计算board to cam 变换矩阵
R_all_chess_to_cam_1=[]
T_all_chess_to_cam_1=[]
for i in good_picture:
    # print(i)
    image_path=str(i)+'_Color.png'
    # print(image_path)
    RT=get_RT_from_chessboard(image_path, chess_board_x_num, chess_board_y_num, K, chess_board_len)

    # RT=np.linalg.inv(RT)

    R_all_chess_to_cam_1.append(RT[:3,:3])
    T_all_chess_to_cam_1.append(RT[:3, 3].reshape((3,1)))
# print(T_all_chess_to_cam.shape)

#计算end to base变换矩阵
file_address=r"pose.xlsx"#从记录文件读取机器人六个位姿
sheet_1 = pd.read_excel(file_address,engine='openpyxl') #  pip install openpyxl
R_all_end_to_base_1=[]
T_all_end_to_base_1=[]
# print(sheet_1.iloc[0]['ax'])
for i in good_picture:
    # print(i)
    # print(sheet_1.iloc[i-1]['ax'],sheet_1.iloc[i-1]['ay'],sheet_1.iloc[i-1]['az'],sheet_1.iloc[i-1]['dx'],
    #                                   sheet_1.iloc[i-1]['dy'],sheet_1.iloc[i-1]['dz'])
    RT=pose_robot(sheet_1.iloc[i-1]['ax'],sheet_1.iloc[i-1]['ay'],sheet_1.iloc[i-1]['az'],sheet_1.iloc[i-1]['dx'],
                                      sheet_1.iloc[i-1]['dy'],sheet_1.iloc[i-1]['dz'])
    # print((sheet_1.iloc[i-1]['ax'],sheet_1.iloc[i-1]['ay'],sheet_1.iloc[i-1]['az'],sheet_1.iloc[i-1]['dx'],
    #                                   sheet_1.iloc[i-1]['dy'],sheet_1.iloc[i-1]['dz']))
    # RT=np.column_stack(((cv2.Rodrigues(np.array([[sheet_1.iloc[i-1]['ax']],[sheet_1.iloc[i-1]['ay']],[sheet_1.iloc[i-1]['az']]])))[0],
    #                    np.array([[sheet_1.iloc[i-1]['dx']],
    #                                   [sheet_1.iloc[i-1]['dy']],[sheet_1.iloc[i-1]['dz']]])))
    # RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    # RT = np.linalg.inv(RT)

    R_all_end_to_base_1.append(RT[:3, :3])
    T_all_end_to_base_1.append(RT[:3, 3].reshape((3, 1)))

# print(R_all_end_to_base_1)
R,T=cv2.calibrateHandEye(R_all_end_to_base_1,T_all_end_to_base_1,R_all_chess_to_cam_1,T_all_chess_to_cam_1)#手眼标定
RT=np.column_stack((R,T))
RT = np.row_stack((RT, np.array([0, 0, 0, 1])))#即为cam to end变换矩阵
print('相机相对于末端的变换矩阵为：')
print(RT)


