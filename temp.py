import roboticstoolbox as rtb
import numpy as np
from math import *
import numpy as np
import spatialmath.base as spatialmath_base


import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义机器人模型
rtb_hsrobot = rtb.DHRobot([rtb.RevoluteDH(d=0.26,alpha=np.pi/2),
                            rtb.RevoluteDH(a=0.48,alpha=np.pi),
                            rtb.RevoluteDH(alpha=np.pi/2),
                            rtb.RevoluteDH(d=0.52, alpha=-np.pi/2),
                            rtb.RevoluteDH(alpha=np.pi/2),
                            rtb.RevoluteDH(d=0.192),
                            ], name="hsrobot")

# 定义关节角度
pi = np.pi
q_int = np.array([0,pi/2,pi/2,0,0,0])
q_cur = np.array([0/180*pi,0/180*pi,0/180*pi,0/180*pi,0/180*pi,0/180*pi]) # 机器人前五个关节位置
q = np.add(q_int,q_cur)

# 计算每个关节的前向运动学
Ts = rtb_hsrobot.fkine_all(q)

# 显示机器人构型
rtb_hsrobot.plot(q)

# 创建一个新的 3D 图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 在每个关节位置上绘制坐标系
for T in Ts:
    ax.quiver(T.t[0], T.t[1], T.t[2], T.R[0, 0], T.R[1, 0], T.R[2, 0], color='r', length=0.1)  
    ax.quiver(T.t[0], T.t[1], T.t[2], T.R[0, 1], T.R[1, 1], T.R[2, 1], color='g', length=0.1)
    ax.quiver(T.t[0], T.t[1], T.t[2], T.R[0, 2], T.R[1, 2], T.R[2, 2], color='b', length=0.1)

# 显示图形
plt.show()

input("Press Enter to continue...")

# 用于根据欧拉角计算旋转矩阵
# def myRPY2R_robot(x, y, z):
#     Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
#     Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
#     Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
#     R = Rz@Ry@Rx
#     return R

# R = myRPY2R_robot(0,0,pi/2)
# print(R)

# dJ1 = 8.397
# dJ2 = -8.377
# dJ3 = -123.702
# dJ4 = 0.921
# dJ5 = -73.934
# dJ6 = -15.089
# hsrobot = rtb.DHRobot([rtb.RevoluteDH(d=0.26,alpha=pi/2),
#                      rtb.RevoluteDH(a=0.48,alpha=pi),
#                      rtb.RevoluteDH(alpha=pi/2),
#                      rtb.RevoluteDH(d=0.52, alpha=-pi/2),
#                      rtb.RevoluteDH(alpha=pi/2)
#                      ], name="hsrobot")
# # rtb.RevoluteDH(d=0.192)

# rtb_hsrobot = rtb.DHRobot([rtb.RevoluteDH(d=0.26,alpha=np.pi/2),
#                             rtb.RevoluteDH(a=0.48,alpha=np.pi),
#                             rtb.RevoluteDH(alpha=np.pi/2),
#                             rtb.RevoluteDH(d=0.52, alpha=-np.pi/2),
#                             rtb.RevoluteDH(alpha=np.pi/2),
#                             rtb.RevoluteDH(d=0.192),
#                             ], name="hsrobot")
# # 显示机器人构型
# # print(rtb_hsrobot)
# q_int = np.array([0,pi/2,pi/2,0,0,0])
# q_cur = np.array([-0.433/180*pi,-9.019/180*pi,-118.978/180*pi,0.016/180*pi,-66.04/180*pi,17.561/180*pi]) # 机器人前五个关节位置
# q = np.add(q_int,q_cur)
# # 可视化机器人模型  
# rtb_hsrobot.teach(q)
# input("Press Enter to continue...")

# temp_T = rtb_hsrobot.fkine(q) #运用机器人工具箱得到第五个关节的位姿矩阵
# # print(temp_T)
# temp_r = temp_T.A[:3,:3];temp_t = temp_T.A[:3,3]*1000
# temp_t[0] = round(temp_t[0],3)
# temp_t[1] = round(temp_t[1],3)
# temp_t[2] = round(temp_t[2],3)
# # print(temp_r)
# print(temp_t)

# rot2 = np.array([
#                 [ 9.68644205e-01,  1.28405929e-01, -2.12697721e-01],
#                 [ -1.33395714e-01,  9.91020008e-01, -9.21561413e-03],
#                 [ 2.09604358e-01,  3.72996156e-02,  9.77074589e-01],
#                 ])
# rpy_angles = np.rad2deg(spatialmath_base.tr2rpy(rot2))
# print(rpy_angles)

# euler_angles = tr2rpy(temp_r)  
# euler_angles[0] = round(np.degrees(euler_angles[0]),3)
# euler_angles[1] = round(np.degrees(euler_angles[1]),3)
# euler_angles[2] = round(np.degrees(euler_angles[2]),3)
# print("欧拉角:", euler_angles[0], euler_angles[1], euler_angles[2])

# 根据旋转矩阵反解得到欧拉角
# def isRotationMatrix(R):
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype=R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6
 
# def rot2euler(R):
#     assert (isRotationMatrix(R))
#     sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
#     singular = sy < 1e-6
#     if not singular:
#         x = atan2(R[2, 1], R[2, 2]) * 180 / pi
#         y = atan2(-R[2, 0], sy) * 180 / pi
#         z = atan2(R[1, 0], R[0, 0]) * 180 / pi
#     else:
#         x = atan2(-R[1, 2], R[1, 1]) * 180 / pi
#         y = atan2(-R[2, 0], sy) * 180 / pi
#         z = 0
#     return np.array([x, y, z])


# trans_cam2tcp = np.array([[ 0.0366,  0.83415,  -0.55031],
#                           [ 0.00469, 0.55053,   0.83479],
#                           [  0.9996,  -0.033,  0.0162],
#                           ])
                          
# trans_cam2tcp = np.array([[ 0.67645706,  0.4732541 , -0.56430169],
#                            [-0.37851765,  0.88067755 , 0.2848358 ],
#                            [ 0.63176754 , 0.02091896  ,0.77487558]])
# print(trans_cam2tcp)
# euler = rot2euler(trans_cam2tcp)
# print(euler)

# 使用transforms3d的euler.mat2euler函数来获取欧拉角  
# The first character is ‘r’ (rotating == intrinsic), or ‘s’ (static == extrinsic). 
# euler_angles = tf.euler.mat2euler(trans_cam2tcp,'sxyz')
# 使用rad2deg函数将弧度转换为角度  
# euler_angles = [-1.6700528491256965, -0.9978933221161231, -1.4672109925745795]
# euler_degrees = np.rad2deg(euler_angles)  
# print("欧拉角（弧度）:", euler_angles)  
# print("欧拉角（角度）:", euler_degrees) 

# R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
# # print(R)
# X,Y,Z = rot2euler(R)
# print(X,Y,Z)


# rorate_matrix = myRPY2R_robot(0/180*pi,-90/180*pi,-10/180*pi)
# print(rorate_matrix)

# a = [1.3990, 0.9690, 1.4190]
# print(ai/pi*180 for ai in a)


# aa = np.array( [[    -82.905,     -82.027,     -62.731,     -64.161],
#                 [     44.063,      35.901,      37.838,      46.328],
#                 [        413,         466,         471,         415]], dtype=np.float64)
# bb = np.array( [[    -82.025,     -84.749,     -63.776,     -63.171],
#                 [     43.957,      36.748,      37.219,      46.104],
#                 [        412,         477,         473,         413]], dtype=np.float64)