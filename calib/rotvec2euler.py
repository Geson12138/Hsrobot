import numpy as np 
from math import *
import cv2

# 根据旋转矩阵反解得到欧拉角

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
def rot2euler(R):
    assert (isRotationMatrix(R))
 
    sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
 
    if not singular:
        x = atan2(R[2, 1], R[2, 2]) * 180 / pi
        y = atan2(-R[2, 0], sy) * 180 / pi
        z = atan2(R[1, 0], R[0, 0]) * 180 / pi
    else:
        x = atan2(-R[1, 2], R[1, 1]) * 180 / pi
        y = atan2(-R[2, 0], sy) * 180 / pi
        z = 0
 
    return np.array([x, y, z])

# 用于根据欧拉角计算旋转矩阵

def myRPY2R_robot(x, y, z):

    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx

    return R


a = (1.791,	1.791,	0.737) # 旋转矢量rx ry rz
temp = cv2.Rodrigues(a) # 旋转矢量rx ry rz转成旋转矩阵
R = temp[0] # 旋转矩阵
euler_angle = rot2euler(R)
print(euler_angle[0])
print(euler_angle[1])
print(euler_angle[2])

# x = 45/180*pi; y = 0/180*pi; z = 0/180*pi # 欧拉角
# r = myRPY2R_robot(x,y,z) # 欧拉角计算旋转矩
# a = cv2.Rodrigues(r) # 旋转矩阵转成旋转矢量rx ry rz
# print(a[0][0],a[0][1],a[0][2])