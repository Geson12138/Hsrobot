import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from pykin.utils import transform_utils as transform_utils

# 加载预定义的字典
dict_gen = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
# cameraMatrix = np.array([[617.306, 0.0, 424.5],
#                         [0.0, 617.306, 240.5],
#                         [ 0.0, 0.0, 1.0]])
# distCoeffs = np.array([-0.001, 0.0, 0.0, 0.0, 0.0])

# cameraMatrix = np.array([[389.05096435546875, 0.0, 321.7474365234375],
#                         [0.0, 388.4710998535156, 238.11932373046875],
#                         [0.0, 0.0, 1.0]],np.float32)

# robot_q0
# cameraMatrix = np.array([[910.7659301757812, 0.0, 646.5628051757812],
#                         [0.0, 910.3656616210938, 356.5755310058594],
#                         [0.0, 0.0, 1.0]],np.float32)
# distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# robot_q1
distCoeffs = np.array([-0.054244257509708405, 0.06183071807026863, -0.00039655950968153775, 0.001002355944365263, -0.019506923854351044])
cameraMatrix = np.array([[387.25762939453125, 0.0, 321.7474365234375],
                        [0.0, 386.680419921875, 238.11932373046875],
                        [0.0, 0.0, 1.0]])

# Initialize the detector parameters using default values
parameters =  cv2.aruco.DetectorParameters()

# =========================实时视频流检测=============================
rospy.init_node('get_images', anonymous=True)
bridge = CvBridge()

rvec = None
tvec = None

cv2.namedWindow('Video',cv2.WINDOW_NORMAL)
cv2.namedWindow('frame_makers',cv2.WINDOW_NORMAL)

while True:

    # color_image = rospy.wait_for_message("/robot1_camera/color/image_raw", Image, timeout=None)
    color_image = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=None)
    color_img = bridge.imgmsg_to_cv2(color_image, 'bgr8')

    # 显示帧
    cv2.imshow('Video', color_img)

    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)  

    # Detect the markers in the image
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dict_gen, parameters=parameters)
    markid = 1
    # if at least one marker detected
    if markerIds is not None:

        diamondCorners, diamondIds = cv2.aruco.detectCharucoDiamond(gray, markerCorners, markerIds, 1.39, cameraMatrix, distCoeffs) # square length / marker length

        print(diamondIds)
        # print(diamondIds.shape)
        # print(len(diamondIds))
        # print(diamondCorners)
        # print(type(diamondCorners))

        if diamondIds.ndim > 1:
            
            for i in range(len(diamondIds)):
                if diamondIds[i][0][0] == markid:
                    diamondCorner = diamondCorners[i]

            # print(diamondCorner)
            # print(type(diamondCorner))
            # print(len(diamondCorner))

            if len(diamondCorner) > 0:

                diamondCorner = diamondCorner.reshape(1, -1, 2)  # reshape to (1, N, 2)
                # print(diamondCorner)

                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(diamondCorner, 0.032, cameraMatrix, distCoeffs) # square length

                rvec = np.array(rvec)
                tvec = np.array(tvec)

                R = cv2.Rodrigues(rvec)[0]

                # print(f'rvec: {rvec[0][0]}\n')
                # print(f'tvec: {tvec[0][0]}\n')
                # print(f'R: {R}\n')

                euler_angle_rad = transform_utils.get_rpy_from_matrix(R)
                euler_angle = np.rad2deg(euler_angle_rad)
                print(f'euler_angle: {euler_angle}\n')
                print(f'tvec: {tvec}\n')


                frame_axes = cv2.drawFrameAxes(color_img, cameraMatrix, distCoeffs, rvec, tvec, 0.1)   

                cv2.aruco.drawDetectedMarkers(color_img, markerCorners,markerIds)

                if markerIds is not None:

                    cv2.imshow('frame_markers',frame_axes)

    else:
        continue

    # 按 'Esc' 退出循环
    if cv2.waitKey(1) & 0xFF == 27:  # 27是Esc键的ASCII值
        break

# 关闭所有OpenCV窗口

cv2.destroyAllWindows()