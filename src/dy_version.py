import numpy as np
import time
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
#读取图片
from spatialmath import SE3
import math
bridge = CvBridge() 
 
 
 
# mtx = np.array([
#         [2946.48,       0, 1980.53],
#         [      0, 2945.41, 1129.25],
#         [      0,       0,       1],
#         ])
# #我的手机拍棋盘的时候图片大小是 4000 x 2250
# #ip摄像头拍视频的时候设置的是 1920 x 1080，长宽比是一样的，
# #ip摄像头设置分辨率的时候注意一下
#
#
# dist = np.array( [0.226317, -1.21478, 0.00170689, -0.000334551, 1.9892] )
 
 
#相机纠正参数
 
# dist=np.array(([[-0.51328742,  0.33232725 , 0.01683581 ,-0.00078608, -0.1159959]]))
#
# mtx=np.array([[464.73554153, 0.00000000e+00 ,323.989155],
#  [  0.,         476.72971528 ,210.92028],
#  [  0.,           0.,           1.        ]])
# 判断是否是旋转矩阵，根据旋转矩阵和其转置乘积为单位矩阵的原则
def isRotationMatrix(R):
    Rt = np.transpose(R) # 旋转矩阵的转置为其逆矩阵
    shouldBeIdentity = np.dot(Rt, R) # 互为逆矩阵的两个矩阵点乘为单位矩阵
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
def rot2euler(R):
        assert (isRotationMatrix(R))
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

while True:
    time_out = 0
    while time_out < 2:
        color_img = rospy.wait_for_message("/hs_camera/color/image_raw", Image, timeout=None)
        depth_img = rospy.wait_for_message("/hs_camera/aligned_depth_to_color/image_raw", Image, timeout=None)
        time_out = time_out +1
    # if time_count>= 10:
    #     break
    color_image = bridge.imgmsg_to_cv2(color_img, 'bgr8')
    cv2.imwrite(f'IMG_3739.jpg',color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])
    
    frame=cv2.imread('IMG_3739.jpg')

# cap = cv2.VideoCapture(0)


    # ret, frame = cap.read()
    h1, w1 = frame.shape[:2]
    print('h1',h1)
    print('w1',w1)
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
 
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.02, mtx, dist)
        # 估计每个标记的姿态并返回值rvet和tvec ---不同
        # from camera coeficcients
        (rvec-tvec).any() # get rid of that nasty numpy value array error
 
#        aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1) #绘制轴
#        aruco.drawDetectedMarkers(frame, corners) #在标记周围画一个正方形
 
        for i in range(rvec.shape[0]):
            
            #转换为位姿矩阵
            rvec = np.array(rvec)
            tvec = np.array(tvec)
            R,_ = cv2.Rodrigues(rvec)
            pose = np.zeros((4,4))
            pose[:3,:3] = R
            pose[:3,3] = tvec
            pose[3,3] = 1

            pose = pose*SE3.Rx(np.pi)*SE3.Rz(np.pi/2)
            R = pose[:3,:3]
            rvec = rot2euler(R) / 180 * np.pi
            rvec = rvec.reshape(1,1,3)
            print('rvec',rvec)
            print('tvec',tvec)
            print('pose',pose)

            aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)

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