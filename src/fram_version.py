import numpy as np
import time
import cv2
# import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
#读取图片
bridge = CvBridge() 
rospy.init_node('get_images', anonymous=True)
# 定义一个ros话题，把self.color_image发布出去
image_pub = rospy.Publisher('image_topic', Image, queue_size=10)

def get_image_frome_ros():   
    time_count=0
    while True:
        color_img = rospy.wait_for_message("/hs_camera/color/image_raw", Image, timeout=None)
        depth_img = rospy.wait_for_message("/hs_camera/aligned_depth_to_color/image_raw", Image, timeout=None)
        time_count = time_count +1
        if time_count>= 10:
            break
    color_image = bridge.imgmsg_to_cv2(color_img, 'bgr8')
    cv2.imwrite(f'IMG_3739.jpg',color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])

get_image_frome_ros()

frame=cv2.imread('IMG_3739.jpg')
#调整图片大小
frame=cv2.resize(frame,None,fx=0.5,fy=0.5,interpolation=cv2.INTER_CUBIC)
#灰度话
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#设置预定义的字典
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#使用默认值初始化检测器参数
parameters = cv2.aruco.DetectorParameters_create()
#使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
#画出标志位置
cv2.aruco.drawDetectedMarkers(frame, corners,ids)


cv2.imshow("frame",frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
