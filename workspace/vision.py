'''
Date: 3.14.2024
Author: Shuai Gan
Copyright by Casia. Robotic Theory and Application Group
Description: Perception Module
             Input: image
             Output: coordinates of corner points in camera coordinate system
'''

import sys
from ultralytics import YOLO
import torch
import os
from scipy.spatial import distance
import numpy as np
import cv2
import os
import time
import struct
import pyrealsense2 as rs
from math import *
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32



class RealsenseD435(object):


    def __init__(self):
        self.im_height = 480
        self.im_width = 640
        self.model = YOLO('./models/train5/weights/best.pt')

    def get_data(self):
        # Return color image and depth image
        pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        # device.hardware_reset()
        # device_product_line = str(device.get_info(rs.camera_info.product_line))
        # print(f'相机的型号是: {device_product_line}')

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, self.im_width, self.im_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.im_width, self.im_height, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)  

        # 创建深度和颜色流的align对象  
        align_to = rs.stream.color  
        align = rs.align(align_to)  

        # 初始化一个计数器来跟踪时间  
        count = 0  

        # Streaming loop
        try:
            while True:
                # Get frameset of color and depth
                frames = pipeline.wait_for_frames()
                
                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                self.aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not self.aligned_depth_frame or not color_frame:
                    continue
                
                #### 获取相机参数 ####
                self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
                # print(self.depth_intrin)
                self.depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
                self.color_image = np.asanyarray(color_frame.get_data())

                # 如果已经过了五秒，才保存或处理图像  
                count += 1  
                if count >= 5:  # 忽略前五秒的数据  

                    # 将图像从BGR色彩空间转换为HSV色彩空间  
                    hue_factor = 1.05  # 增加色调系数的值，可以根据需要调整   
                    saturation_factor = 1.05 # 增加饱和度系数的值，可以根据需要调整  
                    value_factor = 1.05  # 增加亮度系数的值，可以根据需要调整  
                    hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)  

                    # 调整亮度系数，并将其应用于HSV图像的V通道（亮度）
                    hsv[:,:,0] = hsv[:,:,0] * hue_factor    
                    hsv[:,:,1] = hsv[:,:,1] * saturation_factor  
                    hsv[:,:,2] = hsv[:,:,2] * value_factor  

                    # 将图像从HSV色彩空间转换回BGR色彩空间  
                    self.color_image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)  

                    # 保存图像为 'xxx.jpg'，质量为96%的JPEG格式 
                    cv2.imwrite('save_images/raw_color.jpg', self.color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])
                    print('感认知：获取图像数据正常')
                    break         
        finally:
            pipeline.stop()

    def get_image_frome_ros(self):
        
        print("------------------------获取视觉图像------------------------------")
        self.bridge = CvBridge() 
        rospy.init_node('get_images', anonymous=True)
        time_count=0
        # 定义一个ros话题，把self.color_image发布出去
        self.image_pub = rospy.Publisher('image_topic', Image, queue_size=10)

        while True:
            color_img = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=None)
            depth_img = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=None)
            time_count = time_count +1
            if time_count>= 20:
                break

        self.color_image = self.bridge.imgmsg_to_cv2(color_img, 'bgr8')
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_img, '16UC1')
        cv2.imwrite('save_images/raw_color.jpg', self.color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])


    def rigid_transform_3D(self,A, B):

        assert len(A) == len(B)

        N = A.shape[0]  # total points
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        # centre the points
        AA = A - np.tile(centroid_A, (N, 1))
        BB = B - np.tile(centroid_B, (N, 1))

        H = np.matmul(np.transpose(AA),BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.matmul(Vt.T, U.T)

        # special reflection case
        if np.linalg.det(R) < 0:
            print("Reflection detected")
            Vt[2, :] *= -1
            R = np.matmul(Vt.T,U.T)

        t = -np.matmul(R, centroid_A) + centroid_B
        # err = B - np.matmul(A,R.T) - t.reshape([1, 3])
        return R, t

    def pose_model_output(self):
        results = self.model(self.color_image)
        for result in results:
            for box in result.boxes:
                m = torch.squeeze(box.xyxy.data)
                cv2.rectangle(self.color_image , (int(m[0]), int(m[1])), (int(m[2]), int(m[3])), (0, 0, 255), 2)

            # 在图上画点
            for keypoint in result.keypoints:
                m = torch.squeeze(keypoint.xy.data)

                point = np.array([int(m[0][0]), int(m[0][1]), int(m[1][0]), int(m[1][1]),int(m[2][0]), int(m[2][1]),int(m[3][0]), int(m[3][1])])
                cv2.circle(self.color_image, (point[0], point[1]), 2, (0, 255, 0), 2) # 绿色 
                cv2.circle(self.color_image, (point[2], point[3]), 2, (0, 0, 255), 2) # 红色 
                cv2.circle(self.color_image, (point[4], point[5]), 2, (255, 0, 0), 2) # 蓝色 
                cv2.circle(self.color_image, (point[6], point[7]), 2, (255, 255, 0), 2) # 青色 
                grasp_point = np.array([[point[0], point[1]],[point[2],point[3]],[point[4],point[5]],[point[6],point[7]]])     
                [x,y] = np.mean(grasp_point,axis=0)
                x = int(x);y = int(y)
                cv2.circle(self.color_image, (x, y), 2, [255, 255, 255], thickness=-1) # 白色

        # 保存图像为 'grasp_point.jpg'，质量为96%的JPEG格式  
        cv2.imwrite('save_images/grasp_point.jpg', self.color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 96])    

        ros_image = self.bridge.cv2_to_imgmsg(self.color_image, encoding="bgr8")
        self.image_pub.publish(ros_image)
               

        if len(grasp_point) == 0:
            return None
        else:
            print('感认知：角点检测正常')
            return grasp_point
 
    def vision_module_output(self):

        intrin = rs.pyrealsense2.intrinsics()
        intrin.fx = 606.941
        intrin.fy = 607.06
        intrin.ppx = 316.836
        intrin.ppy = 253.232

        start_time = time.time()

        while True:

            self.get_image_frome_ros() # 先获取图像数据
            # self.get_data()
            pixel_grasp_point = self.pose_model_output() # 角点像素坐标
            pixel_depth_point = np.array([self.depth_image[row[1]-1,row[0]-1] for row in pixel_grasp_point]) # 获取角点像素对应的深度 单位:mm  
            cam_grasp_point = np.ones((3,4))

            for i in range(4):
                cam_grasp_point[:,i] = np.array(rs.rs2_deproject_pixel_to_point(intrin, pixel_grasp_point[i,:], pixel_depth_point[i])).T

            if cam_grasp_point[2,0] !=0 and cam_grasp_point[2,1] !=0 and cam_grasp_point[2,2] !=0 and cam_grasp_point[2,3] !=0 :
                print(f'相机坐标系下四个角点的坐标为: \n {cam_grasp_point}')
                # break

            # 测试视觉模型效果使用
            end_time = time.time()
            if end_time - start_time >= 100:
                break

        print('感认知：角点输出正常')
        return cam_grasp_point

       
# 测试视觉模型效果使用      
if __name__=='__main__':

    realsenseD435 = RealsenseD435()
    cam_grasp_point = realsenseD435.vision_module_output()
    realsenseD435.get_image_frome_ros()