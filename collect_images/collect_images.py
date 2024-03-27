import pyrealsense2 as rs
import numpy as np
import cv2
import time
import threading

class VideoThread(threading.Thread):
    def __init__(self, pipeline):
        super(VideoThread, self).__init__()
        self.pipeline = pipeline

    def run(self):
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                color_image = np.asanyarray(color_frame.get_data())
                cv2.imshow('RealSense', color_image)
                cv2.waitKey(1)
        finally:
            cv2.destroyAllWindows()

class SaveImageThread(threading.Thread):
    def __init__(self, pipeline):
        super(SaveImageThread, self).__init__()
        self.pipeline = pipeline
        self.i = 501

    def run(self):

        time.sleep(1)
        try:
            while self.i <= 600:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                color_image = np.asanyarray(color_frame.get_data())
                # 显示图片
                print('save image: {}'.format(self.i))
                # 保存图片
                cv2.imwrite('./cable_0317/image_{}.jpg'.format(self.i), color_image)
                self.i += 1
                time.sleep(0.5)
        finally:
            self.pipeline.stop()

# 创建管道1
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

pipeline1 = pipeline

# 在主线程中创建并启动新的视频处理线程和保存图片的线程
video_thread = VideoThread(pipeline1)
save_image_thread = SaveImageThread(pipeline)
video_thread.start()
save_image_thread.start()
