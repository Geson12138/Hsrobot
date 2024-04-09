from ultralytics import YOLO
import torch
import cv2
import os

# model = YOLO('./runs/pose/train3/weights/best.pt')

# # results = model('/home/lpc/图片/dataset/f/image_1092_3.jpg')  # predict on an image
# results = model("./datasets/cable200-pose/images/val/image_5.jpg")
# img = cv2.imread("./datasets/cable200-pose/images/val/image_5.jpg")
# source = "./datasets/cable200-pose/images/val"
# img_list = os.listdir(source)
# # img = cv2.resize(img)
# for img in img_list:
#     results = model(os.path.join(source, img))
#     img = cv2.imread(os.path.join(source, img))
#     for result in results:
#         # img = result.orig_img
#         for box in result.boxes:
#             m = torch.squeeze(box.xyxy.data)
#             cv2.rectangle(img, (int(m[0]), int(m[1])), (int(m[2]), int(m[3])), (0, 0, 255), 2)
#         # 在图上画点
#         for keypoint in result.keypoints:
#             m = torch.squeeze(keypoint.xy.data)
#             cv2.circle(img, (int(m[0][0]), int(m[0][1])), 2, (0, 0, 255), 2)
#             cv2.circle(img, (int(m[1][0]), int(m[1][1])), 2, (0, 0, 255), 2)
#     cv2.imshow("result", img)
#     cv2.waitKey(5000)
#     cv2.destroyWindow("result")
# # cv2.imwrite("result.jpg", img)

from scipy.spatial import distance
import numpy as np
def pose_model_output(model, image):
    results = model(image)
    for result in results:
        point_angle = []
        for box in result.boxes:
            m = torch.squeeze(box.xyxy.data)
            cv2.rectangle(image, (int(m[0]), int(m[1])), (int(m[2]), int(m[3])), (0, 0, 255), 2)
        # 在图上画点
        for keypoint in result.keypoints:
            m = torch.squeeze(keypoint.xy.data)
            point = [int(m[0][0]), int(m[0][1]), int(m[1][0]), int(m[1][1])]
            cv2.circle(image, (int(m[0][0]), int(m[0][1])), 2, (0, 255, 0), 2)
            cv2.circle(image, (int(m[1][0]), int(m[1][1])), 2, (0, 0, 255), 2)
            v1 = [point[0]-point[2], point[1]-point[3]]
            v2 = [1, 0]
            x = 0.5 * (point[0] + point[2])
            y = 0.5 * (point[1] + point[3])
            cos_dist = distance.cosine(v1, v2)
            angle = np.arccos(1 - cos_dist)
            
            point_angle.append([x, y, angle])
    if len(point_angle) == 0:
        return None 
    return point_angle
