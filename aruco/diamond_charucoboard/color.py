import cv2
import numpy as np

# 加载黑白图像
img = cv2.imread('test_charucoboarddiamond.jpg', cv2.IMREAD_GRAYSCALE)

# 将黑白图像转换为彩色图像
img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# 创建一个与原图像同样大小的粉红色图像
pink_img = np.full_like(img_color, (255, 105, 180))  # BGR for pink

# 将彩色图像与粉红色图像混合
result = cv2.addWeighted(img_color, 0.5, pink_img, 0.5, 0)

cv2.imwrite("color.jpg", result)

# 显示结果图像
cv2.imshow('Result', result)
cv2.waitKey(0)
cv2.destroyAllWindows()