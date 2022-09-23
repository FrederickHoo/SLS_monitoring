import cv2
import numpy as np

# 生成aruco标记
# 加载预定义的字典
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# 生成标记
marker_image = np.zeros((200, 200), dtype=np.uint8)
for i in range(10):
    marker_image = cv2.aruco.drawMarker(dictionary, i, 200, marker_image, 1)
    fire_name = 'ArucoCode/' + str(i) + '.png'
    cv2.imwrite(fire_name, marker_image)
