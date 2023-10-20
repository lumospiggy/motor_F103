import cv2
import numpy as np
import json

# 定义面积阈值
area_threshold = 10

def empty():
    pass

# 打开摄像头
cap = cv2.VideoCapture(0)
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 240)
h_min = cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
h_max = cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
s_min = cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, empty)
s_max = cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
v_min = cv2.createTrackbar("Val Min", "TrackBars", 0, 255, empty)
v_max = cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

while True:
    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    # 将颜色范围保存为字典
    color_range = {
        "lower_laser": lower.tolist(),
        "upper_laser": upper.tolist()
    }

    # 保存为JSON文件
    with open('laser_range_hsv.json', 'w') as json_file:
        json.dump(color_range, json_file)

    # 读取摄像头帧
    ret, frame = cap.read()
    # 将图像转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 根据颜色范围创建掩膜
    mask = cv2.inRange(hsv, lower, upper)

    # 寻找红色LED的轮廓
    contours_red, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 遍历红色LED轮廓并绘制矩形框
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if area > area_threshold:
            x, y, w, h = cv2.boundingRect(contour)
            region = frame[y:y+h, x:x+w]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            print(cv2.mean(region))
            cv2.putText(frame, 'Red LED', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    # 显示图像
    cv2.imshow('original', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('hsv', hsv)

    # 按下Esc键退出程序
    if cv2.waitKey(1) == 27:
        break

# 释放摄像头和窗口
cap.release()
cv2.destroyAllWindows()
