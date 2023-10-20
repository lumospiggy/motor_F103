import serial
import json
import cv2
import numpy as np
##################比例系数k##################
# 打开摄像头
cap = cv2.VideoCapture(0)
#云台画面修正y
delta_y0_virtual=0
#云台实际修正y/cm
delta_y0_real=8
#云台实际高
h0_real=22
#矩形实际宽0
h_rael=15.5
#矩形画面宽
h_camera=147
#计算实际 图像 比
#得到画面云台高 y0误差修正
k=h_camera/h_rael
h0 = k*h0_real
delta_y0_virtual =k*delta_y0_real
print("h0",h0)
print("delta_y0_virtual",delta_y0_virtual)
#################比例系数k########################
"""
while True:
    # 读取视频流的帧
    ret, frame = cap.read()

    # 转换为灰度图像
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 进行边缘检测
    edges = cv2.Canny(gray, 50, 150)

    # 找到轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 遍历轮廓
    for contour in contours:
        # 计算轮廓的最小外接矩形
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # 绘制外接矩形
        cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

        # 计算长宽坐标
        x, y, w, h = cv2.boundingRect(contour)
        h_camera=h

        # 打印长宽坐标
        print("长宽坐标：", x, y, w, h)

    # 显示视频流
    cv2.imshow('Video', frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头和窗口
cap.release()
cv2.destroyAllWindows()
"""

##################################
# 读取颜色范围JSON文件
def read_color_range(file_path):
    with open(file_path, 'r') as json_file:
        color_range = json.load(json_file)
    return color_range
class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def set(self, x, y):
        self.x = x
        self.y = y

ser = serial.Serial('COM6', 115200)
target = Coordinate(0, 0)
current = Coordinate(0, 0)

# 定义面积阈值
area_threshold = 50
# 打开摄像头
cap = cv2.VideoCapture(0)

#设置参数
# 文件路径
laser_file_path = 'laser_range_hsv.json'
led_file_path = 'led_range_hsv.json'
# 从颜色范围字典中提取lower_laser和upper_laser的值
laser_range = read_color_range(laser_file_path)
lower_laser = np.array(laser_range["lower_laser"])
upper_laser = np.array(laser_range["upper_laser"])
print(lower_laser,upper_laser)
# 从颜色范围字典中提取lower_led和upper_led的值
led_range = read_color_range(led_file_path)
lower_led = np.array(led_range["lower_laser"])
upper_led = np.array(led_range["upper_laser"])
print(lower_led,upper_led)
############################
def culculate( x0,y0,h,x_l ,y_l ,x_t ,y_t ):
    import math
    x0 = x0
    y0 = -y0+480
    h = h
    x_l = x_l
    y_l = -y_l+480
    x_t = x_t
    y_t = -y_t+480
    #camera = (x0, y0)
    #target = (x_t, y_t)
    #current = (x_l, y_l)
    theta_rad = math.atan((x_t - x0) / (y_t - y0)) - math.atan((x_l - x0) / (y_l - y0))
    print("(x_t - x0)",(x_t - x0))
    print("(y_t - y0)",(y_t - y0))
    print("(x_l - x0)",(x_l - x0))
    print("(y_l - y0)",(y_l - y0))
    theta_deg = math.degrees(theta_rad)
    print(theta_deg)
    r1 = math.sqrt((x_t - x0) ** 2 + (y_t - y0) ** 2)
    r2 = math.sqrt((x_l - x0) ** 2 + (y_l - y0) ** 2)
    fey_rad = math.atan(r1 / h) - math.atan(r2 / h)
    fey_deg = math.degrees(fey_rad)
    print(fey_deg)
    return theta_deg,fey_deg

#最大x值： 640
#最大y值： 480
count=0
while True:
    # 读取摄像头帧
    ret, frame = cap.read()

    # 将图像转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 根据颜色范围创建掩膜
    mask_led = cv2.inRange(hsv, lower_led, upper_led)
    mask_laser = cv2.inRange(hsv, lower_laser, upper_laser)

    # 寻找绿色LED的轮廓
    contours_led, _ = cv2.findContours(mask_led, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 寻找红色LED的轮廓
    contours_laser, _ = cv2.findContours(mask_laser, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour_led in contours_led:
        area_led = cv2.contourArea(contour_led)
        #print("I am in LED loop")

        if area_led > area_threshold:
            x1, y1, w1, h1 = cv2.boundingRect(contour_led)
            target.set(x1 + w1 / 2, y1 + h1 / 2)
            region = frame[y1:y1 + h1, x1:x1 + w1]
            cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
            #print(cv2.mean(region))
            cv2.putText(frame, 'LED', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    for contour_laser in contours_laser:
        area_laser = cv2.contourArea(contour_laser)
        #print("I am in laser loop")

        if area_laser > area_threshold:
            x2, y2, w2, h2 = cv2.boundingRect(contour_laser)
            current.set(x2 + w2 / 2, y2 + h2 / 2)
            region = frame[y2:y2 + h2, x2:x2 + w2]
            cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 2)
            #print(cv2.mean(region))
            cv2.putText(frame, 'LASER', (x2, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # 显示图像
    cv2.imshow('original',frame)
    cv2.imshow('mask laser', mask_laser)
    cv2.imshow('mask led', mask_led)
    cv2.imshow('hsv', hsv)

    count=count+1
    print(count)
    if count==300:
        delta_x, delta_y = culculate(320, 480 + delta_y0_virtual, h=h0, x_l=current.x, y_l=current.y, x_t=target.x,
                                     y_t=target.y)
        # 将角度值转换为字符串
        angle_str = "{:.1f},{:.1f}".format(delta_x, delta_y)
        print("发送中：","{.1f},{.1f}".format(delta_x, delta_y))
        # 将字符串转换为字节流
        message = angle_str.encode('utf-8')
        # 发送字节流到串口
        ser.write(message)
        print("发送完毕！")
    # 按下Esc键退出程序
    if cv2.waitKey(1) == 27:
        break

# 释放摄像头和窗口
ser.close()
cap.release()
cv2.destroyAllWindows()