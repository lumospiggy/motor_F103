import serial
import json
import cv2
import numpy as np
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
current = Coordinate(2.1, 1.3)

# 定义面积阈值
area_threshold = 20
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
        if area_led>10:
            x1, y1, w1, h1 = cv2.boundingRect(contour_led)
            target.set(x1 + w1 / 2, y1 +h1/2)
            region = frame[y1:y1 + h1, x1:x1 + w1]
            cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
            print(cv2.mean(region))
            cv2.putText(frame, 'LED', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    for contour_laser in contours_laser:
        area_laser = cv2.contourArea(contour_laser)
        #print("I am in laser loop")

        if area_laser> 200:
            x2, y2, w2, h2 = cv2.boundingRect(contour_laser)
            current.set(x2 + 0.5*w2 , y2 + 0.5*h2)
            region = frame[y2:y2 + h2, x2:x2 + w2]
            cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 2)
            #print(cv2.mean(region))
            cv2.putText(frame, 'LASER', (x2, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # 显示图像
    cv2.imshow('original',frame)
    cv2.imshow('mask laser', mask_laser)
    cv2.imshow('mask led', mask_led)

    delta_x=target.x-current.x
    delta_y=-target.y+current.y
    '''
    if delta_x<50:
        delta_x=0
    if delta_y<50:
        delta_y=0
    '''

    print("发送中...")

    message = "{:.1f},{:.1f}".format(delta_x, delta_y).encode('utf-8')
    ser.write(message)
    print("已发送:",message)
    print(delta_x,delta_y)

    # 按下Esc键退出程序
    if cv2.waitKey(1) == ord('q'):
        break

# 释放摄像头和窗口
ser.close()
cap.release()
cv2.destroyAllWindows()