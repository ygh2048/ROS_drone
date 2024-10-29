#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 添加多线程，发布和订阅

import numpy as np
import rospy
import cv2
import threading
from ctrl_msgs.msg import command

kernel = np.ones((5, 5), np.uint8)
frame = None
frame_lock = threading.Lock()
velocity_pub = None
cnt = 0
body_center = (640, 320)

def calculate_angle(center, new_center):
    new_center = (640, 320)  # 示例定义
    delta_x = center[0] - new_center[0]
    delta_y = -(center[1] - new_center[1])
    angle = np.arctan2(delta_y, delta_x) * (180 / np.pi)  # 转换为度
    return angle


# 目标中心（x1,y1），图像中心（c1,c2）
def calculate_velocity(x1, y1, c1, c2, speed):
    # 计算目标中心与图像中心的偏差
    delta_x = c1 - x1
    delta_y = c2 - y1
    
    # 计算角度
    angle = np.arctan2(delta_y, delta_x)  # 计算与 x 轴的角度

    # 计算速度分量
    vy = np.cos(angle) * speed  # x 方向的速度分量
    vz = np.sin(angle) * speed  # y 方向的速度分量
    
    return vy, vz


def detect_red_blue_circles(frame):
    flag = 0
    finishcv_flag = 0
    # 将帧调整为 HSV 色彩空间
    #frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 定义红色和蓝色的 HSV 范围
    lower_red2 = np.array([155, 70, 70])
    upper_red2 = np.array([180, 255, 255])
    lower_blue = np.array([91, 38, 175])
    upper_blue = np.array([121, 111, 255])

    # 创建掩模
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # 膨胀操作
    kernel = np.ones((5, 5), np.uint8)  # 添加膨胀核
    mask_red_dilated = cv2.dilate(mask_red2, kernel, iterations=5)
    mask_blue_dilated = cv2.dilate(mask_blue, kernel, iterations=5)

    contours_red, _ = cv2.findContours(mask_red_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    center_red = None
    center_blue = None

    if contours_red:
        largest_red = max(contours_red, key=cv2.contourArea)
        if cv2.contourArea(largest_red) > 500:
            x, y, w, h = cv2.boundingRect(largest_red)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_red = (x + w // 2, y + h // 2)
            cv2.circle(frame, center_red, 5, (0, 255, 0), -1)

    if contours_blue:
        largest_blue = max(contours_blue, key=cv2.contourArea)
        if cv2.contourArea(largest_blue) > 500:
            x, y, w, h = cv2.boundingRect(largest_blue)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            center_blue = (x + w // 2, y + h // 2)
            cv2.circle(frame, center_blue, 5, (255, 0, 0), -1)


    # 如果检测到红色和蓝色圆圈
    if center_red and center_blue:
        body_center = ((center_red[0] + center_blue[0]) // 2, (center_red[1] + center_blue[1]) // 2 -25)

        cv2.line(frame, center_red, center_blue, (255, 255, 255), 2)
        cv2.circle(frame, body_center, 5, (255, 255, 255), -1)

        # 检查连线的长度
        width = frame.shape[1]
        distance = np.linalg.norm(np.array(center_red) - np.array(center_blue))
        if distance > (width *  3/ 4):
            # 在左上角绘制红色小圆圈
            cv2.circle(frame, (20, 20), 5, (0, 0, 255), -1)
            finishcv_flag = 2
        # 计算图像中心
        #image_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        # 计算图像中心与 body_center 之间的距离
        if body_center is not None:
    # 将 image_center 替换为确定的坐标 (320, 150)
            new_center = (640, 320)
            dist_to_body_center = np.linalg.norm(np.array(new_center) - np.array(body_center))

    

            # 更新 bingo 值
            if dist_to_body_center < 45:
                flag += 1
            else:
                flag = 0

        

    return frame, body_center, flag, finishcv_flag

def CV_flag_cb(msg):
    global CV_flag
    rospy.loginfo("Received data: %d", msg.CV_flag)
    CV_flag = msg.CV_flag

class CameraViewer:
    def __init__(self):
        rospy.init_node('camera_viewer', anonymous=True)
        self.cap = cv2.VideoCapture(6)  # 打开指定摄像头
        self.running = True

    def capture_frame(self):
        global frame
        while self.running:
            ret, captured_frame = self.cap.read()
            if ret:
                with frame_lock:
                    frame = captured_frame
            else:
                rospy.logerr("Failed to capture image")

    def process_frame(self):
        global frame,cnt,body_center
        finishcv_flag = 0
        rate = rospy.Rate(8)  # 设置发布频率为20Hz
        while self.running:
            with frame_lock:
                if frame is not None:
                    # 在这里添加图像处理逻辑
                    #processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像
                    frame , body_center ,flag ,fflag = detect_red_blue_circles(frame)

                    finishcv_flag = fflag

                    # 更新 flag 值
                    if flag == 1:
                        cnt += 1
                    else:
                        cnt = 0

                    # 检查 flag 值
                    if cnt > 4:
                        finishcv_flag = 1

                    if fflag == 2:
                        finishcv_flag = 2



                    # 发布消息
                    velocity_msg = command()
                    
                    new_center = (640, 320)

                    if body_center != (0, 0):
                        Vy, Vz = calculate_velocity(body_center[0], body_center[1], new_center[0], new_center[1], 0.45)

                        # 限制速度
                        Vy = np.clip(Vy, -0.38, 0.38)
                        Vz = np.clip(Vz, -0.28, 0.28)

                        velocity_msg.vy = -Vy
                        velocity_msg.vz = Vz
                        velocity_msg.Finishcv_flag = finishcv_flag

                        rospy.loginfo(f"vy: {velocity_msg.vy}, vz: {velocity_msg.vz}, x: {body_center[0]}, y: {body_center[1]}")

                        # 绘制速度矢量
                        arrow_start = (int(body_center[0]), int(body_center[1]))  # 目标中心
                        arrow_end = (int(new_center[0]), int(new_center[1]))  # 速度矢量的终点

                        # 在图像上绘制箭头
                        cv2.arrowedLine(frame, arrow_start, arrow_end, (0, 255, 0), 2, tipLength=0.1)

                        # 标注速度值
                        speed_text = f"Speed (Vy, Vz): ({Vy:.2f}, {Vz:.2f})"
                        cv2.putText(frame, speed_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    # 显示图像
                    cv2.imshow("Camera Feed", frame)
                    velocity_pub.publish(velocity_msg)  # 发布速度消息
            rate.sleep()  # 控制循环频率
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 添加退出条件
                break

    def run(self):
        global velocity_pub
        velocity_pub = rospy.Publisher("task/cv_task", command, queue_size=1)
        rospy.Subscriber("task/task_pub", command, CV_flag_cb)

        capture_thread = threading.Thread(target=self.capture_frame)
        capture_thread.start()

        # 在主线程中处理帧
        self.process_frame()

        # 等待线程结束
        capture_thread.join()

    def cleanup(self):
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    viewer = CameraViewer()
    try:
        viewer.run()
    except rospy.ROSInterruptException:
        viewer.cleanup()