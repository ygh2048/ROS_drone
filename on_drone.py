#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ctrl_msgs.msg import command
from functools import partial

frame = None

# def image_cb(msg, cv_bridge, detector_param, image_pub):
#     # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
#     try:
#         cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
#         # 将Opencv图像转换numpy数组形式，数据类型是uint8（0~255）
#         # numpy提供了大量的操作数组的函数，可以方便高效地进行图像处理
#         frame = np.array(cv_image, dtype=np.uint8)
#     except (CvBridgeError, e):
#         print(e)


# 初始化卡尔曼滤波器
kalman = cv2.KalmanFilter(4, 2)
circle_centers = []
last_output_time = time.time()
bridge = None
image_pub = None
velocity_pub = None


# def is_unique_circle(center):
#     for existing in circle_centers:
#         if abs(existing[0] - center[0]) < 5 and abs(existing[1] - center[1]) < 5:
#             return False
#     return True

def calculate_angle(center, image_center):
    delta_x = center[0] - image_center[0]
    delta_y = -(center[1] - image_center[1])
    angle = np.arctan2(delta_y, delta_x) * (180 / np.pi)  # 转换为度
    return angle

def circle():
    return

def ranctangle():
    return
    
def detect_circles_from_camera():
    global circle_centers, last_output_time
    global bridge, image_pub, velocity_pub
    bingo = 0
    cap = cv2.VideoCapture(0)

    kalman_state = np.zeros((4, 1), np.float32)

    if not cap.isOpened():
        print("Error: Unable to open video capture")
        return
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() or bingo <= 20:
        ret, cv_image = cap.read()
        if not ret:
            break
     
        cv_image = cv2.resize(cv_image, (640, 360), interpolation=cv2.INTER_LINEAR)
        height, width, _ = cv_image.shape
        image_center = (width / 2, height / 2)

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.equalizeHist(gray_image)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        blurred = cv2.GaussianBlur(mask, (9, 9), 0)
        edges = cv2.Canny(blurred, 100, 200)
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=500, param1=100, param2=30, minRadius=10,
                                   maxRadius=500)
        true_cricle = [0, 0]  # x,y
        is_found = False
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # TODO:在这个for循环里面要过滤出正确的圆，这里方便调试只是把数组第一个圆当成结果直接break了
                cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)
                true_cricle = [i[0] - image_center[0], image_center[1] - i[1]]
                break
                # TODO:在这个for循环里面要过滤出正确的圆，这里方便调试只是把数组第一个圆当成结果直接break了

        velocity_msg = command()
        # TODO:velocity_msg.is_found不在原先的message里面，建议加一个这样其他节点知道你到底找没找到圆，如果启用要在上面恰当的位置把is_found设置成true
        # velocity_msg.is_found=is_found

        # TODO:如果要做滤波什么的就写在下面
        # 滤波

        # 下面这个是vy为+—0.5，vz按比例缩放的版本
        # velocity_msg.vy = 0.5 * true_cricle[0]/np.abs(true_cricle[0])
        # velocity_msg.vz = velocity_msg.vy/true_cricle[0]*true_cricle[1]
        # 计算角度并输出
        angle = calculate_angle((i[0], i[1]), image_center)

        # 线性设置 Vx 和 Vz
        # 假设图像宽度为640，您可以根据您的实际图像大小进行调整
        image_width = 640
        center_x = image_width / 2

        # 计算 Vy 和 Vz 为线性函数
        Vy = ((i[0] - center_x) / (image_width / 2) ) * 0.7  # 归一化到 [-0.6, 0.6]
        Vz = -Vy * np.tan(np.radians(angle))
        Vy = -Vy

        Vy = max(min(Vy, 0.55), -0.55)
        Vz = max(min(Vz, 0.55), -0.55)#限定VyVz范围
        velocity_msg.vy = Vy
        velocity_msg.vz = Vz

        rospy.loginfo(f"vy: {velocity_msg.vy}, i[0]: {i[0]}")

        # 下面这个是圆心的坐标，图像中心是原点，向右x正，向上y正
        # velocity_msg.vy=true_cricle[0]
        # velocity_msg.vz=true_cricle[1]

        # 下面是用来判断有没有对准的，以及防抖，100是范围，上面while的or判定的时候20是一秒，0.05秒bingo加1
        if np.abs(true_cricle[0]) < 100 and np.abs(true_cricle[1]) < 40:
            bingo = bingo + 1
        else:
            bingo = 0

        if bingo >= 7:
            velocity_msg.Finishcv_flag = 1

        if i[2] > 250 
            velocity_msg.Finishcv_flag = 2 #即将穿越

        velocity_pub.publish(velocity_msg)
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        #rate.sleep()

    # 这下面是完成对准后最后执行的代码
    rospy.loginfo("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
def detect_rectangles_from_camera():
    pass
     

def CV_flag_cb(msg):
    global CV_flag
    rospy.loginfo("Received data: %d ",msg.CV_flag)
    CV_flag = msg.CV_flag


def main():
    global bridge, image_pub, velocity_pub
    rospy.init_node("circle")
    rospy.loginfo("starting circle node")

    kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                         [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                        [0, 1, 0, 1],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.1
    kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1

    # rospy.loginfo("command Vy %f",command.Vy)

    bridge = CvBridge()
    image_pub = rospy.Publisher("/cv_bridge_image", Image, queue_size=1)
    velocity_pub = rospy.Publisher("task/cv_task", command, queue_size=1)  # Use custom message
    rospy.Subscriber("task/task_pub", command, CV_flag_cb)

    if CV_flag == 1:
        detect_circles_from_camera()
    elif CV_flag == 2:
        detect_rectangles_from_camera()
    # bind_image_cb = partial(image_cb, cv_bridge=bridge, velocity_pub=velocity_pub, image_pub=image_pub)
    # rospy.Subscriber("/usb_cam/image_raw", Image, bind_image_cb)

    rospy.spin()


if __name__ == "__main__":
    main()
