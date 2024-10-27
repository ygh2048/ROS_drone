#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import threading
from ctrl_msgs.msg import command

frame = None
frame_lock = threading.Lock()
velocity_pub = None
CV_flag = 0

def CV_flag_cb(msg):
    global CV_flag
    rospy.loginfo("Received data: %d ", msg.CV_flag)
    CV_flag = msg.CV_flag

class CameraViewer:
    def __init__(self):
        rospy.init_node('camera_viewer', anonymous=True)
        self.cap = cv2.VideoCapture(0)  # 打开指定摄像头
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
        global frame
        rate = rospy.Rate(20)  # 设置发布频率为20Hz
        while self.running:
            with frame_lock:
                if frame is not None:
                    # 在这里添加图像处理逻辑
                    # 例如：将图像转换为灰度
                    processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    
                    # 发布消息
                    velocity_msg = command()
                    velocity_msg.vy = 0
                    velocity_msg.vz = 0
                    cv2.imshow("Camera Feed", processed_frame)  # 显示图像
                    velocity_pub.publish(velocity_msg)

            rate.sleep()  # 控制循环频率

    def run(self):
        global velocity_pub
        velocity_pub = rospy.Publisher("task/cv_task", command, queue_size=1)
        rospy.Subscriber("task/task_pub", command, CV_flag_cb)

        capture_thread = threading.Thread(target=self.capture_frame)
        process_thread = threading.Thread(target=self.process_frame)

        capture_thread.start()
        process_thread.start()

        # 等待线程结束
        capture_thread.join()
        process_thread.join()

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
