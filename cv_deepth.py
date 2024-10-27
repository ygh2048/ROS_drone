import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point, PoseArray
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API

class D435iDepthViewer:
    def __init__(self):
        # 初始化节点
        rospy.init_node('d435i_depth_viewer', anonymous=True)

        # 创建 CvBridge 对象
        self.bridge = CvBridge()

        # 订阅 D435i 的深度图像话题
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)

        # 创建一个发布者，用于发布轮廓位置
        self.contour_pub = rospy.Publisher('/contour_positions', PoseArray, queue_size=10)
        
    def depth_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # 将深度图像归一化为灰度图
            depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_gray = np.uint8(depth_image_normalized)

            # 获取滑动条参数
            low_threshold = 32
            high_threshold = 183
            center_weight = 45 / 100.0

            # 处理深度图像
            output_image, contours_info = self.filter_depth_image(depth_image_gray, low_threshold, high_threshold, center_weight)

            # 显示处理后的深度图像
            cv2.imshow("Filtered Depth", output_image)
            cv2.waitKey(1)  # 添加延迟以允许 OpenCV 更新窗口

            # 发布轮廓位置
            self.publish_contour_positions(contours_info)

            # 应用深度滤波器
            self.d435_filter(depth_image)

        except Exception as e:
            rospy.logerr("Could not convert depth image: %s", e)

    def filter_depth_image(self, depth_gray, low_threshold, high_threshold, center_weight=0.5):
        # 创建一个新图像，初始化为全零
        filtered_image = np.zeros_like(depth_gray)
        filtered_image[(depth_gray > 0) & (depth_gray <= 170)] = depth_gray[(depth_gray > 0) & (depth_gray <= 170)]
        depth_gray = filtered_image

        # 边缘检测
        edges = cv2.Canny(depth_gray, low_threshold, high_threshold)

        # 图像膨胀
        kernel_ = np.ones((6, 6), np.uint8)
        edges = cv2.dilate(edges, kernel_, iterations=1)

        # 应用腐蚀操作
        kernel = np.ones((5, 5), np.uint8)
        edges = cv2.erode(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 50  # 设定最小轮廓面积
        filtered_contours = [c for c in contours if cv2.contourArea(c) > min_area]

        height, width = depth_gray.shape
        center = (width // 2, height // 2)

        contour_scores = []
        contours_info = []  # 用于存储轮廓位置信息
        for contour in filtered_contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            area = cv2.contourArea(contour)
            distance = np.sqrt((cX - center[0]) ** 2 + (cY - center[1]) ** 2)
            max_distance = np.sqrt((width // 2) ** 2 + (height // 2) ** 2)

            proximity_weight = max(0, 1 - distance / max_distance)  # 接近度权重
            area_weight = area / 1500  # 面积权重

            # 结合权重并归一化
            score = (center_weight * proximity_weight + (1 - center_weight) * area_weight)
            contour_scores.append((contour, score))

            # 存储轮廓位置信息
            contours_info.append((cX, cY))

        # 根据得分排序
        contour_scores.sort(key=lambda x: x[1], reverse=True)
        top_contours = [contour for contour, score in contour_scores[:2]]

        output_image = cv2.cvtColor(depth_gray, cv2.COLOR_GRAY2BGR)

        for contour in contours:
            cv2.drawContours(output_image, [contour], -1, (255, 255, 255), 1)

        for top_contour in top_contours:
            cv2.drawContours(output_image, [top_contour], -1, (255, 0, 0), 2)

        cv2.circle(output_image, center, 5, (0, 255, 255), -1)

        return output_image, contours_info

    def publish_contour_positions(self, contours_info):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "camera_frame"  # 修改为你的相机坐标系

        for (cX, cY) in contours_info:
            point = Point()
            point.x = cX
            point.y = cY
            point.z = 0  # 假设深度为0，实际可以根据需求修改
            pose_array.poses.append(point)

        self.contour_pub.publish(pose_array)

if __name__ == '__main__':
    viewer = D435iDepthViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

s