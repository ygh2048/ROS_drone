import cv2
import numpy as np

# 1. 读取深度图
depth = cv2.imread('image1.png', cv2.IMREAD_UNCHANGED)

# 确保深度图是单通道的
if len(depth.shape) == 3:
    depth_gray = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
else:
    depth_gray = depth



# 设置需要保留的灰度值范围
lower_bound = 2  # 设置下界
upper_bound = 100  # 设置上界

# 创建掩膜，保留在范围内的像素
mask = cv2.inRange(depth_gray, lower_bound, upper_bound)

# 通过掩膜保留图像中的像素
depth_gray = cv2.bitwise_and(depth_gray, depth_gray, mask=mask)


# 2. 图像去噪声（高斯模糊）
depth_gray = cv2.GaussianBlur(depth_gray, (5, 5), 0)
#depth_gray = cv2.medianBlur(depth_gray, ksize=5)  # 中值滤波


# 3. 直方图均衡化
depth_gray = cv2.equalizeHist(depth_gray)

# 3. 图像膨胀
kernel_ = np.ones((5, 5), np.uint8)

# 定义结构元素
kernel = np.ones((5, 5), np.uint8)  # 5x5 的矩形结构元素



# 4. 提取外轮廓
def filter_depth_image(low_threshold, high_threshold, center_weight=0.5):
    # 边缘检测
    edges = cv2.Canny(depth_gray, low_threshold, high_threshold)

    # 图像膨胀
    edges = cv2.dilate(edges, kernel_, iterations=1)

    # 应用腐蚀操作
    edges = cv2.erode(edges, kernel, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_area = 50  # 设定最小轮廓面积
    filtered_contours = [c for c in contours if cv2.contourArea(c) > min_area]

    height, width = depth_gray.shape
    center = (width // 2, height // 2)

    contour_scores = []
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
        area_weight = area / 70000  # 面积权重

        # 结合权重并归一化
        score = (center_weight * proximity_weight + (1 - center_weight) * area_weight)
        contour_scores.append((contour, score))

    # 根据得分排序
    contour_scores.sort(key=lambda x: x[1], reverse=True)
    top_contours = [contour for contour, score in contour_scores[:2]]

    mask = np.zeros_like(depth_gray)
    for top_contour in top_contours:
        cv2.drawContours(mask, [top_contour], -1, 255, thickness=cv2.FILLED)

    output_image = cv2.cvtColor(depth_gray, cv2.COLOR_GRAY2BGR)

    for contour in contours:
        cv2.drawContours(output_image, [contour], -1, (255, 255, 255), 1)

    for i, top_contour in enumerate(top_contours):
        cv2.drawContours(output_image, [top_contour], -1, (255, 0, 0), 2)

        # 计算外接圆
        (x, y), radius = cv2.minEnclosingCircle(top_contour)
        center = (int(x), int(y))
        radius = int(radius)

        # 绘制外接圆
        cv2.circle(output_image, center, radius, (0, 255, 0), 2)

    cv2.circle(output_image, center, 5, (0, 255, 255), -1)

    return output_image


# 创建窗口
cv2.namedWindow('Filtered Depth')

# 滑动条回调函数
def update(val):
    low_threshold = cv2.getTrackbarPos('Low Threshold', 'Filtered Depth')
    high_threshold = cv2.getTrackbarPos('High Threshold', 'Filtered Depth')
    center_weight = cv2.getTrackbarPos('Center Weight', 'Filtered Depth') / 100.0
    output_image = filter_depth_image(low_threshold, high_threshold, center_weight)
    cv2.imshow('Filtered Depth', output_image)

# 创建滑动条
cv2.createTrackbar('Low Threshold', 'Filtered Depth', 50, 255, update)
cv2.createTrackbar('High Threshold', 'Filtered Depth', 150, 255, update)
cv2.createTrackbar('Center Weight', 'Filtered Depth', 50, 100, update)

# 初始显示
update(0)

cv2.waitKey(0)
cv2.destroyAllWindows()
