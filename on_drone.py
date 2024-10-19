import cv2
import numpy as np
import time

# 初始化卡尔曼滤波器
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                      [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                     [0, 1, 0, 1],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.1
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1

circle_centers = []
last_output_time = time.time()

def is_unique_circle(center):
    for existing in circle_centers:
        if abs(existing[0] - center[0]) < 5 and abs(existing[1] - center[1]) < 5:
            return False
    return True

def calculate_angle(center, image_center):
    delta_x = center[0] - image_center[0]
    delta_y = -(center[1] - image_center[1])
    angle = np.arctan2(delta_y, delta_x) * (180 / np.pi)  # 转换为度
    return angle

def detect_circles_from_camera():
    global circle_centers, last_output_time

    cap = cv2.VideoCapture(0)
    kalman_state = np.zeros((4, 1), np.float32)

    while True:
        ret, cv_image = cap.read()
        if not ret:
            break
        cv_image = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_LINEAR)
        height, width, _ = cv_image.shape
        image_center = (width / 2, height / 2)

        # 直方图均衡化增强对比度
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.equalizeHist(gray_image)

        # 定义 HSV 范围并创建掩码
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([0, 17, 53])
        upper_blue = np.array([88, 141, 200])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 使用模糊处理减少噪声
        blurred = cv2.GaussianBlur(mask, (9, 9), 0)  # 加强模糊程度

        # 边缘检测
        edges = cv2.Canny(blurred, 100, 200)

        # 使用 HoughCircles 进行圆检测
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=500,
                                   param1=100, param2=30, minRadius=10, maxRadius=500)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                if is_unique_circle((i[0], i[1])):
                    circle_centers.append((i[0], i[1]))
                    # 更新卡尔曼滤波器
                    kalman.correct(np.array([[np.float32(i[0])], [np.float32(i[1])]]))
                    kalman_state = kalman.predict()

                # 画圆
                cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)

                # 计算角度并输出
                angle = calculate_angle((i[0], i[1]), image_center)

                # 线性设置 Vx 和 Vz
                # 假设图像宽度为640，您可以根据您的实际图像大小进行调整
                image_width = 640
                center_x = image_width / 2

                # 计算 Vy 和 Vz 为线性函数
                Vy = ((i[0] - center_x) / (image_width / 2) )/1.7  # 归一化到 [-1, 1]
                Vz = -Vy * np.tan(np.radians(angle))
                Vy = -Vy

                if Vy > 0.55:
                    Vy = 0.55
                if Vy < -0.55:
                    Vy = -0.55
                if Vz > 0.55:
                    Vz = 0.55
                if Vz < -0.55:
                    Vz = -0.55

                if i[2] > 205:
                    finshcv_flag = 2
                    print(f"finshcv_flag: {finshcv_flag}")
                # 每两秒输出一次圆心和角度
                if (time.time() - last_output_time) > 2:
                    print(f"Circle Center: ({i[0]}, {i[1]}), Angle: {angle:.2f} degrees, Vy: {Vy:.2f}, Vz: {Vz:.2f}")

                    last_output_time = time.time()

        # 用卡尔曼滤波器预测的结果更新图像
        predicted_center = kalman_state
        if predicted_center is not None:
            cv2.circle(cv_image, (int(predicted_center[0]), int(predicted_center[1])), 5, (255, 0, 0), -1)

        if len(circle_centers) >= 4 and (time.time() - last_output_time) > 2:
            avg_x = sum(x for x, y in circle_centers[-4:]) / 4
            avg_y = sum(y for x, y in circle_centers[-4:]) / 4
            print(f"Average Circle Center: ({avg_x}, {avg_y})")
            last_output_time = time.time()

        cv2.imshow('Video Feed', cv_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

detect_circles_from_camera()

