# OpenCV 视觉处理指南

## 目录

1. [安装 OpenCV](#安装-opencv)
2. [图像基础操作](#图像基础操作)
3. [颜色空间转换](#颜色空间转换)
4. [颜色检测](#颜色检测)
5. [轮廓检测](#轮廓检测)
6. [形状识别](#形状识别)
7. [相机标定](#相机标定)

---

## 安装 OpenCV

```bash
pip install opencv-python
pip install opencv-python-headless  # 无 GUI 版本
pip install opencv-contrib-python   # 包含额外模块
```

---

## 图像基础操作

### 读取和显示图像

```python
import cv2

# 读取图像
image = cv2.imread("image.jpg")

# 转换为灰度图
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 显示图像
cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 保存图像
cv2.imwrite("output.jpg", image)
```

### 摄像头采集

```python
import cv2

# 打开摄像头
cap = cv2.VideoCapture(0)  # 0 表示第一个摄像头

while True:
    # 读取帧
    ret, frame = cap.read()

    if not ret:
        break

    # 显示帧
    cv2.imshow("Camera", frame)

    # 按 'q' 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### 图像属性

```python
# 获取图像尺寸
height, width = image.shape[:2]
print(f"尺寸: {width}x{height}")

# 获取通道数
channels = image.shape[2] if len(image.shape) == 3 else 1
print(f"通道数: {channels}")

# 裁剪图像
cropped = image[y:y+h, x:x+w]

# 调整大小
resized = cv2.resize(image, (640, 480))
```

---

## 颜色空间转换

### BGR 到 HSV

```python
# BGR 转 HSV
hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

# HSV 范围
# H (Hue): 0-180 (色调)
# S (Saturation): 0-255 (饱和度)
# V (Value): 0-255 (明度)
```

### 常见颜色 HSV 范围

```python
# 红色 (两个范围)
red_lower1 = (0, 120, 70)
red_upper1 = (10, 255, 255)
red_lower2 = (170, 120, 70)
red_upper2 = (180, 255, 255)

# 绿色
green_lower = (40, 50, 50)
green_upper = (80, 255, 255)

# 蓝色
blue_lower = (100, 150, 0)
blue_upper = (130, 255, 255)

# 黄色
yellow_lower = (20, 100, 100)
yellow_upper = (40, 255, 255)
```

### 其他颜色空间

```python
# BGR 到灰度
gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

# BGR 到 RGB
rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

# BGR 到 LAB
lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
```

---

## 颜色检测

### 基本颜色检测

```python
import cv2
import numpy as np

# 读取图像
image = cv2.imread("image.jpg")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 定义颜色范围
lower_color = np.array([0, 120, 70])
upper_color = np.array([10, 255, 255])

# 创建掩码
mask = cv2.inRange(hsv, lower_color, upper_color)

# 应用掩码
result = cv2.bitwise_and(image, image, mask=mask)

# 显示结果
cv2.imshow("Original", image)
cv2.imshow("Mask", mask)
cv2.imshow("Result", result)
cv2.waitKey(0)
```

### 形态学操作（去噪）

```python
kernel = np.ones((5, 5), np.uint8)

# 开运算（去除小噪点）
mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# 闭运算（填充小孔洞）
mask_close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# 腐蚀
mask_erode = cv2.erode(mask, kernel, iterations=1)

# 膨胀
mask_dilate = cv2.dilate(mask, kernel, iterations=1)
```

---

## 轮廓检测

### 查找轮廓

```python
import cv2
import numpy as np

# 转换为灰度图
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 二值化
_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# 查找轮廓
contours, hierarchy = cv2.findContours(
    binary,
    cv2.RETR_EXTERNAL,  # 只检测外轮廓
    cv2.CHAIN_APPROX_SIMPLE  # 压缩轮廓
)

print(f"找到 {len(contours)} 个轮廓")
```

### 绘制轮廓

```python
# 绘制所有轮廓
cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

# 绘制单个轮廓
cv2.drawContours(image, [contours[0]], 0, (0, 0, 255), 3)
```

### 轮廓特征

```python
for contour in contours:
    # 面积
    area = cv2.contourArea(contour)

    # 周长
    perimeter = cv2.arcLength(contour, True)

    # 边界框
    x, y, w, h = cv2.boundingRect(contour)

    # 最小外接矩形
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # 最小外接圆
    (x, y), radius = cv2.minEnclosingCircle(contour)

    # 凸包
    hull = cv2.convexHull(contour)

    # 轮廓中心（矩）
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
```

### 筛选轮廓

```python
# 按面积筛选
min_area = 500
max_area = 50000

filtered = [
    c for c in contours
    if min_area < cv2.contourArea(c) < max_area
]

# 按面积排序
sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

# 获取最大轮廓
largest = max(contours, key=cv2.contourArea)
```

---

## 形状识别

### 多边形近似

```python
# 计算轮廓周长
peri = cv2.arcLength(contour, True)

# 多边形近似
epsilon = 0.04 * peri
approx = cv2.approxPolyDP(contour, epsilon, True)

# 顶点数量
num_vertices = len(approx)

print(f"顶点数: {num_vertices}")
```

### 形状分类

```python
def detect_shape(contour):
    """检测轮廓形状"""
    # 多边形近似
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

    # 计算顶点数
    vertices = len(approx)

    # 计算圆形度
    area = cv2.contourArea(contour)
    circularity = 4 * np.pi * area / (peri * peri) if peri > 0 else 0

    # 形状判断
    if vertices == 3:
        return "triangle"
    elif vertices == 4:
        # 检查是否为正方形
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        if 0.95 <= aspect_ratio <= 1.05:
            return "square"
        else:
            return "rectangle"
    elif vertices == 5:
        return "pentagon"
    elif circularity > 0.85:
        return "circle"
    else:
        return "unknown"
```

### 圆形检测（霍夫变换）

```python
# 转换为灰度图
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 高斯模糊
blurred = cv2.GaussianBlur(gray, (9, 9), 2)

# 霍夫圆检测
circles = cv2.HoughCircles(
    blurred,
    cv2.HOUGH_GRADIENT,
    dp=1.2,
    minDist=100,
    param1=50,
    param2=30,
    minRadius=20,
    maxRadius=100
)

if circles is not None:
    circles = np.round(circles[0, :]).astype("int")

    for (x, y, r) in circles:
        # 绘制圆
        cv2.circle(image, (x, y), r, (0, 255, 0), 2)
        # 绘制圆心
        cv2.circle(image, (x, y), 3, (0, 0, 255), -1)
```

---

## 相机标定

### 棋盘格标定

```python
import cv2
import numpy as np

# 标定板参数
chessboard_size = (9, 6)  # 内角点数量
square_size = 25  # 方格大小（mm）

# 准备标定点
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[
    0:chessboard_size[0],
    0:chessboard_size[1]
].T.reshape(-1, 2) * square_size

obj_points = []  # 3D 世界坐标
img_points = []  # 2D 图像坐标

# 采集标定图像
cap = cv2.VideoCapture(0)

while len(obj_points) < 20:
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(
        gray,
        chessboard_size,
        None
    )

    if ret:
        obj_points.append(objp)
        img_points.append(corners)

        # 绘制角点
        cv2.drawChessboardCorners(frame, chessboard_size, corners, ret)
        cv2.putText(frame, f"Captured: {len(obj_points)}/20",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                   1, (0, 255, 0), 2)

    cv2.imshow("Calibration", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 标定相机
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    obj_points,
    img_points,
    gray.shape[::-1],
    None,
    None
)

print("相机内参矩阵:")
print(camera_matrix)

print("畸变系数:")
print(dist_coeffs)
```

### 使用标定结果

```python
# 去畸变
undistorted = cv2.undistort(image, camera_matrix, dist_coeffs)

# 保存标定结果
cv_file = cv2.FileStorage("calibration.xml", cv2.FILE_STORAGE_WRITE)
cv_file.write("camera_matrix", camera_matrix)
cv_file.write("dist_coeffs", dist_coeffs)
cv_file.release()
```

---

## 实用工具函数

### 鼠标交互

```python
import cv2

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"点击位置: ({x}, {y})")
        # 获取颜色
        if isinstance(param, np.ndarray):
            color = param[y, x]
            print(f"颜色 (BGR): {color}")

cv2.namedWindow("Image")
cv2.setMouseCallback("Image", mouse_callback, image)
cv2.imshow("Image", image)
cv2.waitKey(0)
```

### ROI 选择

```python
# 选择感兴趣区域
roi = cv2.selectROI("Select ROI", image, fromCenter=False, showCrosshair=True)

# 裁剪 ROI
x, y, w, h = [int(v) for v in roi]
cropped = image[y:y+h, x:x+w]

cv2.imshow("ROI", cropped)
cv2.waitKey(0)
```

---

## 性能优化

### 多线程处理

```python
import threading
import queue

class CameraThread:
    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        self.q = queue.Queue()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                self.stopped = True
            else:
                self.q.put(frame)

    def read(self):
        return self.q.get()

    def stop(self):
        self.stopped = True
        self.cap.release()

# 使用
camera = CameraThread(0).start()
frame = camera.read()
```

---

## 常见问题

### 摄像头无法打开

```python
# 检查可用摄像头
for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"摄像头 {i} 可用")
        cap.release()
```

### 性能问题

```python
# 降低分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 使用灰度图处理
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
```

---

## 参考资源

- [OpenCV 官方文档](https://docs.opencv.org/)
- [OpenCV Python 教程](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [OpenCV 中文网](https://www.opencv.org.cn/)
