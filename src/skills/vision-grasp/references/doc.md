# vision-grasp 技能参考文档

## 概述

`vision-grasp` 是 OpenClaw Agent 的视觉定位抓取技能。它将摄像头采集的图像通过目标检测算法定位物体，然后经坐标变换计算出机械臂可执行的关节角度，最终调用 `arm-control` 技能完成抓取。

## 完整流程

```
                   用户: "帮我抓桌上的水杯"
                              │
                   ┌──────────▼──────────┐
                   │   OpenClaw Agent    │
                   │   解析意图 → 激活    │
                   │   vision-grasp      │
                   └──────────┬──────────┘
                              │
              ┌───────────────▼───────────────┐
              │        vision-grasp           │
              │                               │
              │  ┌─────────────────────────┐  │
              │  │ 1. 图像采集              │  │
              │  │    CameraCapture        │  │
              │  └────────┬────────────────┘  │
              │           │ BGR (720×1280×3)  │
              │  ┌────────▼────────────────┐  │
              │  │ 2. 目标检测              │  │
              │  │    YOLOv8 / 千问VL /    │  │
              │  │    颜色检测              │  │
              │  └────────┬────────────────┘  │
              │           │ Detection[]       │
              │  ┌────────▼────────────────┐  │
              │  │ 3. 坐标变换              │  │
              │  │    像素 → 相机 → 基座标  │  │
              │  └────────┬────────────────┘  │
              │           │ Pose3D            │
              │  ┌────────▼────────────────┐  │
              │  │ 4. 逆运动学              │  │
              │  │    位姿 → 关节角度       │  │
              │  └────────┬────────────────┘  │
              │           │ joint_angles[]    │
              └───────────┼───────────────────┘
                          │
              ┌───────────▼───────────────┐
              │      arm-control           │
              │                            │
              │  打开夹爪 → 预抓取 →      │
              │  下降 → 关闭夹爪 → 抬起   │
              └────────────────────────────┘
```

## 检测后端对比

| 特性 | YOLOv8 | 通义千问 VL | OpenCV 颜色 |
|------|--------|-----------|------------|
| 速度 | ~30ms/帧 (GPU) | ~2s/帧 (API) | ~5ms/帧 |
| 精度 | 高 (COCO 80类) | 高 (开放词汇) | 低 (仅颜色) |
| 灵活性 | 需训练自定义类别 | 任意文字描述 | 仅颜色特征 |
| 依赖 | ultralytics + PyTorch | openai SDK + 网络 | 仅 OpenCV |
| GPU | 推荐 | 不需要 | 不需要 |
| 离线 | 支持 | 不支持 | 支持 |
| 适用 | 产线/固定场景 | 灵活探索/未知物体 | 快速原型 |

### 选择建议

- **已知物体、追求速度** → YOLOv8
- **未知物体、自然语言描述** → 通义千问 VL
- **颜色明显、极简部署** → OpenCV 颜色检测

## 坐标变换详解

### 1. 像素坐标 → 相机坐标

已知像素坐标 $(u, v)$ 和深度 $d$，以及相机内参：

$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

计算相机坐标系下 3D 点：

$$X_{cam} = \frac{(u - c_x) \cdot d}{f_x}, \quad Y_{cam} = \frac{(v - c_y) \cdot d}{f_y}, \quad Z_{cam} = d$$

### 2. 相机坐标 → 基座标

通过手眼标定得到的 4×4 齐次变换矩阵 $T_{base}^{cam}$：

$$P_{base} = T_{base}^{cam} \cdot P_{cam}$$

### 3. 深度获取

本技能支持以下方式获取深度信息：

| 方式 | 精度 | 说明 |
|------|------|------|
| 深度相机 | 高 | 直接读取像素深度值 |
| 固定高度假设 | 中 | 已知桌面高度，假设物体在桌面上 |
| 默认值 | 低 | 使用预设的 default_depth 参数 |

## 相机标定

### 内参标定

使用 OpenCV 棋盘格标定法：

```bash
# 采集标定图像后运行
python -c "
import cv2
import numpy as np
import glob

# 棋盘格参数
chess_size = (9, 6)
square_size = 0.025  # 25mm

objp = np.zeros((chess_size[0]*chess_size[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chess_size[0], 0:chess_size[1]].T.reshape(-1,2) * square_size

obj_points, img_points = [], []
images = glob.glob('calibration_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chess_size, None)
    if ret:
        obj_points.append(objp)
        img_points.append(corners)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)
print(f'内参矩阵 K:\n{K}')
np.save('camera_intrinsics.npy', K)
"
```

### 手眼标定

需要已知多组"机械臂末端位姿"和"标定板在相机下的位姿"对。
推荐使用 `cv2.calibrateHandEye()` 或 MoveIt2 手眼标定工具。

## 逆运动学

### 当前实现

脚本中内置了一个简化的 2-连杆几何 IK 求解器，仅用于演示。

### 推荐替换方案

| 方案 | 优点 | 安装 |
|------|------|------|
| ikpy | 纯 Python，支持 URDF | `pip install ikpy` |
| PyKDL | 高性能 C++ 后端 | 随 ROS2 安装 |
| MoveIt2 | 碰撞检测 + 运动规划 | ROS2 生态 |
| 解析解 | 最快，精确 | 需按 DH 参数推导 |

使用 ikpy 替换的示例：

```python
import ikpy.chain
import numpy as np

# 从 URDF 加载运动链
chain = ikpy.chain.Chain.from_urdf_file("r1_lite_left_arm.urdf")

# 求解逆运动学
target = [0.3, 0.1, 0.2]  # 目标位置 (x, y, z)
joints = chain.inverse_kinematics(target)
print(f"关节角度: {joints}")
```

## 与 arm-control 的接口

vision-grasp 通过 Python import 调用 arm-control：

```python
# vision-grasp 内部调用方式
from main import create_controller, ArmSide

ctrl = create_controller(mode="sdk", port="COM3")
ctrl.connect()

# 使用 vision-grasp 计算出的关节角度
ctrl.set_gripper(1.0, arm=ArmSide.LEFT)           # 打开夹爪
ctrl.set_joint_positions(pre_joints, arm=ArmSide.LEFT)   # 预抓取
ctrl.set_joint_positions(grasp_joints, arm=ArmSide.LEFT) # 抓取
ctrl.set_gripper(0.0, arm=ArmSide.LEFT)           # 关闭夹爪
ctrl.set_joint_positions(pre_joints, arm=ArmSide.LEFT)   # 抬起

ctrl.disconnect()
```

## 通义千问视觉 API 配置

### 获取 API Key

1. 访问 https://dashscope.console.aliyun.com/
2. 开通"通义千问"服务
3. 在"API-KEY 管理"中创建 Key

### 设置环境变量

```bash
# Linux / macOS
export DASHSCOPE_API_KEY="sk-xxxxx"

# Windows PowerShell
$env:DASHSCOPE_API_KEY = "sk-xxxxx"
```

### 或通过命令行参数传入

```bash
python main.py --target 杯子 --detector qwen_vl --qwen-api-key sk-xxxxx
```

## 常见问题

### Q: 检测不到目标？

1. 确认摄像头已正确连接且可采集图像
2. 降低 `--confidence` 阈值（如 0.3）
3. 尝试不同的检测后端
4. 对于 YOLOv8，确认目标在 COCO 80 类中；否则使用千问 VL

### Q: 坐标偏差大？

1. 完成相机内参标定（替换默认值）
2. 完成手眼标定（替换默认外参矩阵）
3. 使用深度相机获取精确深度值

### Q: 抓不住物体？

1. 调整 `--grasp-height` 参数
2. 检查逆运动学解是否合理
3. 可能需要替换为精确的 IK 求解器
4. 确认夹爪开合范围匹配物体尺寸
