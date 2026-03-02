```skill
# vision-detection

## 描述

SO100机械臂视觉检测定位技能 —— 通过摄像头和深度学习模型识别物体,计算精确位置,为机械臂抓取提供多格式坐标数据。

**核心功能**:
- 使用YOLOv8/通义千问视觉API进行目标检测
- 输出多种坐标格式:像素坐标、机械臂3D坐标、极坐标(角度+距离)
- Eye-in-Hand手眼标定(摄像头固定在机械臂上)
- 实时目标追踪和定位

**机器人配置**:
- 机型: SO100(6自由度单臂)
- 电机: 6×飞特STS3215舵机(ID 1-6)
- 通信: USB串口(默认COM7,波特率1000000)
- 视觉: USB摄像头(固定在机械臂末端/手腕)
- 检测模型: YOLOv8(本地) 或 通义千问VL API(云端)

**关节列表**:

| 电机ID | 关节名称 | 描述 | 位置范围 | 零位 |
|--------|----------|------|----------|------|
| 1 | shoulder_pan | 底部旋转 | 0~4095 | 2047 |
| 2 | shoulder_lift | 大臂升降 | 0~4095 | 2047 |
| 3 | elbow_flex | 小臂弯曲 | 0~4095 | 2047 |
| 4 | wrist_flex | 手腕俯仰 | 0~4095 | 2047 |
| 5 | wrist_roll | 手腕旋转 | 0~4095 | 2047 |
| 6 | gripper | 夹爪开合 | 0~4095 | 2047 |

## 使用场景

- 当用户需要"找到"或"检测"物体位置时
- 当需要获取物体的像素坐标、3D坐标或极坐标数据时
- 当需要视觉引导机械臂进行抓取时
- 当需要进行手眼标定时
- 当需要追踪移动物体时
- 当需要识别多种类型的物体(杯子、手机、瓶子等)时

## 核心功能

### 1. 物体检测
- **YOLOv8本地检测**: 支持COCO 80类物体,低延迟
- **通义千问视觉API**: 开放词汇检测,支持任意物体描述
- **颜色检测**(备用): OpenCV HSV颜色空间检测

### 2. 坐标输出格式

#### 像素坐标 (Pixel Coordinates)
- 输出: `(pixel_x, pixel_y, confidence)`
- 原点: 图像左上角
- 单位: 像素
- 用途: 原始检测结果,可视化调试

#### 机械臂3D坐标 (Arm 3D Coordinates)
- 输出: `(arm_x, arm_y, arm_z, confidence)`
- 原点: 机械臂底座中心
- 单位: 毫米(mm)
- 用途: 直接用于逆运动学求解

#### 极坐标 (Polar Coordinates)
- 输出: `(angle_degrees, distance_mm, confidence)`
- 原点: 图像中心
- 单位: 度(°)和毫米
- 用途: 快速定位,相对于视角中心的偏移

### 3. Eye-in-Hand标定
- 摄像头固定在机械臂末端
- 标定板检测(棋盘格/圆点阵)
- 手眼标定矩阵计算
- 深度估计(基于物体尺寸)

## 使用指令

```
检测桌上的杯子并给出位置
找到所有红色物体
识别手机的位置
标定摄像头
追踪绿色物体
检测所有可见物体
给我物体的3D坐标
计算物体相对于图像中心的角度和距离
```

## 权限要求

- USB摄像头访问权限
- USB串口访问权限(COM7)
- 网络访问权限(使用通义千问API时)
- Python 3.10+

## 依赖

### 必需依赖
- opencv-python: 图像采集和处理
- numpy: 数值计算和坐标变换
- ultralytics: YOLOv8模型推理

### 可选依赖
- OpenAI/dashscope: 通义千问视觉API
- matplotlib: 标定可视化
- scipy: 高级数学运算

### 相关技能
- arm-control: 机械臂底层控制
- scservo-controller-SDK: 舵机SDK控制

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| target | str | 必填 | 要检测的目标描述(如"杯子"、"手机") |
| detector | str | "yolo" | 检测后端:"yolo"、"qwen_vl"、"color" |
| camera_id | int | 0 | 摄像头设备ID |
| output_format | str | "all" | 输出格式:"pixel"、"3d"、"polar"、"all" |
| confidence | float | 0.5 | 检测置信度阈值(0.0-1.0) |
| show_preview | bool | false | 是否显示检测可视化窗口 |
| max_results | int | 10 | 返回最大结果数量 |

## Python API快速参考

### 基础API

```python
from scripts.vision_detector import VisionDetector

# 创建检测器
detector = VisionDetector(
    camera_id=0,
    detector="yolo",  # or "qwen_vl", "color"
    output_format="all"
)

# 初始化
detector.load_model()

# 检测物体
results = detector.detect_objects(
    target="cup",
    confidence=0.5
)

# 输出格式示例:
# [
#     {
#         "class": "cup",
#         "confidence": 0.89,
#         "pixel": (320, 240),
#         "arm_3d": (200.5, 50.3, 100.0),
#         "polar": (0.0, 250.7),
#         "bounding_box": (280, 200, 360, 280)
#     },
#     ...
# ]

# 获取最佳匹配目标
best = detector.get_best_match(results)

# 获取相对于图像中心的角度和距离
angle, distance = detector.get_polar_coord(pixel_x, pixel_y)

# 释放资源
detector.release()
```

### 标定API

```python
from scripts.calibrator import HandEyeCalibrator

# 创建标定器
calibrator = HandEyeCalibrator(
    camera_id=0,
    pattern_type="chessboard",  # or "circles"
    pattern_size=(9, 6)
)

# 执行标定
calibration_data = calibrator.calibrate(
    arm_port="COM7",
    num_poses=15
)

# 保存标定结果
calibrator.save_calibration("calibration_data.json")

# 加载标定数据
calibrator.load_calibration("calibration_data.json")
```

### 高级API

```python
# 批量检测
all_objects = detector.detect_all()

# 过滤特定类别
cups = detector.filter_by_class(all_objects, "cup")

# 按距离排序
sorted_by_distance = detector.sort_by_distance(results)

# 可视化检测
detector.show_results(results, annotate=True)

# 实时追踪
detector.track_object(target="person", max_duration=30)
```

## 坐标系说明

### 像素坐标系
- 原点: 图像左上角
- X轴: 向右
- Y轴: 向下
- 单位: 像素
- 范围: x∈[0, 640], y∈[0, 480]

### 相机坐标系
- 原点: 相机光心
- X轴: 右
- Y轴: 下
- Z轴: 前方(深度)
- 单位: 毫米

### 机械臂坐标系
- 原点: 底座旋转中心
- X轴: 前方
- Y轴: 左方
- Z轴: 上方
- 单位: 毫米

### 极坐标系
- 原点: 图像中心(320, 240)
- 角度: 相对于Z轴的偏角(度),顺时针为正
- 距离: 到原点的像素距离

## 标定流程

### Eye-in-Hand标定步骤

1. **准备标定板**
   - 打印棋盘格标定板(推荐9×6方格,方格大小30mm)
   - 或打印圆点阵列标定板

2. **采集标定数据**
   ```bash
   python scripts/vision-control/scripts/calibrate.py
   ```

   按照提示:
   - 将标定板放置在工作空间
   - 机械臂移动到不同位置(建议15-20个姿态)
   - 每个位置同时采集图像和记录机械臂位姿

3. **计算标定矩阵**
   - 相机内参标定
   - 手眼标定矩阵计算
   - 深度估计参数拟合

4. **验证标定精度**
   - 检测已知物体
   - 比较计算坐标与实际坐标
   - 误差应<5mm

## 工作流程

```
1. 初始化
   ├── 加载检测模型(YOLOv8或连接API)
   ├── 打开摄像头
   └── 加载标定数据

2. 物体检测
   ├── 采集图像
   ├── 前处理(调整大小、归一化)
   ├── 模型推理
   └── 后处理(NMS、置信度过滤)

3. 坐标计算
   ├── 像素坐标提取(边界框中心)
   ├── 深度估计(基于物体尺寸/标定)
   ├── 3D坐标转换(相机坐标系)
   └── 机械臂坐标转换(手眼标定矩阵)

4. 格式化输出
   ├── 像素坐标
   ├── 3D坐标(x, y, z)
   └── 极坐标(角度, 距离)
```

## 物体检测方法

### 1. YOLOv8本地检测(推荐)

**优势**:
- 低延迟(~20ms)
- 支持COCO 80类物体
- 无需网络

**支持类别**:
人、自行车、汽车、摩托车、飞机、公交车、火车、卡车、船、交通灯、
消防栓、停车标志、停车计时器、长凳、鸟、猫、狗、马、羊、牛、大象、
熊、斑马、长颈鹿、背包、雨伞、手提包、领带、手提箱、飞盘、滑雪板、
滑雪板、运动球、风筝、棒球棒、棒球手套、滑板、冲浪板、网球拍、瓶子、
杯子、叉子、刀、勺子、碗、香蕉、苹果、三明治、橙子、西兰花、胡萝卜、
热狗、披萨、炸面圈、蛋糕、椅子、沙发、盆栽植物、床、餐桌、厕所、
显示器、笔记本电脑、鼠标、遥控器、键盘、手机、微波炉、烤箱、烤面包机、
洗碗机、水槽、冰箱、书、花瓶、剪刀、泰迪熊、吹风机、牙刷

**使用示例**:
```python
results = detector.detect_objects(target="cup", detector="yolo")
```

### 2. 通义千问视觉API(开放词汇)

**优势**:
- 支持任意物体描述
- 无需训练模型
- 上下文理解能力强

**使用示例**:
```python
results = detector.detect_objects(
    target="红色的可口可乐瓶子",
    detector="qwen_vl"
)
```

### 3. 颜色检测(备用)

**优势**:
- 简单快速
- 无需模型
- 适合特定颜色物体

**使用示例**:
```python
results = detector.detect_objects(
    target="红色",
    detector="color",
    color_lower=(0, 120, 70),
    color_upper=(10, 255, 255)
)
```

## 安全注意事项

⚠️ **重要**:

1. **摄像头固定**: 确保摄像头牢固固定在机械臂上,避免松动
2. **工作空间**: 确保机械臂运动范围内无障碍物
3. **光照条件**: 保持稳定光照,避免强光直射或阴影
4. **检测延迟**: 实时应用时注意检测延迟,避免机械臂运动过快
5. **坐标验证**: 首次使用前验证标定精度
6. **急停准备**: 随时准备手动急停
7. **物体限制**: 不要抓取易碎、危险或过重物体(<500g)

## 常见问题

### Q1: 检测不到物体

**可能原因**:
- 光照条件不佳
- 物体不在模型类别中(YOLOv8)
- 置信度阈值设置过高
- 摄像头对焦不清

**解决方案**:
- 调整光照
- 使用通义千问API(开放词汇)
- 降低confidence参数
- 检查摄像头对焦

### Q2: 坐标不准确

**可能原因**:
- 未进行手眼标定
- 标定数据不够精确
- 深度估计误差大

**解决方案**:
- 运行标定程序
- 增加标定点数量
- 使用已知尺寸物体辅助深度估计

### Q3: 检测速度慢

**可能原因**:
- 使用云端API(网络延迟)
- 模型推理速度慢
- 图像分辨率过高

**解决方案**:
- 使用YOLOv8本地检测
- 降低输入图像分辨率
- 使用GPU加速(如可用)

## 示例代码

### 示例1: 检测杯子并输出多格式坐标

```python
from scripts.vision_detector import VisionDetector

detector = VisionDetector(camera_id=0, output_format="all")
detector.load_model()

# 检测杯子
results = detector.detect_objects(target="cup", confidence=0.6)

for obj in results:
    print(f"类别: {obj['class']}, 置信度: {obj['confidence']:.2f}")
    print(f"像素坐标: {obj['pixel']}")
    print(f"3D坐标: {obj['arm_3d']}")
    print(f"极坐标: 角度={obj['polar'][0]:.1f}°, 距离={obj['polar'][1]:.1f}px")
    print("-" * 50)

detector.release()
```

### 示例2: 视觉引导抓取

```python
from scripts.vision_detector import VisionDetector
from skills.arm_control.main import create_controller

# 初始化检测器和机械臂
detector = VisionDetector(camera_id=0, output_format="3d")
arm = create_controller(port="COM7")

detector.load_model()
arm.connect()

# 检测物体
results = detector.detect_objects(target="bottle", confidence=0.5)

if results:
    best = detector.get_best_match(results)
    x, y, z = best['arm_3d']

    print(f"检测到瓶子,位置: ({x:.1f}, {y:.1f}, {z:.1f})")

    # 计算关节角度(逆运动学)
    # 这里需要实现或调用逆运动学求解器
    # joints = inverse_kinematics(x, y, z)

    # 移动机械臂
    # arm.move_to(joints)

# 清理
detector.release()
arm.disconnect()
```

### 示例3: 实时追踪

```python
from scripts.vision_detector import VisionDetector

detector = VisionDetector(camera_id=0, show_preview=True)
detector.load_model()

# 追踪绿色物体
detector.track_object(
    target="green",
    duration=30,
    max_lost_frames=10
)

detector.release()
```

### 示例4: 手眼标定

```python
from scripts.calibrator import HandEyeCalibrator

calibrator = HandEyeCalibrator(
    camera_id=0,
    pattern_type="chessboard",
    pattern_size=(9, 6),
    square_size=30.0  # mm
)

# 执行标定
calibration_data = calibrator.calibrate(
    arm_port="COM7",
    num_poses=15
)

# 保存结果
calibrator.save_calibration("calibration_data.json")

print(f"标定完成,重投影误差: {calibration_data['reprojection_error']:.3f}像素")
```

## 参考资源

### 标定工具
- `scripts/calibrate.py`: 交互式手眼标定程序
- `scripts/test_camera.py`: 摄像头测试工具
- `scripts/color_picker.py`: HSV颜色拾取工具

### 参考文档
- `references/yolo_guide.md`: YOLOv8使用指南
- `references/hand_eye_calibration.md`: 手眼标定原理
- `references/coordinate_transforms.md`: 坐标变换详解

## 相关技能

- `arm-control`: 机械臂底层控制
- `scservo-controller-SDK`: 舵机SDK控制
- `vision-grasp`: 视觉抓取(整合检测+控制)
```
