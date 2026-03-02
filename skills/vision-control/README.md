# Vision Control Skill - SO100 机械臂视觉控制

## 概述

`vision-control` 是 SO100 机械臂的视觉识别和控制技能，通过摄像头识别物体位置并控制机械臂进行抓取操作。

## 功能特性

- **颜色识别**：基于 HSV 颜色空间的物体检测
- **形状识别**：圆形、矩形、三角形等形状检测
- **手眼标定**：相机与机械臂坐标系转换
- **自动抓取**：从视觉识别到机械臂抓取的完整流程
- **实用工具**：颜色拾取、摄像头测试、标定工具

## 目录结构

```
vision-control/
├── SKILL.md                   # 技能定义文件
├── README.md                  # 本文件
├── scripts/                   # 脚本目录
│   ├── vision_controller.py   # 核心视觉控制器
│   ├── calibrate.py           # 手眼标定工具
│   ├── color_picker.py        # HSV 颜色拾取工具
│   └── test_camera.py         # 摄像头测试工具
└── references/                # 参考文档
    └── opencv_guide.md        # OpenCV 使用指南
```

## 快速开始

### 1. 环境准备

确保已安装以下依赖：

```bash
# OpenCV
pip install opencv-python

# NumPy
pip install numpy

# 依赖 arm-control 技能
cd skills/arm-control
pip install -e .
```

### 2. 测试摄像头

首先测试摄像头是否正常工作：

```bash
python scripts/vision-control/scripts/test_camera.py
```

### 3. 颜色标定（可选）

如果需要检测特定颜色，使用颜色拾取工具：

```bash
python scripts/vision-control/scripts/color_picker.py
```

在画面中点击目标颜色，调整滑块获得最佳 HSV 范围，按 's' 保存。

### 4. 手眼标定

首次使用必须进行手眼标定：

```bash
python scripts/vision-control/scripts/calibrate.py
```

按提示采集标定点，系统会自动计算坐标转换矩阵并保存。

## 使用示例

### 示例 1：颜色识别抓取

```python
from scripts.vision_controller import VisionController

# 创建控制器
vc = VisionController(port="COM7", camera_id=0)
vc.connect()
vc.open_camera()

# 检测红色物体
red_objects = vc.detect_by_color(
    color_lower=(0, 120, 70),
    color_upper=(10, 255, 255),
    show_debug=True
)

if red_objects:
    # 抓取最大的红色物体
    largest = vc.get_largest_object(red_objects)
    vc.grasp_object_at(largest["center_x"], largest["center_y"])
    print("抓取完成")
else:
    print("未找到红色物体")

vc.disconnect()
```

### 示例 2：形状识别抓取

```python
from scripts.vision_controller import VisionController

vc = VisionController(port="COM7", camera_id=0)
vc.connect()
vc.open_camera()

# 检测圆形物体（如球、杯子）
circles = vc.detect_by_shape("circle")

for circle in circles:
    print(f"发现圆形物体，位置: ({circle['center_x']}, {circle['center_y']})")
    vc.grasp_object_at(circle["center_x"], circle["center_y"])
    vc.place_object_at(320, 100)  # 放置到指定位置

vc.disconnect()
```

## 常见颜色 HSV 范围

| 颜色 | H 下限 | H 上限 | S 下限 | S 上限 | V 下限 | V 上限 |
|------|--------|--------|--------|--------|--------|--------|
| 红色 | 0 | 10 | 120 | 255 | 70 | 255 |
| 绿色 | 40 | 80 | 50 | 255 | 50 | 255 |
| 蓝色 | 100 | 130 | 150 | 255 | 0 | 255 |
| 黄色 | 20 | 40 | 100 | 255 | 100 | 255 |

注意：红色在 HSV 空间中有两个范围，另一个范围是 H: 170-180。

## 安全注意事项

⚠️ **重要安全提示**：

1. **工作空间**：确保机械臂周围有足够空间（至少 1 米半径）
2. **摄像头位置**：确保摄像头固定且不会被机械臂碰撞
3. **光照条件**：保持稳定光照，避免强光直射或阴影
4. **首次测试**：从较高位置开始测试，避免碰撞桌面
5. **急停准备**：随时准备手动急停（断电或断开 USB）
6. **物体限制**：不要抓取易碎、危险或过重物体（< 500g）

## 故障排查

### 问题 1：检测不到物体

**可能原因**：
- 光照条件不佳
- 物体颜色与背景相近
- HSV 颜色范围设置不当

**解决方案**：
- 调整光照，使用均匀光源
- 使用颜色拾取工具获取准确的 HSV 值
- 增加图像预处理步骤（去噪、滤波）

### 问题 2：抓取位置不准确

**可能原因**：
- 未进行手眼标定
- 标定数据不够精确
- 机械臂未回零

**解决方案**：
- 运行标定程序：`python scripts/calibrate.py`
- 增加标定点数量（建议 8-10 个）
- 标定点要覆盖整个工作空间
- 确保机械臂从零位开始

### 问题 3：摄像头无法打开

**可能原因**：
- 摄像头被其他程序占用
- 摄像头 ID 不正确

**解决方案**：
```bash
# 列出可用摄像头
python scripts/test_camera.py --list

# 使用正确的摄像头 ID
python scripts/test_camera.py --camera 1
```

### 问题 4：机械臂运动异常

**可能原因**：
- 目标位置超出工作空间
- 坐标转换错误

**解决方案**：
- 检查坐标转换矩阵
- 限制目标范围在安全区域内
- 使用较小的移动步长进行测试

## API 参考

### VisionController 类

#### 初始化

```python
VisionController(
    port="COM7",              # 串口端口号
    baudrate=1000000,         # 波特率
    camera_id=0,              # 摄像头 ID
    calibration_file=None     # 标定文件路径
)
```

#### 主要方法

| 方法 | 说明 |
|------|------|
| `connect()` | 连接机械臂和摄像头 |
| `disconnect()` | 断开连接 |
| `open_camera()` | 打开摄像头 |
| `capture_frame()` | 捕获一帧图像 |
| `detect_by_color(lower, upper)` | 通过颜色检测物体 |
| `detect_by_shape(shape)` | 通过形状检测物体 |
| `get_largest_object(objects)` | 获取最大物体 |
| `pixel_to_arm_coord(x, y)` | 像素坐标转机械臂坐标 |
| `grasp_object_at(x, y)` | 在指定位置抓取物体 |
| `place_object_at(x, y)` | 在指定位置放置物体 |

## 标定流程详解

### 准备工作

1. 准备一个明显的标记物（如红色圆形物体，直径 3-5cm）
2. 确保光照均匀稳定
3. 清理工作空间

### 标定步骤

```bash
python scripts/vision-control/scripts/calibrate.py
```

1. 在工作空间放置标记物
2. 在摄像头画面中点击标记物中心
3. 手动移动机械臂末端到标记物位置
4. 输入当前机械臂坐标（X, Y, Z）
5. 重复步骤 1-4，至少采集 6 个点
6. 标定点要均匀分布在工作空间各位置
7. 系统自动计算标定矩阵并保存

### 标定验证

标定完成后，系统会询问是否测试。建议进行测试验证：

1. 将标记物放到任意位置
2. 在图像中点击标记物
3. 观察机械臂是否准确移动到该位置
4. 如有偏差，考虑重新标定

## 相关技能

- `arm-control`：SO100 机械臂底层控制技能
- `scservo-controller-SDK`：舵机 SDK 控制技能

## 参考资源

### 文档
- `SKILL.md`：完整的技能使用说明
- `references/opencv_guide.md`：OpenCV 视觉处理指南
- `../arm-control/README.md`：机械臂控制文档

### 工具脚本
- `scripts/test_camera.py`：测试摄像头
- `scripts/color_picker.py`：HSV 颜色拾取
- `scripts/calibrate.py`：手眼标定

### 外部资源
- [OpenCV 官方文档](https://docs.opencv.org/)
- [HSV 颜色空间解释](https://en.wikipedia.org/wiki/HSL_and_HSV)
- [手眼标定原理](https://en.wikipedia.org/wiki/Robot_calibrations)

## 贡献

欢迎提交问题报告和改进建议！

## 许可

MIT License
