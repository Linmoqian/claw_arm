# CV 视觉模块

为 SO100 机械臂提供视觉支持的独立脚本集合。

## 脚本说明

| 脚本 | 功能 | 输出 |
|------|------|------|
| `1_camera_test.py` | 摄像头测试 | 实时画面、分辨率、FPS |
| `2_color_detection.py` | 颜色识别 | 框选检测物体 |
| `3_object_position.py` | 像素坐标 | (cx, cy) 像素坐标 |
| `4_world_coordinate.py` | 世界坐标转换 | (X, Y, Z) 世界坐标 |
| `5_distance_measure.py` | 距离测量 | 物体到摄像头距离 |

## 快速开始

```bash
# 激活环境
conda activate lerobot

# 运行脚本
python CV/1_camera_test.py
python CV/2_color_detection.py
python CV/3_object_position.py
python CV/4_world_coordinate.py
python CV/5_distance_measure.py
```

## 配置文件

`camera_config.json` - 相机标定参数：
- `camera_matrix`: 相机内参矩阵
- `dist_coeffs`: 畸变系数
- `fixed_height`: 假设的固定 Z 高度（米）

## 操作说明

- `q`: 退出程序
- `s`: 调整 HSV 颜色范围（部分脚本支持）

## 依赖

- opencv-python
- numpy
