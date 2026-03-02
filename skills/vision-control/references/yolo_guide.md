# YOLOv8 使用指南

## 简介

YOLOv8 (You Only Look Once version 8) 是目前最流行的实时目标检测模型之一,具有速度快、精度高的特点。

## 安装

```bash
pip install ultralytics
```

## 模型选择

| 模型 | mAP (val) | 速度 (CPU) | 速度 (GPU) | 参数量 |
|------|-----------|------------|------------|--------|
| YOLOv8n | 37.3 | 80.4 ms | 0.99 ms | 3.2M |
| YOLOv8s | 44.9 | 128.4 ms | 1.20 ms | 11.2M |
| YOLOv8m | 50.2 | 234.7 ms | 1.83 ms | 25.9M |
| YOLOv8l | 52.9 | 375.2 ms | 2.39 ms | 43.7M |
| YOLOv8x | 53.9 | 479.1 ms | 3.53 ms | 68.2M |

**推荐**:
- 边缘设备/Raspberry Pi: YOLOv8n
- 普通 CPU: YOLOv8n 或 YOLOv8s
- 有 GPU: YOLOv8m 或 YOLOv8l

## 基本使用

### Python API

```python
from ultralytics import YOLO

# 加载模型
model = YOLO('yolov8n.pt')  # 自动下载预训练模型

# 推理
results = model('image.jpg')

# 处理结果
for result in results:
    boxes = result.boxes  # 边界框
    for box in boxes:
        # 获取类别
        class_id = int(box.cls[0])
        class_name = model.names[class_id]

        # 获取置信度
        confidence = float(box.conf[0])

        # 获取边界框坐标
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

        print(f"检测到: {class_name}, 置信度: {confidence:.2f}")
        print(f"位置: ({x1:.0f}, {y1:.0f}) - ({x2:.0f}, {y2:.0f})")
```

### 摄像头实时检测

```python
from ultralytics import YOLO
import cv2

# 加载模型
model = YOLO('yolov8n.pt')

# 打开摄像头
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 推理
    results = model(frame, verbose=False)

    # 可视化
    annotated_frame = results[0].plot()

    # 显示
    cv2.imshow('YOLOv8 Detection', annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

## 支持的类别 (COCO 80类)

### 人物类
- person: 人

### 动物类
- bird: 鸟
- cat: 猫
- dog: 狗
- horse: 马
- sheep: 羊
- cow: 牛
- elephant: 大象
- bear: 熊
- zebra: 斑马
- giraffe: 长颈鹿

### 交通工具类
- bicycle: 自行车
- car: 汽车
- motorcycle: 摩托车
- airplane: 飞机
- bus: 公交车
- train: 火车
- truck: 卡车
- boat: 船

### 交通设施类
- traffic light: 交通灯
- fire hydrant: 消防栓
- stop sign: 停车标志
- parking meter: 停车计时器

### 日常用品类
- bench: 长凳
- backpack: 背包
- umbrella: 雨伞
- handbag: 手提包
- tie: 领带
- suitcase: 手提箱
- frisbee: 飞盘
- skis: 滑雪板
- snowboard: 滑雪板
- sports ball: 运动球
- kite: 风筝
- baseball bat: 棒球棒
- baseball glove: 棒球手套
- skateboard: 滑板
- surfboard: 冲浪板
- tennis racket: 网球拍

### 餐饮类
- bottle: 瓶子
- wine glass: 酒杯
- cup: 杯子
- fork: 叉子
- knife: 刀
- spoon: 勺子
- bowl: 碗
- banana: 香蕉
- apple: 苹果
- sandwich: 三明治
- orange: 橙子
- broccoli: 西兰花
- carrot: 胡萝卜
- hot dog: 热狗
- pizza: 披萨
- donut: 炸面圈
- cake: 蛋糕

### 家具类
- chair: 椅子
- couch: 沙发
- potted plant: 盆栽植物
- bed: 床
- dining table: 餐桌
- toilet: 厕所

### 电子产品类
- tv: 电视
- monitor: 显示器
- laptop: 笔记本电脑
- mouse: 鼠标
- remote: 遥控器
- keyboard: 键盘
- cell phone: 手机
- microwave: 微波炉
- oven: 烤箱
- toaster: 烤面包机
- dishwasher: 洗碗机
- refrigerator: 冰箱

### 其他类
- book: 书
- vase: 花瓶
- scissors: 剪刀
- teddy bear: 泰迪熊
- hair drier: 吹风机
- toothbrush: 牙刷

## 高级用法

### 筛选特定类别

```python
# 只检测人、车、狗
results = model(frame, classes=[0, 2, 16])  # person, car, dog的类别ID
```

### 设置置信度阈值

```python
results = model(frame, conf=0.5)  # 置信度 > 0.5
```

### 设置NMS阈值

```python
results = model(frame, iou=0.5)  # IoU阈值
```

### 自定义推理尺寸

```python
results = model(frame, imgsz=640)  # 推理图像尺寸
```

## 模型训练(自定义数据集)

```python
from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')

# 训练
model.train(
    data='custom_dataset.yaml',
    epochs=100,
    imgsz=640
)
```

## 性能优化

### 使用GPU加速

```python
import torch

# 检查CUDA可用
print(torch.cuda.is_available())

# 模型会自动使用GPU(如果可用)
model = YOLO('yolov8n.pt')
results = model(frame, device='cuda')  # 明确指定使用GPU
```

### 降低分辨率以提高速度

```python
results = model(frame, imgsz=320)  # 默认640
```

### 使用模型量化

```python
# 导出为ONNX格式(支持量化)
model.export(format='onnx', opset=12, simplify=True)
```

## 常见问题

### Q1: 模型下载失败

**解决方案**:
```bash
# 手动下载模型文件
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# 或者使用国内镜像
export HF_ENDPOINT=https://hf-mirror.com
```

### Q2: 检测速度慢

**解决方案**:
- 使用更小的模型 (YOLOv8n)
- 降低推理分辨率
- 使用GPU加速
- 使用TensorRT优化(需要GPU)

### Q3: 检测精度不足

**解决方案**:
- 使用更大的模型 (YOLOv8m/l/x)
- 提高输入分辨率
- 降低置信度阈值
- 收集更多数据训练自定义模型

## 参考链接

- 官方文档: https://docs.ultralytics.com/
- GitHub: https://github.com/ultralytics/ultralytics
- 模型下载: https://github.com/ultralytics/assets/releases
