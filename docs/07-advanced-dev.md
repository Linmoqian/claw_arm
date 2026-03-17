# 二次开发指南

本文档面向希望在项目基础上进行二次开发的用户。

## 项目架构

```
claw_arm/
├── CV/           # 视觉模块（独立）
├── SDK/          # 底层通信（稳定，不建议修改）
├── script/       # 调试脚本（可参考）
├── Joints/       # 关节控制封装（可扩展）
├── skills/       # 自然语言技能（可自定义）
└── examples/     # 示例代码（可添加）
```

## 扩展点

### 1. 添加新的视觉检测

在 `CV/` 目录下添加新脚本：

```python
# CV/6_yolo_detection.py
"""
YOLO 目标检测示例
"""
import cv2
from ultralytics import YOLO

def main():
    model = YOLO("yolo26n.pt")
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame)
        # 处理检测结果...

        cv2.imshow("YOLO", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

### 2. 创建自定义 Skill

在 `skills/nature_arm/` 下扩展：

```markdown
# skills/custom_arm/SKILL.md

## 规则

1. 用户安全第一
2. 自身安全第二
3. 执行用户指令

## 技能

- /custom_arm [指令]
  - 支持的指令：抓取、放置、移动

## 脚本

- scripts/custom_control.py
```

### 3. 扩展关节控制

在 `Joints/` 下添加新的控制类：

```python
# Joints/arm_controller.py
"""
完整的机械臂控制器
"""
from SDK import PortHandler, PacketHandler

class ArmController:
    def __init__(self, port, pkt):
        self.port = port
        self.pkt = pkt
        self.joints = {}

    def add_joint(self, name, joint_id):
        self.joints[name] = joint_id

    def move_to(self, positions):
        """移动到指定位置"""
        for name, pos in positions.items():
            # 执行移动...
            pass

    def home(self):
        """回到初始位置"""
        self.move_to({
            "base": 2048,
            "shoulder": 2048,
            "elbow": 2048,
            # ...
        })
```

## 最佳实践

### 代码组织

```
your_feature/
├── __init__.py
├── core.py       # 核心逻辑
├── utils.py      # 工具函数
└── tests/        # 测试
    └── test_core.py
```

### 配置管理

使用 YAML 配置文件：

```python
import yaml

def load_config(path="config/hardware.yaml"):
    with open(path) as f:
        return yaml.safe_load(f)
```

### 错误处理

```python
class ArmError(Exception):
    """机械臂错误基类"""
    pass

class ConnectionError(ArmError):
    """连接错误"""
    pass

class MotionError(ArmError):
    """运动错误"""
    pass
```

## 示例项目

### 自动分拣系统

```python
# examples/advanced/sorting_system.py
"""
自动分拣系统
- 检测物体颜色
- 根据颜色分类
- 放置到对应位置
"""

COLORS = {
    "red": {"position": (0.2, 0.1, 0.1)},
    "green": {"position": (0.2, 0.2, 0.1)},
    "blue": {"position": (0.2, 0.3, 0.1)},
}

def sort_object(color):
    # 1. 抓取物体
    # 2. 移动到目标位置
    # 3. 放置物体
    pass
```

### 遥控机械臂

```python
# examples/advanced/remote_control.py
"""
通过网络遥控机械臂
"""

from flask import Flask, request
app = Flask(__name__)

@app.route('/move', methods=['POST'])
def move():
    x = request.json.get('x')
    y = request.json.get('y')
    z = request.json.get('z')
    # 执行移动...
    return {"status": "ok"}
```

## 调试技巧

### 日志记录

```python
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

logger.info("开始执行抓取")
logger.error("连接失败")
```

### 可视化调试

```python
import matplotlib.pyplot as plt

def plot_trajectory(positions):
    x = [p[0] for p in positions]
    y = [p[1] for p in positions]
    plt.plot(x, y)
    plt.show()
```

## 贡献代码

1. Fork 项目
2. 创建功能分支
3. 提交 Pull Request

代码规范：
- 使用 4 空格缩进
- 函数添加文档字符串
- 变量使用 snake_case
