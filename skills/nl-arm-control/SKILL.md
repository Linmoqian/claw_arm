---
name: nl-arm-control
description: 自然语言控制 SO100 机械臂。当用户需要用自然语言描述来控制机械臂执行动作时，使用此技能。支持基础运动（单关节移动、多关节协同、回零位）、复杂任务序列（抓取、放置、倒水）、视觉引导抓取（自动识别并抓取物体）。用户说类似"把机械臂移到左边"、"张开夹爪"、"抓起前面的杯子"、"回到初始位置"等自然语言指令时应触发此技能。
metadata:
  {
    "openclaw": {
      "requires": { "bins": ["python3", "pip3"] },
      "user-invocable": true
    }
  }
---

# 自然语言控制 SO100 机械臂

此技能让用户通过自然语言描述来控制 SO100 六自由度机械臂，自动生成可运行的 Python 代码并立即执行。

## 工作流程

1. **理解用户意图** - 分析自然语言描述，确定要执行的任务类型
2. **生成控制代码** - 基于现有 API 生成最小可运行代码
3. **立即执行** - 运行代码控制机械臂
4. **保存代码**（可选）- 将代码保存到 `script/` 目录供后续使用

## 任务类型映射

| 用户描述 | 任务类型 | API 调用 |
|----------|----------|----------|
| "移动关节X" / "转到位置X" | 基础运动 | `set_joint()`, `move_smooth()` |
| "回零位" / "回到初始位置" | 基础运动 | `go_home()` |
| "张开夹爪" / "闭合夹爪" | 基础运动 | `open_gripper()`, `close_gripper()` |
| "抓取物体" / "拿起" | 复杂任务 | `pick_object()` |
| "放置物体" / "放下" | 复杂任务 | `place_object()` |
| "倒水" / "倾倒" | 复杂任务 | `pour_water()` |
| "识别并抓取XX" | 视觉引导 | `VisionDetector` + `pick_object()` |
| "执行动作序列" | 序列控制 | `execute_sequence()` |

## 关节名称映射

用户可能会用多种方式指代关节，需要识别并映射到标准名称：

| 用户可能的说法 | 标准名称 | 电机 ID |
|----------------|----------|---------|
| 底部旋转、底座旋转、转盘、水平旋转 | shoulder_pan | 1 |
| 大臂、大臂升降、肩部抬起 | shoulder_lift | 2 |
| 小臂、肘部、小臂弯曲 | elbow_flex | 3 |
| 手腕俯仰、手腕上下、手腕弯曲 | wrist_flex | 4 |
| 手腕旋转、手腕转动、手腕扭转 | wrist_roll | 5 |
| 夹爪、抓手、手爪 | gripper | 6 |

## 方向/位置映射

| 描述 | 位置值 |
|------|--------|
| 最左/最右 | 0 / 4095 |
| 最高/最低 | 0 / 4095 |
| 中间/中间位置/初始位置 | 2047 |
| 张开夹爪 | 3000 |
| 闭合夹爪 | 1000 |

## 代码生成模板

### 基础：导入和初始化

```python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from skills.arm-control.scripts.main import create_controller
from skills.vision-control.scripts.vision_detector import VisionDetector

# 创建控制器
ctrl = create_controller(port="COM7")
ctrl.connect()

try:
    # ... 用户任务代码 ...
    pass
finally:
    ctrl.disconnect()
```

### 模板 1：单关节移动

```python
# 移动单个关节
ctrl.set_joint("shoulder_pan", 2500)
```

### 模板 2：平滑移动多关节

```python
# 平滑移动到目标位置
ctrl.move_smooth({
    "shoulder_pan": 2500,
    "shoulder_lift": 2000,
    "gripper": 3000
})
```

### 模板 3：回零位

```python
# 回到初始位置
ctrl.go_home()
```

### 模板 4：抓取序列

```python
# 抓取物体
ctrl.open_gripper()  # 张开夹爪
time.sleep(1.0)
ctrl.move_smooth({"shoulder_pan": 2047, "shoulder_lift": 2500})  # 移动到抓取位置
time.sleep(0.5)
ctrl.close_gripper()  # 闭合夹爪
time.sleep(1.0)
ctrl.move_smooth({"shoulder_lift": 1500})  # 抬起
```

### 模板 5：视觉引导抓取

```python
# 视觉检测并抓取
detector = VisionDetector(camera_id=0, detector="yolo", output_format="all")
detector.load_model()

results = detector.detect_objects(target="cup", confidence=0.5)
if results:
    best = results[0]  # 取置信度最高的
    print(f"检测到 {best.class_name}，置信度 {best.confidence}")
    # 根据检测结果移动机械臂（需要手眼标定）
    ctrl.pick_object()
else:
    print("未检测到目标")
detector.release()
```

### 模板 6：自由拖动模式

```python
# 释放扭矩，手动拖动
positions = ctrl.freedrag()
print("当前位置:", positions)
# 按 Ctrl+C 退出，自动重新启用扭矩
```

## 代码执行注意事项

1. **安全检查** - 位置值必须在 0-4095 范围内
2. **扭矩管理** - 移动前自动启用扭矩，断开时自动禁用
3. **异常处理** - 使用 try-finally 确保断开连接
4. **等待时间** - 动作之间适当添加 `time.sleep()`

## 代码保存规则

代码保存到 `script/` 目录，文件名规则：
- 基础运动: `arm_{action}.py`（如 `arm_move_left.py`）
- 复杂任务: `arm_{task}.py`（如 `arm_pick_cup.py`）
- 视觉任务: `vision_{task}.py`（如 `vision_grab_cup.py`）

## 交互流程

1. 用户输入自然语言描述
2. 显示解析出的任务摘要，确认理解
3. 生成代码并显示预览
4. 执行代码
5. 询问是否保存代码

## 示例对话

**用户**: "把夹爪张开"

**响应**:
```
任务: 张开夹爪
生成代码: ctrl.open_gripper()
正在执行...
[执行结果]
是否保存代码? (y/n)
```

**用户**: "识别摄像头中的杯子并抓起来"

**响应**:
```
任务: 视觉引导抓取杯子
步骤:
1. 加载视觉检测器
2. 检测杯子
3. 移动到目标位置
4. 抓取
正在执行...
[执行过程和结果]
是否保存代码? (y/n)
```

## 错误处理

- 未识别的关节名称 → 列出可用关节
- 位置值超出范围 → 裁剪到 [0, 4095] 并警告
- 摄像头未打开 → 提示检查摄像头连接
- 电机离线 → 显示在线电机列表
- 检测失败 → 提示调整目标描述或检测参数
