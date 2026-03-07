# 自然语言控制 SO100 机械臂

通过自然语言描述控制 SO100 六自由度机械臂，自动生成并执行控制代码。

## 功能特性

- **基础运动控制**: 单关节移动、多关节协同、回零位
- **复杂任务序列**: 抓取、放置、倒水等复合动作
- **视觉引导抓取**: 结合 YOLOv8 或通义千问 VL API 自动识别物体
- **代码自动保存**: 生成的代码可保存到 `script/` 目录供后续使用

## 安装

此技能是项目的一部分，无需额外安装。确保项目结构如下：

```
claw_arm/
├── SDK/                          # 飞特舵机通信协议
├── skills/
│   ├── arm-control/              # 机械臂控制技能
│   ├── vision-control/           # 视觉检测技能
│   └── nl-arm-control/           # 本技能
│       ├── SKILL.md
│       ├── README.md
│       ├── scripts/
│       │   └── executor.py
│       └── evals/
└── script/                       # 生成的代码保存位置
```

## 使用方法

### 方式 1：命令行交互模式

**Windows 用户注意**：由于 Windows 终端默认编码问题，需要设置 `PYTHONIOENCODING` 环境变量：

```bash
# PowerShell
$env:PYTHONIOENCODING="utf-8"
python skills/nl-arm-control/scripts/executor.py

# CMD
set PYTHONIOENCODING=utf-8
python skills/nl-arm-control/scripts/executor.py

# Git Bash / Bash
PYTHONIOENCODING=utf-8 python skills/nl-arm-control/scripts/executor.py
```

然后输入自然语言指令：
```
nl-arm> 张开夹爪
nl-arm> 回到初始位置
nl-arm> 把大臂移动到 2500
nl-arm> 抓起杯子
nl-arm> 进入自由拖动模式
nl-arm> quit
```

### 方式 2：直接执行指令

```bash
python skills/nl-arm-control/scripts/executor.py "张开夹爪"
```

### 方式 3：不保存代码

```bash
python skills/nl-arm-control/scripts/executor.py "张开夹爪" --no-save
```

## 支持的自然语言指令

### 基础运动

| 指令示例 | 说明 |
|----------|------|
| 张开夹爪 / 打开夹爪 | 夹爪张开到 3000 |
| 闭合夹爪 / 关闭夹爪 | 夹爪闭合到 1000 |
| 回到初始位置 / 回零位 | 所有关节回到 2047 |
| 把大臂移动到 2500 | 大臂(shoulder_lift)移到 2500 |
| 把手腕转到左边 | 手腕旋转到左位置 |

### 关节名称支持

| 标准名称 | 可用说法 |
|----------|----------|
| shoulder_pan | 底座、底部旋转、转盘、水平旋转、旋转 |
| shoulder_lift | 大臂、大臂升降、肩部抬起、大臂抬起、肩部 |
| elbow_flex | 小臂、肘部、小臂弯曲、肘关节 |
| wrist_flex | 手腕俯仰、手腕上下、手腕弯曲、手腕 |
| wrist_roll | 手腕旋转、手腕转动、手腕扭转 |
| gripper | 夹爪、抓手、手爪 |

### 复杂任务

| 指令示例 | 说明 |
|----------|------|
| 抓取物体 / 抓起杯子 | 执行抓取序列 |
| 放置物体 / 放下 | 执行放置序列 |
| 倒水 | 执行倒水动作序列 |
| 识别并抓取杯子 | 使用视觉检测并抓取 |

### 模式控制

| 指令示例 | 说明 |
|----------|------|
| 进入自由拖动模式 | 释放扭矩，可手动拖动 |
| 扫描电机 | 检测在线电机 |

## 生成的代码示例

输入: "张开夹爪"

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动生成的机械臂控制代码"""

import sys
import os
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from skills.arm_control.scripts.main import create_controller

def main():
    """张开夹爪"""
    ctrl = create_controller(port="COM7")
    ctrl.connect()

    try:
        print("执行: 张开夹爪...")
        ctrl.open_gripper(3000)
        print("完成！")

    finally:
        ctrl.disconnect()
        print("已断开连接")

if __name__ == "__main__":
    main()
```

## 架构说明

```
用户输入
    ↓
IntentParser (意图解析)
    ↓
CodeGenerator (代码生成)
    ↓
CodeExecutor (立即执行)
    ↓
保存到 script/ 目录 (可选)
```

## 测试

运行测试验证功能（无需硬件）：

```bash
python skills/nl-arm-control/evals/test_simple.py
```

## 注意事项

1. **串口连接**: 确保机械臂已连接到 COM7 端口
2. **安全范围**: 位置值自动限制在 0-4095 范围内
3. **扭矩管理**: 代码自动处理扭矩启用/禁用
4. **视觉检测**: 使用视觉功能需安装 ultralytics 或配置通义千问 API
