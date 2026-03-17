# 开源文档实施计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 为 claw_arm 项目创建完整的开源文档体系，让有 Python 基础但无机器人经验的开发者快速上手

**Architecture:** 渐进式学习路径文档 + 分层示例代码 + 配置模板，采用"原理先行，实践紧随"的文档风格

**Tech Stack:** Markdown 文档、Python 示例代码、YAML 配置、Conda 环境

---

## Phase 1: P0 基础文件（开源必备）

### Task 1: 创建 requirements.txt

**Files:**
- Create: `requirements.txt`

**Step 1: 分析项目依赖**

查看现有代码的 import 语句，确认依赖列表：
- opencv-python (CV 模块)
- numpy (数值计算)
- pyserial (串口通信)
- ultralytics (YOLO 模型，可选)

**Step 2: 创建 requirements.txt**

```txt
# 核心依赖
opencv-python>=4.8.0
numpy>=1.24.0
pyserial>=3.5

# 可选依赖（用于 YOLO 目标检测）
# ultralytics>=8.0.0
```

**Step 3: 验证**

```bash
pip install -r requirements.txt --dry-run
```
Expected: 显示将要安装的包，无报错

**Step 4: 提交**

```bash
git add requirements.txt
git commit -m "chore: 添加 Python 依赖清单"
```

---

### Task 2: 创建 environment.yml

**Files:**
- Create: `environment.yml`

**Step 1: 创建 Conda 环境配置**

```yaml
name: claw_arm
channels:
  - conda-forge
  - defaults
dependencies:
  - python=3.10
  - pip
  - pip:
    - opencv-python>=4.8.0
    - numpy>=1.24.0
    - pyserial>=3.5
```

**Step 2: 验证语法**

```bash
conda env create -f environment.yml --dry-run
```
Expected: 显示将要创建的环境，无报错

**Step 3: 提交**

```bash
git add environment.yml
git commit -m "chore: 添加 Conda 环境配置"
```

---

### Task 3: 创建 config 目录和配置模板

**Files:**
- Create: `config/hardware.yaml`
- Create: `config/camera.yaml`

**Step 1: 创建 config 目录**

```bash
mkdir -p config
```

**Step 2: 创建 hardware.yaml**

```yaml
# 硬件配置模板
# 复制此文件为 hardware.local.yaml 并修改为你的实际配置

serial:
  port: "COM3"        # Windows: COM3, Linux: /dev/ttyUSB0
  baudrate: 1000000   # 波特率，SO100 默认 1M

motors:
  # 电机 ID 配置 (SO100 默认配置)
  - id: 1
    name: "base"        # 底座旋转
    min: 0
    max: 4095
  - id: 2
    name: "shoulder"    # 肩关节
    min: 0
    max: 4095
  - id: 3
    name: "elbow"       # 肘关节
    min: 0
    max: 4095
  - id: 4
    name: "wrist_pitch" # 腕俯仰
    min: 0
    max: 4095
  - id: 5
    name: "wrist_roll"  # 腕旋转
    min: 0
    max: 4095
  - id: 6
    name: "gripper"     # 夹爪
    min: 0
    max: 4095
```

**Step 3: 创建 camera.yaml**

```yaml
# 相机配置模板
# 复制此文件为 camera.local.yaml 并修改为你的实际配置

camera:
  index: 0              # 摄像头索引
  width: 1920           # 分辨率宽度
  height: 1080          # 分辨率高度
  fps: 30               # 帧率

# 相机内参（需要通过标定获取）
calibration:
  # 以下为示例值，请运行标定程序获取实际值
  camera_matrix:
    - [800.0, 0.0, 960.0]
    - [0.0, 800.0, 540.0]
    - [0.0, 0.0, 1.0]
  dist_coeffs:
    - [0.0, 0.0, 0.0, 0.0, 0.0]

  # 假设的固定高度（用于单目测距）
  fixed_height: 0.3     # 单位：米
```

**Step 4: 提交**

```bash
git add config/
git commit -m "chore: 添加配置文件模板"
```

---

### Task 4: 重写 README.md（英文版）

**Files:**
- Modify: `README.md`

**Step 1: 备份现有 README**

```bash
cp README.md README.md.bak
```

**Step 2: 编写新的 README.md**

```markdown
<div align="center">
  <img src="docs/images/logo.png" width="200" style="border-radius: 10%;">

  # Claw Arm

  **Intelligent Robot Arm Control System**

  Vision-guided grasping with natural language control

  [Demo Video](./movie.mp4) · [Documentation](docs/01-getting-started.md) · [中文文档](README_CN.md)
</div>

---

## Features

- 🎯 **Vision-Guided Grasping** - Detect and locate objects using computer vision
- 🗣️ **Natural Language Control** - Control the arm via Claude/OpenClaw skills
- 🔧 **Multiple Control Modes** - Interactive, teaching, calibration modes
- 🛡️ **Safety-First Design** - Built on the Three Laws of Robotics

## Hardware Requirements

| Component | Model | Note |
|-----------|-------|------|
| Robot Arm | SO-100 (6-DOF) | Feetech STS3215 servos |
| Camera | USB Webcam 1080p | Or RealSense D435 |

## Quick Start (5 min)

```bash
# Clone the repository
git clone https://github.com/your-username/claw_arm.git
cd claw_arm

# Create environment
conda env create -f environment.yml
conda activate claw_arm

# Test hardware connection
python script/1.3test_motors.py
python CV/1_camera_test.py

# Run your first grasp
python examples/basic/hello_arm.py
```

## Documentation

| Stage | Document | Goal |
|-------|----------|------|
| Setup | [Getting Started](docs/01-getting-started.md) | Environment ready |
| Hardware | [Hardware Setup](docs/02-hardware-setup.md) | Arm connected |
| Vision | [Vision Pipeline](docs/03-vision-pipeline.md) | Understand coordinates |
| Control | [Arm Control](docs/04-arm-control.md) | Master SDK usage |
| Grasp | [First Grasp](docs/06-first-grasp.md) | Complete workflow |

## Project Structure

```
claw_arm/
├── CV/           # Computer vision module
├── SDK/          # SCServo SDK wrapper
├── script/       # Debug and utility scripts
├── Joints/       # Joint control utilities
├── skills/       # Claude/OpenClaw skills
├── examples/     # Example code for beginners
└── docs/         # Documentation
```

## License

MIT License - feel free to use for personal or commercial projects.

## Acknowledgments

- [Feetech](https://www.feetechrc.com/) for STS3215 servos
- [LeRobot](https://github.com/huggingface/lerobot) for robotics framework inspiration
```

**Step 3: 提交**

```bash
git add README.md
git commit -m "docs: 重写英文 README，面向开源社区"
```

---

### Task 5: 创建 README_CN.md（中文详细版）

**Files:**
- Create: `README_CN.md`

**Step 1: 编写中文 README**

```markdown
<div align="center">
  <img src="docs/images/logo.png" width="200" style="border-radius: 10%;">

  # Claw Arm - 智能机械臂控制系统

  基于计算机视觉和自然语言控制的智能机械臂

  [观看演示](./movie.mp4) · [快速开始](#快速开始) · [English](README.md)
</div>

---

## 项目简介

Claw Arm 是一个完整的智能机械臂控制系统，结合了计算机视觉和精确控制技术。本项目面向有 Python 基础但无机器人开发经验的开发者，提供从零到一的完整学习路径。

### 核心特性

| 特性 | 说明 |
|------|------|
| 🎯 视觉引导抓取 | 通过摄像头识别物体位置，自动计算抓取坐标 |
| 🗣️ 自然语言控制 | 通过 Claude/OpenClaw 技能，用自然语言控制机械臂 |
| 🔧 多种控制模式 | 交互式控制、示教录制/回放、校准模式 |
| 🛡️ 安全设计 | 基于机器人三原则设计，用户安全优先 |

## 硬件准备

### 必需硬件

| 组件 | 型号 | 说明 | 参考价格 |
|------|------|------|----------|
| 机械臂 | SO-100 (6自由度) | 飞特 STS3215 舵机 | ¥1500-2000 |
| 摄像头 | USB 摄像头 1080p | 或 RealSense D435 | ¥100-500 |
| 电源 | 12V 5A | 为舵机供电 | ¥30 |

### 可选硬件

- 标定板（用于相机标定）
- 照明设备（提高视觉识别准确率）

## 快速开始

### 1. 克隆项目

```bash
git clone https://github.com/your-username/claw_arm.git
cd claw_arm
```

### 2. 创建环境

```bash
# 使用 Conda（推荐）
conda env create -f environment.yml
conda activate claw_arm

# 或使用 pip
pip install -r requirements.txt
```

### 3. 检查硬件连接

```bash
# 测试电机连接
python script/1.3test_motors.py

# 测试摄像头
python CV/1_camera_test.py
```

### 4. 运行第一个示例

```bash
python examples/basic/hello_arm.py
```

## 学习路径

我们为不同阶段的用户设计了渐进式学习路径：

| 阶段 | 文档 | 目标 | 预计时间 |
|------|------|------|----------|
| 入门 | [环境搭建](docs/01-getting-started.md) | 完成开发环境配置 | 15 分钟 |
| 基础 | [硬件连接](docs/02-hardware-setup.md) | 理解串口通信，连接机械臂 | 20 分钟 |
| 进阶 | [视觉模块](docs/03-vision-pipeline.md) | 理解坐标转换原理 | 30 分钟 |
| 实战 | [第一次抓取](docs/06-first-grasp.md) | 完成自动抓取任务 | 45 分钟 |
| 高级 | [二次开发](docs/07-advanced-dev.md) | 扩展自定义功能 | 持续 |

## 项目结构

```
claw_arm/
├── CV/                    # 计算机视觉模块
│   ├── 1_camera_test.py   # 摄像头测试
│   ├── 2_color_detection.py   # 颜色检测
│   └── ...                # 更多视觉脚本
├── SDK/                   # 舵机通信 SDK
├── script/                # 调试和工具脚本
├── Joints/                # 关节控制封装
├── skills/                # Claude/OpenClaw 技能
├── examples/              # 入门示例代码
│   ├── basic/             # 基础示例
│   ├── intermediate/      # 进阶示例
│   └── advanced/          # 高级示例
├── config/                # 配置文件模板
└── docs/                  # 详细文档
```

## 技术原理

### 系统架构

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   摄像头    │ ──▶ │  视觉处理   │ ──▶ │  坐标转换   │
└─────────────┘     └─────────────┘     └─────────────┘
                                              │
                                              ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   机械臂    │ ◀── │  运动控制   │ ◀── │  路径规划   │
└─────────────┘     └─────────────┘     └─────────────┘
```

### 坐标转换流程

1. **像素坐标** → 摄像头图像上的 (x, y)
2. **相机坐标** → 相对于摄像头的 (X, Y, Z)
3. **世界坐标** → 相对于机械臂基座的 (X, Y, Z)

## 常见问题

**Q: 电机连接不上？**
A: 检查串口号和波特率，确保驱动已安装。

**Q: 摄像头打不开？**
A: 尝试更改 `config/camera.yaml` 中的摄像头索引。

**Q: 检测不到物体？**
A: 检查光照条件，调整 HSV 颜色阈值。

更多问题请查看 [故障排除](docs/08-troubleshooting.md)

## 贡献指南

欢迎提交 Issue 和 Pull Request！

## 开源协议

MIT License - 可自由用于个人或商业项目。

## 致谢

- [飞特](https://www.feetechrc.com/) 提供 STS3215 舵机
- [LeRobot](https://github.com/huggingface/lerobot) 机器人学习框架启发
```

**Step 2: 提交**

```bash
git add README_CN.md
git commit -m "docs: 添加中文详细版 README"
```

---

## Phase 2: P0 核心文档

### Task 6: 创建 01-getting-started.md

**Files:**
- Create: `docs/01-getting-started.md`

**Step 1: 编写环境搭建文档**

```markdown
# 环境搭建

本文档帮助你完成开发环境的配置。

## 为什么需要虚拟环境？

Python 项目通常依赖多个第三方库。使用虚拟环境可以：
- 隔离不同项目的依赖，避免版本冲突
- 方便复现和部署
- 保持系统 Python 环境的整洁

## 前置条件

- Python 3.10+ 已安装
- Conda 或 pip 可用
- Git 已安装

## 方式一：使用 Conda（推荐）

Conda 是一个流行的 Python 环境管理工具。

```bash
# 1. 创建环境
conda env create -f environment.yml

# 2. 激活环境
conda activate claw_arm

# 3. 验证安装
python -c "import cv2; print(f'OpenCV: {cv2.__version__}')"
python -c "import serial; print('pyserial: OK')"
```

## 方式二：使用 pip

如果你更习惯使用 pip：

```bash
# 1. 创建虚拟环境
python -m venv venv

# 2. 激活环境
# Windows:
venv\Scripts\activate
# Linux/macOS:
source venv/bin/activate

# 3. 安装依赖
pip install -r requirements.txt

# 4. 验证安装
python -c "import cv2; print(f'OpenCV: {cv2.__version__}')"
```

## 依赖说明

| 包名 | 用途 |
|------|------|
| opencv-python | 图像处理、摄像头操作 |
| numpy | 数值计算、矩阵运算 |
| pyserial | 串口通信，与机械臂交互 |

## 下一步

环境准备就绪后，继续阅读 [硬件连接](02-hardware-setup.md)。

## 常见问题

### pip 安装速度慢？

使用国内镜像源：

```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### Windows 上找不到模块？

确保已激活虚拟环境，命令行前会显示 `(claw_arm)` 或 `(venv)`。
```

**Step 2: 提交**

```bash
git add docs/01-getting-started.md
git commit -m "docs: 添加环境搭建文档"
```

---

### Task 7: 创建 02-hardware-setup.md

**Files:**
- Create: `docs/02-hardware-setup.md`

**Step 1: 编写硬件连接文档**

```markdown
# 硬件连接

本文档指导你连接 SO-100 机械臂和摄像头。

## 串口通信原理

机械臂通过**串口**（Serial Port）与电脑通信。串口是一种逐位传输数据的通信方式：
- 数据通过 TX（发送）和 RX（接收）两根线传输
- 波特率决定传输速度（SO100 使用 1000000 bps）
- 每个舵机有唯一的 ID（1-6）

## 硬件连接步骤

### 1. 连接机械臂

```
电脑 USB ──▶ USB转TTL模块 ──▶ 舵机控制板 ──▶ 6个舵机
                                      │
                                   12V电源
```

1. 将 USB 转 TTL 模块插入电脑
2. 连接 TTL 模块到舵机控制板（注意 TX/RX 交叉）
3. 连接 12V 电源到舵机控制板
4. 确认电源指示灯亮起

### 2. 查找串口号

**Windows:**

```bash
# 在设备管理器中查看
# 或使用 Python
python -c "import serial.tools.list_ports; [print(p.device) for p in serial.tools.list_ports.comports()]"
```

常见串口号：`COM3`、`COM4`、`COM5`

**Linux:**

```bash
ls /dev/ttyUSB*
# 常见：/dev/ttyUSB0
```

### 3. 配置串口

编辑 `config/hardware.yaml`：

```yaml
serial:
  port: "COM3"        # 改为你的串口号
  baudrate: 1000000   # 保持不变
```

### 4. 测试连接

```bash
python script/1.3test_motors.py
```

预期输出：
```
[>>] 正在连接 COM3 @ 1000000...
[OK] 连接成功
[OK] 发现电机 1: base
[OK] 发现电机 2: shoulder
...
```

## 摄像头连接

### 1. 连接摄像头

将 USB 摄像头插入电脑 USB 接口。

### 2. 测试摄像头

```bash
python CV/1_camera_test.py
```

预期：弹出摄像头画面窗口。

### 3. 配置摄像头

如果摄像头索引不是 0，编辑 `config/camera.yaml`：

```yaml
camera:
  index: 1  # 尝试 0, 1, 2...
```

## 常见问题

### 串口打开失败？

1. 检查串口是否被其他程序占用
2. 确认驱动已安装（CH340、CP2102 等）
3. 尝试以管理员权限运行

### 找不到电机？

1. 检查电源是否接通
2. 确认波特率设置正确（1000000）
3. 检查 TTL 线是否接反

### 摄像头黑屏？

1. 尝试不同的摄像头索引
2. 检查摄像头是否被其他程序占用
3. 确认摄像头驱动已安装

## 下一步

硬件连接成功后，继续阅读 [视觉模块](03-vision-pipeline.md) 了解坐标转换原理。
```

**Step 2: 提交**

```bash
git add docs/02-hardware-setup.md
git commit -m "docs: 添加硬件连接文档"
```

---

## Phase 3: P1 核心文档

### Task 8: 创建 03-vision-pipeline.md

**Files:**
- Create: `docs/03-vision-pipeline.md`

**Step 1: 编写视觉模块文档**

（文档内容较长，包含坐标转换原理、5 个脚本说明、实践步骤）

**Step 2: 提交**

```bash
git add docs/03-vision-pipeline.md
git commit -m "docs: 添加视觉模块文档"
```

---

### Task 9: 创建 06-first-grasp.md

**Files:**
- Create: `docs/06-first-grasp.md`

**Step 1: 编写第一次抓取文档**

**Step 2: 提交**

```bash
git add docs/06-first-grasp.md
git commit -m "docs: 添加第一次抓取教程"
```

---

## Phase 4: P1 示例代码

### Task 10: 创建 examples 目录结构

**Files:**
- Create: `examples/basic/hello_arm.py`
- Create: `examples/basic/detect_color.py`
- Create: `examples/basic/read_joint.py`

**Step 1: 创建目录**

```bash
mkdir -p examples/basic examples/intermediate examples/advanced
```

**Step 2: 创建 hello_arm.py**

**Step 3: 创建 detect_color.py**

**Step 4: 创建 read_joint.py**

**Step 5: 提交**

```bash
git add examples/
git commit -m "feat: 添加基础示例代码"
```

---

## Phase 5: P2 进阶文档

### Task 11-15: 创建进阶文档

- `04-arm-control.md`
- `05-calibration.md`
- `07-advanced-dev.md`
- `08-troubleshooting.md`

---

## 验证清单

完成所有任务后，验证：

- [ ] `pip install -r requirements.txt` 无报错
- [ ] `conda env create -f environment.yml` 无报错
- [ ] 所有文档链接可访问
- [ ] 示例代码可运行
- [ ] README 在 GitHub 上显示正常

---

*计划创建日期：2026-03-17*
