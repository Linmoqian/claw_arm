<div align="center">
  <img src="docs/refences/xiakao_logo.png" width="200" style="border-radius: 10%;">

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

## 自然语言控制

将 `skills/nature_arm` 复制到你的 Claude Code 或 OpenClaw skills 目录，然后：

```
/nature_arm 跳个舞
/nature_arm 抓取红色物体
```

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
