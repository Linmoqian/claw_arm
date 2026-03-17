# 开源文档设计方案

## 项目背景

将 claw_arm 智能机械臂项目对外开放，让有 Python 基础但无机器人经验的开发者快速上手。

## 目标用户

- 有 Python 编程基础
- 无机器人/硬件开发经验
- 希望学习视觉引导机械臂控制

## 设计目标

完整学习路径：**演示 → 理解 → 开发**

## 硬件要求

- SO-100 机械臂（6 自由度）
- USB 摄像头（1080p）

## 文档风格

- 原理+实践结合（先解释为什么，再教怎么做）
- 中英双语

---

## 文档架构

### README 结构

```
README.md          # 英文主页面（简洁）
README_CN.md       # 中文详细版
```

**README.md 内容要点**：
- 项目简介 + 效果 GIF
- 硬件清单表格
- 5 分钟快速开始命令
- 文档链接索引
- MIT License

**README_CN.md 内容要点**：
- 详细项目介绍
- 硬件准备指南
- 详细安装步骤
- 学习路径表格

### docs/ 目录结构

| 文档 | 核心内容 | 原理讲解 | 实践步骤 |
|------|----------|----------|----------|
| 01-getting-started.md | 环境搭建 | Python 虚拟环境、依赖作用 | conda/pip 安装命令 |
| 02-hardware-setup.md | 硬件连接 | 串口通信原理、USB 设备识别 | 连接步骤、端口配置 |
| 03-vision-pipeline.md | 视觉模块 | 像素坐标→相机坐标→世界坐标 | 5 个脚本依次运行 |
| 04-arm-control.md | 机械臂控制 | 舵机协议、运动学基础 | SDK 使用、关节控制 |
| 05-calibration.md | 标定流程 | 手眼标定原理、相机内参 | 标定板操作步骤 |
| 06-first-grasp.md | 第一次抓取 | 完整流程串联 | 从视觉到抓取的端到端 |
| 07-advanced-dev.md | 二次开发 | 代码架构、扩展点 | 自定义 Skill 编写 |
| 08-troubleshooting.md | 常见问题 | 问题根因分析 | 解决方案 |

### examples/ 目录结构

```
examples/
├── basic/
│   ├── hello_arm.py           # 最简示例：让机械臂动一下
│   ├── detect_color.py        # 颜色检测入门
│   └── read_joint.py          # 读取关节角度
├── intermediate/
│   ├── coordinate_transform.py # 坐标转换演示
│   ├── teach_mode.py          # 示教模式
│   └── calibration_helper.py  # 标定辅助工具
└── advanced/
    ├── auto_grasp.py          # 完整自动抓取
    └── custom_skill.py        # 自定义 Skill 示例
```

### 配套文件

| 文件 | 作用 |
|------|------|
| requirements.txt | Python 依赖清单 |
| environment.yml | Conda 环境配置 |
| config/hardware.yaml | 硬件配置模板 |
| config/camera.yaml | 相机参数模板 |
| docs/images/ | 效果图、架构图、流程图 |

### 媒体资源

| 资源 | 用途 |
|------|------|
| docs/images/demo.gif | README 顶部动图 |
| docs/images/architecture.png | 系统架构图 |
| docs/images/coordinate-systems.png | 坐标系图解 |
| docs/images/hardware-connection.jpg | 硬件连接示意图 |

---

## 实施优先级

### P0 必须（开源基本要求）

- [ ] README_CN.md
- [ ] requirements.txt
- [ ] environment.yml
- [ ] 01-getting-started.md
- [ ] 02-hardware-setup.md

### P1 重要（核心功能体验）

- [ ] 03-vision-pipeline.md
- [ ] 06-first-grasp.md
- [ ] examples/basic/

### P2 建议（深度学习）

- [ ] 04-arm-control.md
- [ ] 05-calibration.md
- [ ] 07-advanced-dev.md
- [ ] 08-troubleshooting.md
- [ ] examples/intermediate/
- [ ] examples/advanced/

---

## 示例代码规范

每个示例包含：
1. 顶部文档字符串（功能+学习目标+前置条件）
2. 详细中文注释
3. 关键参数可配置（命令行或配置文件）

```python
"""
hello_arm.py - 最简机械臂控制示例

功能：让机械臂移动到预设位置

学习目标：
  - 理解 SDK 初始化流程
  - 掌握关节控制基础

硬件要求：SO-100 机械臂已连接
"""
```

---

## 文档模板要点

每篇文档遵循：
1. **为什么需要** - 原理讲解
2. **概念图解** - 可视化说明
3. **处理流程** - 步骤概览
4. **动手实践** - 具体命令和代码
5. **常见问题** - FAQ

---

*设计日期：2026-03-17*
