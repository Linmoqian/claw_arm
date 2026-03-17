<div align="center">

  <img src="docs/refences/xiakao_logo.png" width="180" style="border-radius: 15%;">

  # Claw Arm

  ### **让机械臂听懂你的话**

  <i>看到 - 理解 - 行动</i>

  <p>

  <a href="./movie.mp4">
    <img src="https://img.shields.io/badge/Demo-观看演示-FF6B6B?style=for-the-badge" alt="Demo">
  </a>
  <a href="docs/01-getting-started.md">
    <img src="https://img.shields.io/badge/Docs-快速开始-4ECDC4?style=for-the-badge" alt="Docs">
  </a>
  <a href="README_CN.md">
    <img src="https://img.shields.io/badge/中文-详细文档-FFE66D?style=for-the-badge" alt="Chinese">
  </a>

</div>

---

## 简介

**Claw Arm** 是一个让机械臂"变聪明"的开源项目——它能看到、能理解、能行动。

```
你说："抓起那个红色的方块"
它做：识别方块 -> 计算位置 -> 精准抓取
```

## 核心特性

<table>
<tr>
<td width="50%">

### 自然语言控制

```bash
/nature_arm 跳个舞
/nature_arm 抓取红色物体
/nature_arm 把方块放到左边
```

结合 Claude Code / OpenClaw，用自然语言控制机械臂

</td>
<td width="50%">

### 视觉引导抓取

```
摄像头 -> 颜色检测 -> 坐标转换 -> 机械臂运动 -> 完成
```

</td>
</tr>
<tr>
<td>

### 多种控制模式

| 模式 | 用途 |
|------|------|
| 交互式 | 手动控制 |
| 示教 | 录制回放 |
| 校准 | 精度调优 |

</td>
<td>

### 安全设计

基于机器人三原则设计：
1. 用户安全第一
2. 自身安全第二
3. 服从用户指令

</td>
</tr>
</table>

## 5 分钟快速开始

```bash
# 1. 克隆项目
git clone https://github.com/your-username/claw_arm.git
cd claw_arm

# 2. 一键配置环境
conda env create -f environment.yml
conda activate claw_arm

# 3. 测试硬件
python script/1.3test_motors.py  # 电机测试
python CV/1_camera_test.py       # 摄像头测试

# 4. 运行第一个示例
python examples/basic/hello_arm.py
```

> 第一次？跟着 [快速开始指南](docs/01-getting-started.md) 一步步来

## 硬件清单

| 组件 | 推荐型号 | 价格参考 |
|------|----------|----------|
| 机械臂 | SO-100 (6自由度) | 1500-2000 |
| 摄像头 | USB 1080p / RealSense D435 | 100-500 |
| 电源 | 12V 5A | 30 |

> 预算有限？最小配置只需机械臂+普通摄像头

## 学习路径

为新手设计的渐进式学习路径：

```
入门          基础           进阶           实战
 |             |              |              |
 v             v              v              v
环境搭建  ->  硬件连接  ->   视觉理解  ->  第一次抓取
```

| 阶段 | 文档 | 你将学会 |
|------|------|----------|
| 入门 | [环境搭建](docs/01-getting-started.md) | 配置开发环境 |
| 基础 | [硬件连接](docs/02-hardware-setup.md) | 连接机械臂 |
| 进阶 | [视觉模块](docs/03-vision-pipeline.md) | 理解坐标转换 |
| 实战 | [第一次抓取](docs/06-first-grasp.md) | 完成自动抓取 |
| 高级 | [二次开发](docs/07-advanced-dev.md) | 自定义功能 |

## 项目结构

```
claw_arm/
├── CV/              # 计算机视觉模块
│   ├── 1_camera_test.py
│   ├── 2_color_detection.py
│   └── ...
├── SDK/             # 舵机通信 SDK
├── script/          # 调试脚本
├── Joints/          # 关节控制
├── skills/          # 自然语言技能
├── examples/        # 入门示例
│   ├── basic/       # 基础示例
│   ├── intermediate/# 进阶示例
│   └── advanced/    # 高级示例
├── config/          # 配置模板
└── docs/            # 详细文档
```

## 技术架构

### 系统流程

```
摄像头 -> 视觉处理 -> 坐标转换 -> 运动控制 -> 机械臂
```

### 坐标转换

| 步骤 | 坐标系 | 示例 |
|------|--------|------|
| 1. 像素坐标 | 图像平面 | (640, 360) |
| 2. 相机坐标 | 相对摄像头 | (0.1, 0.05, 0.5) m |
| 3. 世界坐标 | 相对基座 | (0.15, 0.08, 0.3) m |

## 路线图

- [x] 视觉引导抓取
- [x] 自然语言控制
- [x] 完整文档体系
- [ ] YOLO 目标检测
- [ ] LeRobot 具身智能
- [ ] Web 控制界面

## 参与贡献

欢迎所有形式的贡献：

- 报告 Bug：[提交 Issue](../../issues)
- 提建议：[参与讨论](../../discussions)
- 贡献代码：[提交 PR](../../pulls)

## 开源协议

[MIT License](LICENSE) - 可自由用于个人或商业项目。

## 致谢

- [Feetech](https://www.feetechrc.com/) - STS3215 舵机
- [LeRobot](https://github.com/huggingface/lerobot) - 具身智能启发
- 所有贡献者

---

<div align="center">

**如果这个项目对你有帮助，给个 Star 支持一下**

</div>
