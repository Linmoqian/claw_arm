<div align="center">

  <img src="docs/refences/xiakao_logo.png" width="180" style="border-radius: 15%;">

  # Claw Arm

  ### **Make Your Robot Arm Understand You**

  <i>See - Understand - Act</i>

  <img src="movie.gif" width="100%">

  <p>

  <a href="docs/01-getting-started.md">
    <img src="https://img.shields.io/badge/Docs-Get_Started-4ECDC4?style=for-the-badge" alt="Docs">
  </a>
  <a href="README_CN.md">
    <img src="https://img.shields.io/badge/Chinese-中文文档-FFE66D?style=for-the-badge" alt="Chinese">
  </a>

</div>

---

## Overview

**Claw Arm** is an open-source project that makes robot arms "smart" - it can see, understand, and act.

```
You say: "Pick up the red cube"
It does: Detect cube -> Calculate position -> Grasp precisely
```

## Key Features

<table>
<tr>
<td width="50%">

### Natural Language Control

```bash
/nature_arm dance
/nature_arm grab the red object
/nature_arm place the cube on the left
```

Control the arm using natural language via Claude Code / OpenClaw

</td>
<td width="50%">

### Vision-Guided Grasping

```
Camera -> Color Detection -> Coordinate Transform -> Motion Control -> Done
```

</td>
</tr>
<tr>
<td>

### Multiple Control Modes

| Mode | Purpose |
|------|---------|
| Interactive | Manual control |
| Teaching | Record & playback |
| Calibration | Fine-tuning |

</td>
<td>

### Safety-First Design

Built on the Three Laws of Robotics:
1. User safety first
2. Self-preservation second
3. Follow user commands

</td>
</tr>
</table>

## Quick Start (5 min)

```bash
# 1. Clone the repository
git clone https://github.com/Linmoqian/claw_arm.git
cd claw_arm

# 2. Setup environment
conda env create -f environment.yml
conda activate claw_arm

# 3. Test hardware
python script/1.3test_motors.py  # Motor test
python CV/1_camera_test.py       # Camera test

# 4. Run your first example
python examples/basic/hello_arm.py
```

> First time? Follow the [Getting Started Guide](docs/01-getting-started.md) step by step

## Hardware Requirements

| Component | Recommended | Price Range |
|-----------|-------------|-------------|
| Robot Arm | SO-100 (6-DOF) | $200-280 |
| Camera | USB 1080p / RealSense D435 | $15-70 |
| Power Supply | 12V 5A | $5 |

> On a budget? Minimum setup requires only the arm + basic webcam

## Learning Path

A progressive learning path designed for beginners:

```
 Beginner        Basic           Advanced        Practical
    |              |                 |               |
    v              v                 v               v
 Setup Env  ->  Connect HW  ->   Understand   ->   First Grasp
               Hardware        Vision
```

| Stage | Document | You Will Learn |
|-------|----------|----------------|
| Beginner | [Getting Started](docs/01-getting-started.md) | Setup development environment |
| Basic | [Hardware Setup](docs/02-hardware-setup.md) | Connect the robot arm |
| Advanced | [Vision Pipeline](docs/03-vision-pipeline.md) | Understand coordinate transform |
| Practical | [First Grasp](docs/06-first-grasp.md) | Complete auto-grasping |
| Expert | [Advanced Dev](docs/07-advanced-dev.md) | Build custom features |

## Project Structure

```
claw_arm/
├── CV/              # Computer vision module
│   ├── 1_camera_test.py
│   ├── 2_color_detection.py
│   └── ...
├── SDK/             # Servo communication SDK
├── script/          # Debug scripts
├── Joints/          # Joint control
├── skills/          # Natural language skills
├── examples/        # Beginner examples
│   ├── basic/       # Basic examples
│   ├── intermediate/# Intermediate examples
│   └── advanced/    # Advanced examples
├── config/          # Config templates
└── docs/            # Documentation
```

## Architecture

### System Flow

```
Camera -> Vision Processing -> Coordinate Transform -> Motion Control -> Robot Arm
```

### Coordinate Transform

| Step | Coordinate System | Example |
|------|-------------------|---------|
| 1. Pixel | Image plane | (640, 360) |
| 2. Camera | Relative to camera | (0.1, 0.05, 0.5) m |
| 3. World | Relative to base | (0.15, 0.08, 0.3) m |

## Roadmap

- [x] Vision-guided grasping
- [x] Natural language control
- [x] Complete documentation
- [ ] YOLO object detection
- [ ] LeRobot embodied AI
- [ ] Web control interface

## Contributing

All forms of contribution are welcome:

- Report bugs: [Open an Issue](../../issues)
- Suggestions: [Join Discussion](../../discussions)
- Code: [Submit a PR](../../pulls)

## License

[MIT License](LICENSE) - Free for personal or commercial use.

## Acknowledgments

- [Feetech](https://www.feetechrc.com/) - STS3215 servos
- [LeRobot](https://github.com/huggingface/lerobot) - Embodied AI inspiration
- All contributors

---

<div align="center">

**If this project helps you, please give it a Star**

</div>
