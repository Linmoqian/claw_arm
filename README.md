<div align="center">
  <img src="docs/refences/xiakao_logo.png" width="200" style="border-radius: 10%;">

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

# Run your first example
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
├── config/       # Configuration templates
└── docs/         # Documentation
```

## Architecture

### Traditional Approach (Current)

```
Camera → Vision Processing → Coordinate Transform → Motion Control → Robot Arm
```

### Embodied AI (Future)

Based on [LeRobot](https://github.com/huggingface/lerobot) framework with VLA models.

## License

MIT License - feel free to use for personal or commercial projects.

## Acknowledgments

- [Feetech](https://www.feetechrc.com/) for STS3215 servos
- [LeRobot](https://github.com/huggingface/lerobot) for robotics framework inspiration
