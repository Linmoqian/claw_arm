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

### Conda 创建环境失败？

尝试使用 pip 方式，或更新 Conda：

```bash
conda update conda
```
