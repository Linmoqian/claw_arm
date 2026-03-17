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
