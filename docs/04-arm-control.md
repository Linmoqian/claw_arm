# 机械臂控制

本文档深入介绍 SO-100 机械臂的控制原理和 SDK 使用方法。

## 舵机通信协议

SO-100 使用飞特 STS3215 舵机，采用半双工串行通信协议：

```
┌─────────────────────────────────────────────────────┐
│  数据包结构                                          │
├─────────────────────────────────────────────────────┤
│  Header (0xFF 0xFF) │ ID │ Length │ Instruction │ Parameters │ Checksum │
└─────────────────────────────────────────────────────┘
```

### 关键寄存器

| 地址 | 名称 | 说明 | 范围 |
|------|------|------|------|
| 40 | TORQUE_ENABLE | 扭矩开关 | 0=关, 1=开 |
| 42 | GOAL_POSITION | 目标位置 | 0-4095 |
| 56 | PRESENT_POSITION | 当前位置 | 0-4095 |
| 44 | GOAL_TIME | 运动时间 | 0-65535 ms |

## SDK 使用

### 初始化连接

```python
from SDK import PortHandler, PacketHandler

# 创建端口处理器
port = PortHandler("COM3")
port.setBaudRate(1000000)

# 打开端口
if not port.openPort():
    print("无法打开串口")
    exit(1)

# 创建数据包处理器
pkt = PacketHandler(0.0)  # 0.0 = 协议版本
```

### 控制单个舵机

```python
MOTOR_ID = 1

# 启用扭矩
pkt.write1ByteTxRx(port, MOTOR_ID, 40, 1)

# 移动到位置 2048（中间位置）
pkt.write2ByteTxRx(port, MOTOR_ID, 42, 2048)

# 读取当前位置
position, result, _ = pkt.read2ByteTxRx(port, MOTOR_ID, 56)
print(f"当前位置: {position}")

# 禁用扭矩
pkt.write1ByteTxRx(port, MOTOR_ID, 40, 0)

# 关闭端口
port.closePort()
```

### 批量控制

使用 `GroupSyncWrite` 同时控制多个舵机：

```python
from SDK import GroupSyncWrite

# 创建批量写入对象
group = GroupSyncWrite(port, pkt, 42, 2)  # 地址42，长度2字节

# 添加目标位置
group.addParam(1, [2048 & 0xFF, 2048 >> 8])  # 电机1
group.addParam(2, [1024 & 0xFF, 1024 >> 8])  # 电机2

# 执行
group.txPacket()
group.clearParam()
```

## 运动学基础

### 正向运动学

已知关节角度，计算末端位置：

```
P = f(θ₁, θ₂, θ₃, θ₄, θ₅, θ₆)
```

### 逆向运动学

已知目标位置，计算关节角度：

```
(θ₁, θ₂, ...) = f⁻¹(X, Y, Z)
```

本项目使用数值迭代方法求解逆运动学。

## 运动控制类

使用 `Joint` 类简化控制：

```python
from Joints.simple_joint import Joint

# 创建关节对象
joint = Joint(port, pkt, joint_id=1)

# 启用
joint.enable()

# 读取位置
pos = joint.position()

# 移动
joint.move(2048)

# 禁用
joint.disable()
```

## 安全限制

### 关节限位

每个关节都有物理限位，超出范围会损坏机械结构：

```python
def safe_move(joint, position):
    position = max(joint.min, min(joint.max, position))
    joint.move(position)
```

### 速度限制

过快的运动会损坏舵机：

```python
# 使用 GOAL_TIME 寄存器控制速度
pkt.write2ByteTxRx(port, MOTOR_ID, 44, 1000)  # 1000ms 完成动作
```

## 常见问题

### 舵机不响应？

1. 检查 ID 是否正确
2. 检查波特率是否匹配
3. 检查电源是否正常

### 运动抖动？

1. 检查电源电压是否稳定
2. 降低运动速度
3. 检查机械连接是否松动

### 位置不准确？

1. 检查零点校准
2. 检查机械间隙
3. 使用闭环控制

## 下一步

- [相机标定](05-calibration.md) - 提高定位精度
- [第一次抓取](06-first-grasp.md) - 实践应用
