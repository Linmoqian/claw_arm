name: scservo-controller-SDK
description: 使用 SCServo SDK 控制机械臂/夹爪应用中的 SC 舵机。凡是涉及舵机控制、机械臂运动、夹爪开合、串口舵机读写、速度控制、多舵机同步控制或状态监测的任务，都应使用此技能。

# SC 舵机控制器

此技能用于通过 SCServo SDK 控制 SC 舵机（SDK 位于 `D:\project\claw_arm\SDK`）。该 SDK 提供与舵机串口通信的能力，可用于机械臂和夹爪控制。

## SDK 组件

- **PortHandler**：串口通信（打开/关闭串口、设置波特率、读写）
- **ProtocolPacketHandler**：协议包操作（ping、读、写、同步操作）
- **GroupSyncRead**：多舵机同步读取
- **GroupSyncWrite**：多舵机同步写入

## 基本流程

1. **导入 SDK**
   ```python
   import sys
   sys.path.append('D:\\project\\claw_arm\\SDK')
   from scservo_def import *
   from port_handler import PortHandler
   from packet_handler import PacketHandler
   from group_sync_read import GroupSyncRead
   from group_sync_write import GroupSyncWrite
   ```

2. **连接舵机**
   ```python
    # 打开串口（通常为 COM3、COM4、/dev/ttyUSB0 等）
   portHandler = PortHandler("COM3")  # Windows
   # portHandler = PortHandler("/dev/ttyUSB0")  # Linux
   if not portHandler.openPort():
       print("Failed to open port")
       sys.exit()

    # 设置波特率（默认值 1000000）
   portHandler.setBaudRate(1000000)

    # 创建数据包处理器（end: 0 表示小端）
   packetHandler = PacketHandler(0)
   ```

3. **Ping 验证连接**
   ```python
    scs_id = 1  # 舵机 ID（1-252）
   model_number, result, error = packetHandler.ping(portHandler, scs_id)
   if result != COMM_SUCCESS:
       print(f"Ping failed: {packetHandler.getTxRxResult(result)}")
   else:
       print(f"Connected! Model: {model_number}")
   ```

## 常用控制表地址

控制表通过内存地址读写舵机参数，常见地址如下：

| Address | Name | Size | Description |
|---------|------|------|-------------|
| 3 | Model Number | 2 | 型号信息 |
| 11 | Position | 2 | 当前位置（0-1000） |
| 12 | Speed | 2 | 当前速度 |
| 20 | Goal Position | 2 | 目标位置 |
| 21 | Moving Speed | 2 | 目标运动速度 |
| 22 | Torque Limit | 2 | 最大扭矩 |
| 23 | Acceleration | 2 | 加速度设置 |
| 24 | LED | 1 | LED 控制 |
| 25 | LED Control | 1 | LED 错误显示 |

**注意**：不同舵机型号的地址可能不同，请始终以对应型号的数据手册为准。

## 核心操作

### 读取位置
```python
# 读取当前位置（地址 11，2 字节）
position, result, error = packetHandler.read2ByteTxRx(portHandler, scs_id, 11)
if result == COMM_SUCCESS:
    print(f"Position: {position}")
```

### 写入目标位置
```python
# 移动到位置 500（地址 20，2 字节）
goal_pos = 500
result, error = packetHandler.write2ByteTxRx(portHandler, scs_id, 20, goal_pos)
if result != COMM_SUCCESS:
    print(f"Write failed: {packetHandler.getTxRxResult(result)}")
```

### 设置运动速度
```python
# 设置速度（地址 21，2 字节）
speed = 500  # 范围 0-1000
result, error = packetHandler.write2ByteTxRx(portHandler, scs_id, 21, speed)
```

### 设置扭矩限制
```python
# 设置扭矩限制（地址 22，2 字节）
torque = 800  # 范围 0-1000
result, error = packetHandler.write2ByteTxRx(portHandler, scs_id, 22, torque)
```

### 设置加速度
```python
# 设置加速度（地址 23，2 字节）
accel = 50  # 范围 0-255
result, error = packetHandler.write1ByteTxRx(portHandler, scs_id, 23, accel)
```

### 控制 LED
```python
# 打开 LED（地址 24，1 字节）
result, error = packetHandler.write1ByteTxRx(portHandler, scs_id, 24, 1)
```

## 多舵机同步控制

### 同步写（同时驱动多个舵机）
```python
# 创建用于目标位置的同步写处理器（地址 20，2 字节）
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 20, 2)

# 为每个舵机添加参数
groupSyncWrite.addParam(1, [SCS_LOBYTE(500), SCS_HIBYTE(500)])  # ID 1 到位置 500
groupSyncWrite.addParam(2, [SCS_LOBYTE(700), SCS_HIBYTE(700)])  # ID 2 到位置 700
groupSyncWrite.addParam(3, [SCS_LOBYTE(300), SCS_HIBYTE(300)])  # ID 3 到位置 300

# 一次性发送到所有舵机
result = groupSyncWrite.txPacket()
if result != COMM_SUCCESS:
    print(f"Sync write failed: {packetHandler.getTxRxResult(result)}")

# 清理参数，便于下次使用
groupSyncWrite.clearParam()
```

### 同步读（读取多个舵机）
```python
# 创建用于读取位置的同步读处理器（地址 11，2 字节）
groupSyncRead = GroupSyncRead(portHandler, packetHandler, 11, 2)

# 添加需要读取的舵机 ID
groupSyncRead.addParam(1)
groupSyncRead.addParam(2)
groupSyncRead.addParam(3)

# 发送读取请求
result = groupSyncRead.txRxPacket()
if result == COMM_SUCCESS:
    # 分别读取每个舵机的位置
    pos1 = groupSyncRead.getData(1, 11, 2)
    pos2 = groupSyncRead.getData(2, 11, 2)
    pos3 = groupSyncRead.getData(3, 11, 2)
    print(f"Positions: {pos1}, {pos2}, {pos3}")
```

## 错误处理

### 检查通信结果
```python
if result != COMM_SUCCESS:
    error_msg = packetHandler.getTxRxResult(result)
    print(f"Communication error: {error_msg}")
    # COMM_SUCCESS = 0, COMM_PORT_BUSY = -1, COMM_TX_FAIL = -2,
    # COMM_RX_FAIL = -3, COMM_TX_ERROR = -4, COMM_RX_TIMEOUT = -6
```

### 检查硬件错误
```python
if error != 0:
    error_msg = packetHandler.getRxPacketError(error)
    print(f"Hardware error: {error_msg}")
    # ERRBIT_VOLTAGE = 1, ERRBIT_ANGLE = 2, ERRBIT_OVERHEAT = 4,
    # ERRBIT_OVERELE = 8, ERRBIT_OVERLOAD = 32
```

## 清理
```python
# 完成后务必关闭串口
portHandler.closePort()
```

## 完整示例：单舵机控制
```python
import sys
sys.path.append('D:\\project\\claw_arm\\SDK')
from scservo_def import *
from port_handler import PortHandler
from packet_handler import PacketHandler

# 初始化
portHandler = PortHandler("COM3")
packetHandler = PacketHandler(0)

# 连接
if not portHandler.openPort():
    print("Failed to open port")
    sys.exit()

# Ping 舵机
scs_id = 1
model, result, error = packetHandler.ping(portHandler, scs_id)
if result != COMM_SUCCESS:
    print(f"Ping failed: {packetHandler.getTxRxResult(result)}")
    portHandler.closePort()
    sys.exit()

print(f"Connected to servo {scs_id}, model {model}")

# 读取当前位置
pos, result, error = packetHandler.read2ByteTxRx(portHandler, scs_id, 11)
print(f"Current position: {pos}")

# 移动到位置 500
result, error = packetHandler.write2ByteTxRx(portHandler, scs_id, 20, 500)
if result == COMM_SUCCESS:
    print("Moving to position 500...")

# 等待后再读取新位置
import time
time.sleep(1)
pos, result, error = packetHandler.read2ByteTxRx(portHandler, scs_id, 11)
print(f"New position: {pos}")

# 清理
portHandler.closePort()
```

## 完整示例：多舵机机械臂控制
```python
import sys
sys.path.append('D:\\project\\claw_arm\\SDK')
from scservo_def import *
from port_handler import PortHandler
from packet_handler import PacketHandler
from group_sync_write import GroupSyncWrite
import time

# 初始化
portHandler = PortHandler("COM3")
packetHandler = PacketHandler(0)

# 连接
if not portHandler.openPort():
    print("Failed to open port")
    sys.exit()

# 配置用于目标位置的同步写
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 20, 2)

# 机械臂配置（底座、肩部、肘部、腕部、夹爪）
arm_positions = {
    1: 500,  # 底座舵机
    2: 600,  # 肩部
    3: 400,  # 肘部
    4: 700,  # 腕部
    5: 800   # 夹爪
}

# 设置所有目标位置
for scs_id, pos in arm_positions.items():
    groupSyncWrite.addParam(scs_id, [SCS_LOBYTE(pos), SCS_HIBYTE(pos)])

result = groupSyncWrite.txPacket()
if result == COMM_SUCCESS:
    print("Arm moved to target positions")
else:
    print(f"Failed: {packetHandler.getTxRxResult(result)}")

groupSyncWrite.clearParam()
time.sleep(2)

# 关闭夹爪
groupSyncWrite.addParam(5, [SCS_LOBYTE(200), SCS_HIBYTE(200)])
groupSyncWrite.txPacket()

# 清理
portHandler.closePort()
```

## 常见串口名称

- **Windows**：`COM1`、`COM3`、`COM4` 等
- **Linux**：`/dev/ttyUSB0`、`/dev/ttyACM0`
- **macOS**：`/dev/cu.usbserial-*`、`/dev/tty.usbserial-*`


## 最佳实践

1. **先 ping 再控制**：发送控制指令前先确认舵机在线
2. **检查返回值**：同时检查通信错误与硬件错误
3. **优先使用同步操作**：多舵机协同时更稳定、更一致
4. **及时关闭串口**：释放串口设备，避免被占用
5. **合理设置速度和加速度**：减少突兀动作与机械冲击
6. **监控扭矩限制**：防止过热和过载

## 故障排查

| Problem | Solution |
|---------|----------|
| "Port is in use" | 串口被其他程序占用，关闭其他串口应用 |
| "Failed to open port" | 检查串口名、USB 连接和系统权限 |
| "Rx timeout" | 舵机无响应，检查供电、接线和舵机 ID |
| "Overload error" | 降低负载或扭矩限制 |
| "Overheat error" | 暂停使用让舵机降温，降低工作强度 |
| Servo doesn't move | 检查扭矩限制（地址 22）是否被设为 0 |
