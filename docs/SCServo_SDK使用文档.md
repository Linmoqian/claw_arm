# SCServo SDK Python 使用文档

## 目录

1. [概述](#概述)
2. [安装](#安装)
3. [模块结构](#模块结构)
4. [核心类](#核心类)
5. [API 参考](#api-参考)
6. [示例代码](#示例代码)
7. [寄存器地址](#寄存器地址)
8. [错误处理](#错误处理)
9. [高级用法](#高级用法)

---

## 概述

**SCServo SDK** 是 Feetech 公司提供的 Python 串行舵机控制库，用于控制 STS、SMS、SCS 系列总线舵机。

### 特点

- 支持多舵机总线通信
- 同步读写功能
- 位置、速度、扭矩控制
- Python 2.7/3.x 兼容

### 适用电机

- STS3215 (SO100 使用)
- 其他 STS/SMS/SCS 系列舵机

---

## 安装

### 随 LeRobot 安装（当前方式）

```bash
pip install lerobot[feetech]
```

SDK 位置：
```
C:\Users\30470\Anaconda3\envs\lerobot\lib\site-packages\scservo_sdk\
```

### 独立安装

从官网下载：http://www.scservo.com/

或从 GitHub 克隆：
```bash
git clone https://gitee.com/kennethyu/FTServo_Python.git
```

---

## 模块结构

```
SDK/
├── __init__.py                    # 主入口
├── port_handler.py               # 串口通信处理
├── packet_handler.py             # 数据包处理（简化的接口）
├── protocol_packet_handler.py    # 底层协议处理
├── group_sync_read.py            # 批量同步读取
├── group_sync_write.py           # 批量同步写入
└── scservo_def.py                # 常量和宏定义
```

---

## 核心类

### 1. PortHandler

**功能**: 串口通信管理

#### 初始化

```python
from scservo_sdk import PortHandler

port_handler = PortHandler("COM7")  # 或 "/dev/ttyUSB0"
```

#### 主要方法

| 方法 | 说明 |
|------|------|
| `openPort()` | 打开串口 |
| `closePort()` | 关闭串口 |
| `setBaudRate(baudrate)` | 设置波特率 |
| `getBaudRate()` | 获取当前波特率 |
| `clearPort()` | 清空缓冲区 |
| `writePort(packet)` | 写入数据 |
| `readPort(length)` | 读取数据 |

#### 支持的波特率

```
4800, 9600, 14400, 19200, 38400, 57600,
115200, 128000, 250000, 500000, 1000000
```

#### 示例

```python
from scservo_sdk import PortHandler

# 创建端口处理器
port = PortHandler("COM7")

# 打开端口（自动使用默认波特率 1000000）
if port.openPort():
    print("端口打开成功")
    print(f"波特率: {port.getBaudRate()}")
else:
    print("端口打开失败")

# 关闭端口
port.closePort()
```

### 2. PacketHandler

**功能**: 数据包发送和接收（推荐使用）

#### 初始化

```python
from scservo_sdk import PortHandler, PacketHandler

port_handler = PortHandler("COM7")
packet_handler = PacketHandler(0.0)  # 协议版本
```

#### 主要方法

**读取方法:**

```python
# 读取 1 字节
data, result, error = packet_handler.read1ByteTxRx(
    port_handler, motor_id, address
)

# 读取 2 字节（最常用）
data, result, error = packet_handler.read2ByteTxRx(
    port_handler, motor_id, address
)

# 读取 4 字节
data, result, error = packet_handler.read4ByteTxRx(
    port_handler, motor_id, address
)
```

**写入方法:**

```python
# 写入 1 字节
result, error = packet_handler.write1ByteTxRx(
    port_handler, motor_id, address, data
)

# 写入 2 字节（最常用）
result, error = packet_handler.write2ByteTxRx(
    port_handler, motor_id, address, data
)

# 写入 4 字节
result, error = packet_handler.write4ByteTxRx(
    port_handler, motor_id, address, data
)
```

**其他方法:**

```python
# Ping 电机
model_number, result, error = packet_handler.ping(port_handler, motor_id)

# 触发动作
result = packet_handler.action(port_handler, motor_id)
```

### 3. GroupSyncRead

**功能**: 批量同步读取多个电机的数据

#### 初始化

```python
from scservo_sdk import PortHandler, PacketHandler, GroupSyncRead

port_handler = PortHandler("COM7")
packet_handler = PacketHandler(0.0)

# 创建批量读取器
# 参数：端口、数据包处理器、起始地址、数据长度
group_read = GroupSyncRead(
    port_handler,
    packet_handler,
    start_address=56,  # PRESENT_POSITION
    data_length=2       # 2 字节
)
```

#### 主要方法

| 方法 | 说明 |
|------|------|
| `addParam(motor_id)` | 添加电机到读取列表 |
| `removeParam(motor_id)` | 从读取列表移除电机 |
| `clearParam()` | 清空读取列表 |
| `txRxPacket()` | 发送读取请求并接收数据 |

#### 示例

```python
# 添加电机
for motor_id in range(1, 7):
    group_read.addParam(motor_id)

# 批量读取
result = group_read.txRxPacket()

if result == 0:  # COMM_SUCCESS
    for motor_id in range(1, 7):
        position = group_read.getData(motor_id, 56, 2)
        print(f"电机 {motor_id}: {position}")
```

### 4. GroupSyncWrite

**功能**: 批量同步写入数据到多个电机

#### 初始化

```python
from scservo_sdk import PortHandler, PacketHandler, GroupSyncWrite

port_handler = PortHandler("COM7")
packet_handler = PacketHandler(0.0)

# 创建批量写入器
group_write = GroupSyncWrite(
    port_handler,
    packet_handler,
    start_address=42,  # GOAL_POSITION
    data_length=2       # 2 字节
)
```

#### 主要方法

| 方法 | 说明 |
|------|------|
| `addParam(motor_id, data)` | 添加电机和数据 |
| `changeParam(motor_id, data)` | 修改电机数据 |
| `removeParam(motor_id)` | 移除电机 |
| `clearParam()` | 清空列表 |
| `txPacket()` | 发送写入数据 |

#### 示例

```python
# 添加电机和目标位置
positions = {
    1: 2048,
    2: 2048,
    3: 2048,
    4: 2048,
    5: 2048,
    6: 2048
}

for motor_id, position in positions.items():
    # 将 16 位位置值分解为 2 字节
    data = [position & 0xFF, (position >> 8) & 0xFF]
    group_write.addParam(motor_id, data)

# 批量写入
result = group_write.txPacket()
```

---

## API 参考

### 通信结果码

```python
# 从 scservo_def.py

COMM_SUCCESS = 0         # 通信成功
COMM_PORT_BUSY = -1       # 端口忙
COMM_TX_FAIL = -2         # 发送失败
COMM_RX_FAIL = -3         # 接收失败
COMM_TX_ERROR = -4        # 数据包错误
COMM_RX_WAITING = -5      # 等待接收
COMM_RX_TIMEOUT = -6      # 接收超时
COMM_RX_CORRUPT = -7      # 数据包损坏
COMM_NOT_AVAILABLE = -9   # 功能不可用
```

### 指令码

```python
INST_PING = 1             # Ping 指令
INST_READ = 2             # 读指令
INST_WRITE = 3            # 写指令
INST_REG_WRITE = 4        # 寄存器写
INST_ACTION = 5           # 动作指令
INST_SYNC_WRITE = 131      # 同步写
INST_SYNC_READ = 130       # 同步读
```

### 辅助函数

```python
# 字节操作
SCS_LOBYTE(w)             # 获取低字节
SCS_HIBYTE(w)             # 获取高字节
SCS_LOWORD(l)             # 获取低 16 位
SCS_HIWORD(l)             # 获取高 16 位
SCS_MAKEWORD(a, b)        # 组合 16 位
SCS_MAKEDWORD(a, b)       # 组合 32 位
```

---

## 示例代码

### 示例 1：基础控制

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scservo_sdk import PortHandler, PacketHandler
import time

# 配置
PORT_NAME = "COM7"
BAUDRATE = 1000000
MOTOR_ID = 1

# 初始化
port_handler = PortHandler(PORT_NAME)
packet_handler = PacketHandler(0.0)

try:
    # 打开端口
    if not port_handler.openPort():
        print("无法打开端口")
        exit(1)

    print(f"端口已打开: {PORT_NAME}")
    print(f"波特率: {port_handler.getBaudRate()}")

    # 启用扭矩
    result, error = packet_handler.write1ByteTxRx(
        port_handler,
        MOTOR_ID,
        40,  # TORQUE_ENABLE
        1    # 1 = 启用, 0 = 禁用
    )

    if result == 0:
        print("扭矩已启用")
    else:
        print(f"启用扭矩失败: {result}")
        exit(1)

    # 移动到中间位置
    target_position = 2048  # 0-4095
    result, error = packet_handler.write2ByteTxRx(
        port_handler,
        MOTOR_ID,
        42,  # GOAL_POSITION
        target_position
    )

    print(f"移动到位置: {target_position}")
    time.sleep(2)

    # 读取当前位置
    position, result, error = packet_handler.read2ByteTxRx(
        port_handler,
        MOTOR_ID,
        56  # PRESENT_POSITION
    )

    if result == 0:
        print(f"当前位置: {position}")
    else:
        print(f"读取失败: {result}")

    # 禁用扭矩
    packet_handler.write1ByteTxRx(
        port_handler,
        MOTOR_ID,
        40,
        0
    )
    print("扭矩已禁用")

finally:
    port_handler.closePort()
    print("端口已关闭")
```

### 示例 2：多电机控制

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scservo_sdk import PortHandler, PacketHandler
import time

PORT_NAME = "COM7"

# 电机配置
motors = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper"
}

port_handler = PortHandler(PORT_NAME)
packet_handler = PacketHandler(0.0)

try:
    port_handler.openPort()
    print("已连接到 SO100")

    # 启用所有电机
    print("\n启用扭矩...")
    for motor_id in motors.keys():
        packet_handler.write1ByteTxRx(port_handler, motor_id, 40, 1)
    print("所有电机已启用")

    # 移动到零点
    print("\n移动到零点...")
    for motor_id in motors.keys():
        packet_handler.write2ByteTxRx(port_handler, motor_id, 42, 2048)
    time.sleep(2)

    # 读取所有位置
    print("\n当前位置:")
    for motor_id, name in motors.items():
        pos, result, _ = packet_handler.read2ByteTxRx(
            port_handler, motor_id, 56
        )
        if result == 0:
            print(f"  {name}: {pos}")

    # 测试动作
    print("\n执行测试动作...")

    test_action = {
        1: 2500,
        2: 2500,
        3: 1500,
        4: 1500,
        5: 2048,
        6: 3000
    }

    for motor_id, position in test_action.items():
        packet_handler.write2ByteTxRx(port_handler, motor_id, 42, position)

    time.sleep(2)
    print("动作完成")

finally:
    # 禁用所有电机
    for motor_id in motors.keys():
        packet_handler.write1ByteTxRx(port_handler, motor_id, 40, 0)

    port_handler.closePort()
    print("\n已断开连接")
```

### 示例 3：批量同步读写

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scservo_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncRead,
    GroupSyncWrite
)
import time

PORT_NAME = "COM7"

port_handler = PortHandler(PORT_NAME)
packet_handler = PacketHandler(0.0)

try:
    port_handler.openPort()

    # ========== 批量读取 ==========
    print("批量读取所有电机位置...")

    group_read = GroupSyncRead(
        port_handler,
        packet_handler,
        start_address=56,  # PRESENT_POSITION
        data_length=2
    )

    # 添加电机
    for motor_id in range(1, 7):
        group_read.addParam(motor_id)

    # 执行批量读取
    result = group_read.txRxPacket()

    if result == 0:
        print("读取成功:")
        for motor_id in range(1, 7):
            pos = group_read.getData(motor_id, 56, 2)
            print(f"  电机 {motor_id}: {pos}")

    # ========== 批量写入 ==========
    print("\n批量写入目标位置...")

    group_write = GroupSyncWrite(
        port_handler,
        packet_handler,
        start_address=42,  # GOAL_POSITION
        data_length=2
    )

    # 添加电机和目标位置
    target_positions = [2048] * 6  # 所有电机到中间
    motor_ids = list(range(1, 7))

    for motor_id, position in zip(motor_ids, target_positions):
        # 分解为 2 字节
        data = [position & 0xFF, (position >> 8) & 0xFF]
        group_write.addParam(motor_id, data)

    # 执行批量写入
    result = group_write.txPacket()

    if result == 0:
        print("批量写入成功")
        time.sleep(2)
    else:
        print(f"批量写入失败: {result}")

finally:
    port_handler.closePort()
```

### 示例 4：错误处理

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scservo_sdk import PortHandler, PacketHandler
from scservo_sdk.scservo_def import COMM_SUCCESS

PORT_NAME = "COM7"
MOTOR_ID = 1

port_handler = PortHandler(PORT_NAME)
packet_handler = PacketHandler(0.0)

def check_result(result, error, operation):
    """检查通信结果"""
    if result == COMM_SUCCESS:
        print(f"[OK] {operation} 成功")
        return True
    else:
        error_msg = packet_handler.getTxRxResult(result)
        print(f"[ERROR] {operation} 失败:")
        print(f"  结果码: {result}")
        print(f"  信息: {error_msg}")

        # 检查错误位
        if error:
            error_detail = packet_handler.getRxPacketError(error)
            print(f"  错误详情: {error_detail}")

        return False

try:
    port_handler.openPort()

    # 启用扭矩
    result, error = packet_handler.write1ByteTxRx(
        port_handler, MOTOR_ID, 40, 1
    )
    check_result(result, error, "启用扭矩")

    # 写入位置
    result, error = packet_handler.write2ByteTxRx(
        port_handler, MOTOR_ID, 42, 2048
    )
    check_result(result, error, "写入位置")

    # 读取位置
    position, result, error = packet_handler.read2ByteTxRx(
        port_handler, MOTOR_ID, 56
    )
    if check_result(result, error, "读取位置"):
        print(f"  位置: {position}")

    # Ping 测试
    model_number, result, error = packet_handler.ping(port_handler, MOTOR_ID)
    if check_result(result, error, "Ping"):
        print(f"  型号: {model_number}")

finally:
    port_handler.closePort()
```

### 示例 5：完整控制类

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scservo_sdk import PortHandler, PacketHandler
from scservo_sdk.scservo_def import COMM_SUCCESS
import time

class SO100Controller:
    """SO100 机械臂控制器"""

    # STS3215 寄存器地址
    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56
    MODEL_NUMBER = 3

    MAX_POSITION = 4095

    def __init__(self, port="COM7", baudrate=1000000):
        self.port_name = port
        self.baudrate = baudrate

        # 初始化
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(0.0)

        # 电机名称
        self.motors = {
            1: "shoulder_pan",
            2: "shoulder_lift",
            3: "elbow_flex",
            4: "wrist_flex",
            5: "wrist_roll",
            6: "gripper"
        }

    def connect(self):
        """连接到机械臂"""
        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port_name}")

        print(f"[OK] 已连接到 {self.port_name}")
        return True

    def disconnect(self):
        """断开连接"""
        # 禁用所有电机
        for motor_id in range(1, 7):
            self.disable_torque(motor_id)

        self.port_handler.closePort()
        print(f"[OK] 已断开连接")

    def enable_torque(self, motor_id):
        """启用电机扭矩"""
        result, _ = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id,
            self.TORQUE_ENABLE, 1
        )
        return result == COMM_SUCCESS

    def disable_torque(self, motor_id):
        """禁用电机扭矩"""
        result, _ = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id,
            self.TORQUE_ENABLE, 0
        )
        return result == COMM_SUCCESS

    def set_position(self, motor_id, position):
        """设置目标位置 (0-4095)"""
        position = max(0, min(self.MAX_POSITION, position))
        result, _ = self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id,
            self.GOAL_POSITION, position
        )
        return result == COMM_SUCCESS

    def get_position(self, motor_id):
        """读取当前位置"""
        position, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id,
            self.PRESENT_POSITION
        )

        if result == COMM_SUCCESS:
            return position
        return 0

    def ping(self, motor_id):
        """检测电机"""
        model_num, result, _ = self.packet_handler.ping(
            self.port_handler, motor_id
        )
        return result == COMM_SUCCESS

    def scan_motors(self):
        """扫描所有电机"""
        found = []
        for motor_id in range(1, 7):
            if self.ping(motor_id):
                found.append(motor_id)
        return found


# 使用示例
if __name__ == "__main__":
    robot = SO100Controller("COM7")

    try:
        robot.connect()

        # 扫描电机
        found = robot.scan_motors()
        print(f"\n检测到 {len(found)} 个电机")

        # 启用所有电机
        print("\n启用扭矩...")
        for motor_id in found:
            robot.enable_torque(motor_id)

        # 移动测试
        print("\n移动到零点...")
        for motor_id in found:
            robot.set_position(motor_id, 2048)
        time.sleep(2)

        # 读取位置
        print("\n当前位置:")
        for motor_id in found:
            pos = robot.get_position(motor_id)
            print(f"  电机 {motor_id}: {pos}")

    finally:
        robot.disconnect()
```

---

## 寄存器地址

### STS3215 常用寄存器

| 地址 | 名称 | 访问 | 说明 | 默认值 |
|------|------|------|------|--------|
| 3 | ID | R/W | 电机 ID | - |
| 40 | TORQUE_ENABLE | R/W | 扭矩使能 | 0 |
| 42 | GOAL_POSITION | R/W | 目标位置 | - |
| 46 | MOVING_SPEED | R/W | 移动速度 | 0 |
| 56 | PRESENT_POSITION | R | 当前位置 | - |
| 38 | TORQUE_LIMIT | R/W | 扭矩限制 | - |
| 44 | ACCELERATION | R/W | 加速度 | 0 |

### EEPROM 区域（需要重新上电生效）

| 地址 | 名称 | 说明 |
|------|------|------|
| 3 | ID | 电机 ID |
| 6 | BAUD_RATE | 波特率 |

---

## 错误处理

### 结果码

```python
COMM_SUCCESS = 0         # 正常
COMM_PORT_BUSY = -1       # 端口被占用
COMM_TX_FAIL = -2         # 发送失败
COMM_RX_FAIL = -3         # 接收失败
COMM_TX_ERROR = -4        # 发送错误
COMM_RX_TIMEOUT = -6      # 接收超时
COMM_RX_CORRUPT = -7      # 数据包损坏
```

### 错误位

```python
ERRBIT_VOLTAGE = 1        # 电压错误
ERRBIT_ANGLE = 2          # 角度传感器错误
ERRBIT_OVERHEAT = 4       # 过热
ERRBIT_OVERELE = 8        # 过载（过电流）
ERRBIT_OVERLOAD = 32      # 过载
```

### 错误处理示例

```python
result, error = packet_handler.write2ByteTxRx(
    port_handler, motor_id, 42, 2048
)

if result != COMM_SUCCESS:
    # 获取错误信息
    error_msg = packet_handler.getTxRxResult(result)
    print(f"通信失败: {error_msg}")

    # 检查硬件错误
    if error:
        error_detail = packet_handler.getRxPacketError(error)
        print(f"硬件错误: {error_detail}")

        # 根据错误处理
        if ERRBIT_VOLTAGE in error_detail:
            print("错误: 电压异常")
        elif ERRBIT_OVERHEAT in error_detail:
            print("错误: 电机过热")
        elif ERRBIT_OVERLOAD in error_detail:
            print("错误: 电机过载")
```

---

## 高级用法

### 1. 异步控制

```python
import threading

class AsyncController:
    def __init__(self, port):
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(0.0)
        self.is_running = False

    def start(self):
        """启动控制线程"""
        self.port_handler.openPort()
        self.is_running = True

        # 创建线程
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.start()

    def stop(self):
        """停止控制线程"""
        self.is_running = False
        self.thread.join()
        self.port_handler.closePort()

    def _control_loop(self):
        """控制循环"""
        while self.is_running:
            # 持续读取位置
            positions = {}
            for motor_id in range(1, 7):
                pos, result, _ = self.packet_handler.read2ByteTxRx(
                    self.port_handler, motor_id, 56
                )
                if result == 0:
                    positions[motor_id] = pos

            # 处理位置数据...
            print(f"Positions: {positions}")
            time.sleep(0.1)  # 10 Hz
```

### 2. 轨迹规划

```python
def execute_trajectory(robot, trajectory):
    """
    执行轨迹

    Args:
        robot: SO100Controller 实例
        trajectory: 轨迹列表 [[p1,p2,p3,p4,p5,p6], ...]
    """
    for step, positions in enumerate(trajectory):
        print(f"步骤 {step + 1}/{len(trajectory)}")

        # 发送所有电机目标位置
        for motor_id, position in enumerate(positions, start=1):
            robot.set_position(motor_id, position)

        # 等待运动完成
        time.sleep(0.5)

        # 验证到达
        reached = True
        for motor_id, target_pos in enumerate(positions, start=1):
            current_pos = robot.get_position(motor_id)
            if abs(current_pos - target_pos) > 10:  # 允许 10 步误差
                reached = False

        if not reached:
            print(f"  警告: 未完全到达目标")

    print("轨迹执行完成")
```

### 3. 位置限制保护

```python
class SafeController(SO100Controller):
    """带安全保护的控制器"""

    def __init__(self, port="COM7", min_positions=None, max_positions=None):
        super().__init__(port)

        # 设置位置限制
        self.min_pos = min_positions or {1: 500, 2: 500, 3: 500, 4: 500, 5: 0, 6: 0}
        self.max_pos = max_positions or {1: 3500, 2: 3500, 3: 3500, 4: 3500, 5: 4095, 6: 4095}

    def set_position_safe(self, motor_id, position):
        """安全设置位置（带限制）"""
        # 限制范围
        min_pos = self.min_pos.get(motor_id, 0)
        max_pos = self.max_pos.get(motor_id, 4095)

        # 裁剪到安全范围
        position = max(min_pos, min(position, max_pos))

        # 检查当前位置
        current_pos = self.get_position(motor_id)

        # 限制每次移动幅度
        max_delta = 500  # 每次最多移动 500
        delta = position - current_pos
        if abs(delta) > max_delta:
            position = current_pos + (max_delta if delta > 0 else -max_delta)

        return super().set_position(motor_id, position)
```

---

## 附录

### 常见问题

**Q: 如何修改电机 ID？**

A: 使用 Feetech 官方软件，或 SDK 的 write1ByteTxRx 写入地址 3。

**Q: 如何设置波特率？**

A: 写入 EEPROM 地址 6，然后重新上电。

**Q: 批量写入失败？**

A: 检查：1. 是否添加了所有电机，2. 数据长度是否正确，3. 电源是否足够。

**Q: 如何实现高速控制？**

A: 1. 使用批量写入，2. 减少读取频率，3. 优化控制周期。

---

**文档版本**: 1.0
**最后更新**: 2026-03-01
**SDK 路径**: `C:\Users\30470\Anaconda3\envs\lerobot\lib\site-packages\scservo_sdk\`

---

## 参考资源

- [Feetech 官网](http://www.scservo.com/)
- [STS3215 数据手册](http://www.scservo.com/)
- [LeRobot 项目](https://github.com/huggingface/lerobot)
