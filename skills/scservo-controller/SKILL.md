name: scservo-controller
description: Control SC Servo motors for robotic arm/claw applications. Use this skill whenever the user needs to control servos, move robotic arms, control claw/gripper mechanisms, work with SCServo SDK, or manipulate serial-connected servo motors. This includes connecting to servo hardware, reading/writing servo positions, velocity control, multi-servo synchronization, and servo status monitoring.

# SC Servo Controller

This skill helps control SC Servo motors using the SCServo SDK (located at `D:\project\claw_arm\SDK`). The SDK provides serial communication with servo motors for robotic arm and claw control.

## SDK Components

The SDK consists of:

## Basic Workflow

1. **Import the SDK**
   ```python
   import sys
   sys.path.append('D:\\project\\claw_arm\\SDK')
   from scservo_def import *
   from port_handler import PortHandler
   from packet_handler import PacketHandler
   from group_sync_read import GroupSyncRead
   from group_sync_write import GroupSyncWrite
   ```

2. **Connect to the Servo**
   ```python
   # Open port (typically COM3, COM4, /dev/ttyUSB0, etc.)
   portHandler = PortHandler("COM3")  # Windows
   # portHandler = PortHandler("/dev/ttyUSB0")  # Linux
   if not portHandler.openPort():
       print("Failed to open port")
       sys.exit()

   # Set baudrate (default is 1000000)
   portHandler.setBaudRate(1000000)

   # Create packet handler (end: 0 for little endian)
   packetHandler = PacketHandler(0)
   ```

3. **Ping to Verify Connection**
   ```python
   scs_id = 1  # Servo ID (1-252)
   model_number, result, error = packetHandler.ping(portHandler, scs_id)
   if result != COMM_SUCCESS:
       print(f"Ping failed: {packetHandler.getTxRxResult(result)}")
   else:
       print(f"Connected! Model: {model_number}")
   ```

## Common Control Table Addresses

The control table uses memory addresses to read/write servo parameters. Common addresses:

| Address | Name | Size | Description |
|---------|------|------|-------------|
| 3 | Model Number | 2 | Model information |
| 11 | Position | 2 | Current position (0-1000) |
| 12 | Speed | 2 | Moving speed |
| 20 | Goal Position | 2 | Target position |
| 21 | Moving Speed | 2 | Target moving speed |
| 22 | Torque Limit | 2 | Maximum torque |
| 23 | Acceleration | 2 | Acceleration setting |
| 24 | LED | 1 | LED control |
| 25 | LED Control | 1 | LED error display |

**Note**: Addresses may vary by servo model. Always verify with your servo's datasheet.

## Core Operations

### Read Position
```python
# Read current position (address 11, 2 bytes)
position, result, error = packetHandler.read2ByteTxRx(portHandler, scs_id, 11)
if result == COMM_SUCCESS:
    print(f"Position: {position}")
```

### Write Goal Position
```python
# Move to position 500 (address 20, 2 bytes)
goal_pos = 500
result, error = packetHandler.write2ByteTxRx(portHandler, scs_id, 20, goal_pos)
if result != COMM_SUCCESS:
    print(f"Write failed: {packetHandler.getTxRxResult(result)}")
```

### Set Moving Speed
```python
# Set speed (address 21, 2 bytes)
speed = 500  # 0-1000 range
result, error = packetHandler.write2ByteTxRx(portHandler, scs_id, 21, speed)
```

### Set Torque Limit
```python
# Set torque limit (address 22, 2 bytes)
torque = 800  # 0-1000 range
result, error = packetHandler.write2ByteTxRx(portHandler, scs_id, 22, torque)
```

### Set Acceleration
```python
# Set acceleration (address 23, 2 bytes)
accel = 50  # 0-255 range
result, error = packetHandler.write1ByteTxRx(portHandler, scs_id, 23, accel)
```

### Control LED
```python
# Turn on LED (address 24, 1 byte)
result, error = packetHandler.write1ByteTxRx(portHandler, scs_id, 24, 1)
```

## Multi-Servo Synchronous Control

### Synchronous Write (Move Multiple Servos)
```python
# Create sync write handler for goal position (address 20, 2 bytes)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 20, 2)

# Add parameters for each servo
groupSyncWrite.addParam(1, [SCS_LOBYTE(500), SCS_HIBYTE(500)])  # Servo ID 1 to pos 500
groupSyncWrite.addParam(2, [SCS_LOBYTE(700), SCS_HIBYTE(700)])  # Servo ID 2 to pos 700
groupSyncWrite.addParam(3, [SCS_LOBYTE(300), SCS_HIBYTE(300)])  # Servo ID 3 to pos 300

# Send to all servos at once
result = groupSyncWrite.txPacket()
if result != COMM_SUCCESS:
    print(f"Sync write failed: {packetHandler.getTxRxResult(result)}")

# Clear for next use
groupSyncWrite.clearParam()
```

### Synchronous Read (Read Multiple Servos)
```python
# Create sync read handler for position (address 11, 2 bytes)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, 11, 2)

# Add servo IDs to read
groupSyncRead.addParam(1)
groupSyncRead.addParam(2)
groupSyncRead.addParam(3)

# Send read request
result = groupSyncRead.txRxPacket()
if result == COMM_SUCCESS:
    # Read position for each servo
    pos1 = groupSyncRead.getData(1, 11, 2)
    pos2 = groupSyncRead.getData(2, 11, 2)
    pos3 = groupSyncRead.getData(3, 11, 2)
    print(f"Positions: {pos1}, {pos2}, {pos3}")
```

## Error Handling

### Check Communication Result
```python
if result != COMM_SUCCESS:
    error_msg = packetHandler.getTxRxResult(result)
    print(f"Communication error: {error_msg}")
    # COMM_SUCCESS = 0, COMM_PORT_BUSY = -1, COMM_TX_FAIL = -2,
    # COMM_RX_FAIL = -3, COMM_TX_ERROR = -4, COMM_RX_TIMEOUT = -6
```

### Check Hardware Error
```python
if error != 0:
    error_msg = packetHandler.getRxPacketError(error)
    print(f"Hardware error: {error_msg}")
    # ERRBIT_VOLTAGE = 1, ERRBIT_ANGLE = 2, ERRBIT_OVERHEAT = 4,
    # ERRBIT_OVERELE = 8, ERRBIT_OVERLOAD = 32
```

## Cleanup
```python
# Always close port when done
portHandler.closePort()
```

## Complete Example: Simple Servo Control
```python
import sys
sys.path.append('D:\\project\\claw_arm\\SDK')
from scservo_def import *
from port_handler import PortHandler
from packet_handler import PacketHandler

# Initialize
portHandler = PortHandler("COM3")
packetHandler = PacketHandler(0)

# Connect
if not portHandler.openPort():
    print("Failed to open port")
    sys.exit()

# Ping servo
scs_id = 1
model, result, error = packetHandler.ping(portHandler, scs_id)
if result != COMM_SUCCESS:
    print(f"Ping failed: {packetHandler.getTxRxResult(result)}")
    portHandler.closePort()
    sys.exit()

print(f"Connected to servo {scs_id}, model {model}")

# Read current position
pos, result, error = packetHandler.read2ByteTxRx(portHandler, scs_id, 11)
print(f"Current position: {pos}")

# Move to position 500
result, error = packetHandler.write2ByteTxRx(portHandler, scs_id, 20, 500)
if result == COMM_SUCCESS:
    print("Moving to position 500...")

# Wait and read new position
import time
time.sleep(1)
pos, result, error = packetHandler.read2ByteTxRx(portHandler, scs_id, 11)
print(f"New position: {pos}")

# Cleanup
portHandler.closePort()
```

## Complete Example: Multi-Servo Arm Control
```python
import sys
sys.path.append('D:\\project\\claw_arm\\SDK')
from scservo_def import *
from port_handler import PortHandler
from packet_handler import PacketHandler
from group_sync_write import GroupSyncWrite
import time

# Initialize
portHandler = PortHandler("COM3")
packetHandler = PacketHandler(0)

# Connect
if not portHandler.openPort():
    print("Failed to open port")
    sys.exit()

# Setup sync write for goal position
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 20, 2)

# Arm configuration (base, shoulder, elbow, wrist, claw)
arm_positions = {
    1: 500,  # Base servo
    2: 600,  # Shoulder
    3: 400,  # Elbow
    4: 700,  # Wrist
    5: 800   # Claw
}

# Set all positions
for scs_id, pos in arm_positions.items():
    groupSyncWrite.addParam(scs_id, [SCS_LOBYTE(pos), SCS_HIBYTE(pos)])

result = groupSyncWrite.txPacket()
if result == COMM_SUCCESS:
    print("Arm moved to target positions")
else:
    print(f"Failed: {packetHandler.getTxRxResult(result)}")

groupSyncWrite.clearParam()
time.sleep(2)

# Close claw
groupSyncWrite.addParam(5, [SCS_LOBYTE(200), SCS_HIBYTE(200)])
groupSyncWrite.txPacket()

# Cleanup
portHandler.closePort()
```

## Common Port Names


## Best Practices

1. **Always ping first** to verify servo connection before sending commands
2. **Check return values** for both communication and hardware errors
3. **Use synchronous operations** when coordinating multiple servos
4. **Close the port** when finished to release the serial device
5. **Set appropriate speed and acceleration** to avoid jerky movements
6. **Monitor torque limits** to prevent overheating

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Port is in use" | Another program is using the port. Close other applications. |
| "Failed to open port" | Check port name, verify USB connection, check permissions |
| "Rx timeout" | Servo not responding. Check wiring, power, servo ID |
| "Overload error" | Reduce torque limit or load |
| "Overheat error" | Let servo cool down, reduce usage |
| Servo doesn't move | Check torque limit (address 22) isn't set to 0 |
