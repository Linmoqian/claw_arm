---
name: simple-arm
description: 简化机械臂控制。通过自然语言控制 SO100 机械臂，自动计算逆运动学并反馈完整状态。当用户说类似"把末端移到 x=0.2 y=0.1 z=0.3"、"移动到某个位置"、"抓取"、"回零位"时触发此技能。
---

# 简化版机械臂控制

通过自然语言控制 SO100 机械臂，自动处理逆运动学计算，返回完整的关节和末端状态。

## 工作流程

1. **解析自然语言** → 识别目标位置或动作
2. **计算逆运动学** → 确定如何到达目标
3. **执行移动** → 控制电机运动
4. **反馈状态** → 返回关节位置和末端位姿

## 核心代码模板

```python
import sys
import time
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT_ROOT))

from SDK import PortHandler, PacketHandler
from skills.urdf-kinematics.scripts.urdf_kinematics import SO100Kinematics, ServoConverter

# ============================================================
#  控制器（整合运动学 + SDK）
# ============================================================

class SimpleArm:
    """简化版机械臂控制器"""

    JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    MOTOR_IDS = [1, 2, 3, 4, 5, 6]
    ADDR_TORQUE = 40
    ADDR_GOAL = 42
    ADDR_PRESENT = 56

    def __init__(self, port="COM7", baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self._port = None
        self._pkt = None
        self._kin = SO100Kinematics()

    def connect(self) -> dict:
        """连接机械臂，返回状态"""
        try:
            self._port = PortHandler(self.port)
            self._pkt = PacketHandler(0.0)
            if not self._port.openPort() or not self._port.setBaudRate(self.baudrate):
                return {"success": False, "message": "连接失败"}
            motors = [i for i in range(1, 7) if self._pkt.ping(self._port, i)[1] == 0]
            return {"success": True, "motors": motors, "count": len(motors)}
        except Exception as e:
            return {"success": False, "message": str(e)}

    def disconnect(self):
        if self._port:
            for i in range(1, 7):
                self._pkt.write1ByteTxRx(self._port, i, self.ADDR_TORQUE, 0)
            self._port.closePort()

    def get_state(self) -> dict:
        """获取当前状态：关节位置 + 末端位姿"""
        positions = []
        for i in self.MOTOR_IDS:
            pos, res, _ = self._pkt.read2ByteTxRx(self._port, i, self.ADDR_PRESENT)
            positions.append(pos if res == 0 else 2047)

        # 转换为弧度并计算末端位姿
        q = ServoConverter.positions_to_joints(positions)
        pose = self._kin.get_end_effector_pose(q)

        return {
            "joints": {name: pos for name, pos in zip(self.JOINTS, positions)},
            "joints_rad": q.tolist(),
            "end_effector": {
                "position_m": pose["position"],
                "orientation_deg": np.rad2deg(pose["orientation"]["rpy"]).tolist()
            }
        }

    def move_to_pose(self, x: float, y: float, z: float,
                     roll: float = 0, pitch: float = 0, yaw: float = 0) -> dict:
        """移动到目标末端位姿（自动逆运动学）"""
        target = np.array([x, y, z, roll, pitch, yaw])

        # 检查工作空间
        if not self._kin.in_workspace(target[:3]):
            return {"success": False, "message": "目标位置超出工作空间"}

        # 逆运动学求解
        q = self._kin.inverse_kinematics(target, method="optimize")
        is_valid, msgs = self._kin.check_joint_limits(q)
        if not is_valid:
            return {"success": False, "message": "无有效解", "details": msgs}

        # 转换为舵机位置并执行
        positions = ServoConverter.joints_to_positions(q)
        return self._execute_positions(positions)

    def move_to_joints(self, positions: list) -> dict:
        """直接移动到关节位置"""
        positions = [max(0, min(4095, int(p))) for p in positions]
        return self._execute_positions(positions)

    def _execute_positions(self, positions: list) -> dict:
        """执行位置移动（平滑插值）"""
        # 读取当前位置
        current = []
        for i in self.MOTOR_IDS:
            pos, res, _ = self._pkt.read2ByteTxRx(self._port, i, self.ADDR_PRESENT)
            current.append(pos if res == 0 else 2047)

        # 启用扭矩
        for i in self.MOTOR_IDS:
            self._pkt.write1ByteTxRx(self._port, i, self.ADDR_TORQUE, 1)

        # 平滑插值
        steps = 30
        for s in range(1, steps + 1):
            ratio = s / steps
            for i, (cur, tgt) in enumerate(zip(current, positions)):
                pos = int(cur + (tgt - cur) * ratio)
                self._pkt.write2ByteTxRx(self._port, i + 1, self.ADDR_GOAL, pos)
            time.sleep(0.02)

        return {"success": True, "final_positions": positions}

    def home(self) -> dict:
        """回到零位"""
        return self.move_to_joints([2047] * 6)

    def open_gripper(self) -> dict:
        return self.move_to_joints([2047, 2047, 2047, 2047, 2047, 3000])

    def close_gripper(self) -> dict:
        return self.move_to_joints([2047, 2047, 2047, 2047, 2047, 1000])


# ============================================================
#  自然语言解析
# ============================================================

def parse_natural_language(text: str) -> dict:
    """解析自然语言指令"""
    import re
    result = {"action": None, "target": None}

    # 动作关键词
    if any(k in text for k in ["回零", "零位", "初始", "复位"]):
        result["action"] = "home"
    elif any(k in text for k in ["张开", "打开", "松开"]):
        result["action"] = "open_gripper"
    elif any(k in text for k in ["闭合", "关闭", "抓紧"]):
        result["action"] = "close_gripper"
    elif any(k in text for k in ["移动", "移到", "位置", "坐标"]):
        result["action"] = "move_to_pose"
        # 提取坐标数字
        nums = re.findall(r"[-+]?\d*\.?\d+", text)
        if len(nums) >= 3:
            result["target"] = [float(n) for n in nums[:6]]
            while len(result["target"]) < 6:
                result["target"].append(0)

    return result


# ============================================================
#  执行入口
# ============================================================

def execute(text: str, port: str = "COM7") -> dict:
    """
    执行自然语言指令，返回完整状态

    Args:
        text: 自然语言指令
        port: 串口

    Returns:
        {
            "success": bool,
            "message": str,
            "state": {
                "joints": {关节名: 位置},
                "end_effector": {"position_m": [x,y,z], "orientation_deg": [r,p,y]}
            }
        }
    """
    arm = SimpleArm(port=port)
    conn = arm.connect()

    if not conn["success"]:
        return {"success": False, "message": conn.get("message", "连接失败"), "state": None}

    try:
        intent = parse_natural_language(text)

        if intent["action"] == "home":
            result = arm.home()
            msg = "已回到零位"
        elif intent["action"] == "open_gripper":
            result = arm.open_gripper()
            msg = "夹爪已张开"
        elif intent["action"] == "close_gripper":
            result = arm.close_gripper()
            msg = "夹爪已闭合"
        elif intent["action"] == "move_to_pose" and intent["target"]:
            x, y, z, r, p, yw = intent["target"]
            result = arm.move_to_pose(x, y, z, r, p, yw)
            msg = f"移动到 ({x}, {y}, {z})" if result["success"] else result["message"]
        else:
            result = {"success": False}
            msg = "未理解指令，试试: '移动到 x=0.2 y=0.1 z=0.3' 或 '回零位'"

        state = arm.get_state()
        return {"success": result["success"], "message": msg, "state": state}

    finally:
        arm.disconnect()
```

## 使用示例

```python
# 示例 1: 移动到指定位置
result = execute("把末端移动到 x=0.2 y=0.1 z=0.3")
# 返回:
# {
#   "success": True,
#   "message": "移动到 (0.2, 0.1, 0.3)",
#   "state": {
#     "joints": {"shoulder_pan": 2047, ...},
#     "end_effector": {"position_m": [0.2, 0.1, 0.3], "orientation_deg": [...]}
#   }
# }

# 示例 2: 回零位
result = execute("回零位")

# 示例 3: 夹爪操作
result = execute("张开夹爪")
result = execute("闭合夹爪")
```

## 关节映射

| 关节名 | 电机 ID | 范围 | 说明 |
|--------|---------|------|------|
| shoulder_pan | 1 | 0-4095 | 底座旋转 |
| shoulder_lift | 2 | 0-4095 | 大臂升降 |
| elbow_flex | 3 | 0-4095 | 小臂弯曲 |
| wrist_flex | 4 | 0-4095 | 手腕俯仰 |
| wrist_roll | 5 | 0-4095 | 手腕旋转 |
| gripper | 6 | 0-4095 | 夹爪 |

## 工作空间

- 最大半径: ~0.47 m
- 位置单位: 米 (m)
- 角度单位: 度 (deg)
