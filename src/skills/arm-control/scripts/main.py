#!/usr/bin/env python3
"""
arm-control skill 主脚本
=========================
机械臂控制技能，支持 SDK 直连（飞特舵机 UART）和 ROS2 两种控制模式。
适配机型：Galaxea R1 Lite 双臂机器人

关键特性：
  - 逐关节识别、配置、限位
  - 单关节和批量关节控制
  - 舵机在线扫描（ping / scan）
  - SYNC_WRITE 同步写入
"""

import argparse
import json
import logging
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

import numpy as np

# ============================================================
#  日志配置
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("arm-control")


# ============================================================
#  逐关节配置 —— 每个关节/夹爪的独立参数
# ============================================================

@dataclass
class JointConfig:
    """单个关节的完整配置"""
    name: str           # 关节名称（与 Lerobot 数据集对齐）
    servo_id: int       # 飞特舵机 ID
    side: str           # "left" / "right"
    joint_type: str     # "revolute"（旋转关节）/ "gripper"（夹爪）
    index: int          # 在 14 维状态向量中的下标 (0-13)
    min_rad: float      # 最小角度 (弧度)，夹爪为 0.0
    max_rad: float      # 最大角度 (弧度)，夹爪为 1.0
    home_rad: float     # 初始位置 (弧度)
    max_speed: int      # 最大运动速度（舵机原始值 0-4095）
    description: str    # 关节中文描述


# ----- Galaxea R1 Lite 关节注册表（共 14 个） -----
JOINT_REGISTRY: List[JointConfig] = [
    # ========== 左臂 6 关节 ==========
    JointConfig(
        name="left_arm_joint_1_rad", servo_id=1, side="left",
        joint_type="revolute", index=0,
        min_rad=-3.14, max_rad=3.14, home_rad=0.0, max_speed=500,
        description="左臂基座旋转（肩部水平旋转）",
    ),
    JointConfig(
        name="left_arm_joint_2_rad", servo_id=2, side="left",
        joint_type="revolute", index=1,
        min_rad=-1.57, max_rad=1.57, home_rad=0.0, max_speed=500,
        description="左臂肩部俯仰（肩部上下摆动）",
    ),
    JointConfig(
        name="left_arm_joint_3_rad", servo_id=3, side="left",
        joint_type="revolute", index=2,
        min_rad=-1.57, max_rad=1.57, home_rad=0.0, max_speed=500,
        description="左臂肘部（前臂弯曲）",
    ),
    JointConfig(
        name="left_arm_joint_4_rad", servo_id=4, side="left",
        joint_type="revolute", index=3,
        min_rad=-1.57, max_rad=1.57, home_rad=0.0, max_speed=400,
        description="左臂腕部旋转（前臂旋转）",
    ),
    JointConfig(
        name="left_arm_joint_5_rad", servo_id=5, side="left",
        joint_type="revolute", index=4,
        min_rad=-1.57, max_rad=1.57, home_rad=0.0, max_speed=400,
        description="左臂腕部俯仰（手腕上下）",
    ),
    JointConfig(
        name="left_arm_joint_6_rad", servo_id=6, side="left",
        joint_type="revolute", index=5,
        min_rad=-3.14, max_rad=3.14, home_rad=0.0, max_speed=400,
        description="左臂末端旋转（手腕旋转）",
    ),
    # ========== 左臂夹爪 ==========
    JointConfig(
        name="left_gripper_open", servo_id=7, side="left",
        joint_type="gripper", index=6,
        min_rad=0.0, max_rad=1.0, home_rad=0.5, max_speed=600,
        description="左臂夹爪开合（0=关闭, 1=打开）",
    ),
    # ========== 右臂 6 关节 ==========
    JointConfig(
        name="right_arm_joint_1_rad", servo_id=8, side="right",
        joint_type="revolute", index=7,
        min_rad=-3.14, max_rad=3.14, home_rad=0.0, max_speed=500,
        description="右臂基座旋转（肩部水平旋转）",
    ),
    JointConfig(
        name="right_arm_joint_2_rad", servo_id=9, side="right",
        joint_type="revolute", index=8,
        min_rad=-1.57, max_rad=1.57, home_rad=0.0, max_speed=500,
        description="右臂肩部俯仰（肩部上下摆动）",
    ),
    JointConfig(
        name="right_arm_joint_3_rad", servo_id=10, side="right",
        joint_type="revolute", index=9,
        min_rad=-1.57, max_rad=1.57, home_rad=0.0, max_speed=500,
        description="右臂肘部（前臂弯曲）",
    ),
    JointConfig(
        name="right_arm_joint_4_rad", servo_id=11, side="right",
        joint_type="revolute", index=10,
        min_rad=-1.57, max_rad=1.57, home_rad=0.0, max_speed=400,
        description="右臂腕部旋转（前臂旋转）",
    ),
    JointConfig(
        name="right_arm_joint_5_rad", servo_id=12, side="right",
        joint_type="revolute", index=11,
        min_rad=-1.57, max_rad=1.57, home_rad=0.0, max_speed=400,
        description="右臂腕部俯仰（手腕上下）",
    ),
    JointConfig(
        name="right_arm_joint_6_rad", servo_id=13, side="right",
        joint_type="revolute", index=12,
        min_rad=-3.14, max_rad=3.14, home_rad=0.0, max_speed=400,
        description="右臂末端旋转（手腕旋转）",
    ),
    # ========== 右臂夹爪 ==========
    JointConfig(
        name="right_gripper_open", servo_id=14, side="right",
        joint_type="gripper", index=13,
        min_rad=0.0, max_rad=1.0, home_rad=0.5, max_speed=600,
        description="右臂夹爪开合（0=关闭, 1=打开）",
    ),
]

# ----- 快速查找索引 -----
JOINT_BY_NAME: Dict[str, JointConfig] = {j.name: j for j in JOINT_REGISTRY}
JOINT_BY_ID: Dict[int, JointConfig] = {j.servo_id: j for j in JOINT_REGISTRY}
JOINT_BY_INDEX: Dict[int, JointConfig] = {j.index: j for j in JOINT_REGISTRY}

# 便捷常量
JOINT_NAMES: List[str] = [j.name for j in JOINT_REGISTRY]
ALL_SERVO_IDS: List[int] = [j.servo_id for j in JOINT_REGISTRY]
LEFT_ARM_JOINT_IDS: List[int] = [j.servo_id for j in JOINT_REGISTRY
                                  if j.side == "left" and j.joint_type == "revolute"]
LEFT_GRIPPER_ID: int = next(j.servo_id for j in JOINT_REGISTRY
                             if j.side == "left" and j.joint_type == "gripper")
RIGHT_ARM_JOINT_IDS: List[int] = [j.servo_id for j in JOINT_REGISTRY
                                   if j.side == "right" and j.joint_type == "revolute"]
RIGHT_GRIPPER_ID: int = next(j.servo_id for j in JOINT_REGISTRY
                              if j.side == "right" and j.joint_type == "gripper")

HOME_POSITION: List[float] = [j.home_rad for j in JOINT_REGISTRY]


# ============================================================
#  数据结构
# ============================================================

class ArmSide(Enum):
    """手臂选择枚举"""
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


@dataclass
class JointState:
    """单个关节的运行时状态"""
    config: JointConfig
    position: float = 0.0      # 当前位置（弧度 / 开合度）
    online: bool = False       # 该舵机是否在线
    temperature: int = 0       # 温度 (°C)
    load: int = 0              # 当前负载

    def to_dict(self) -> dict:
        return {
            "name": self.config.name,
            "servo_id": self.config.servo_id,
            "description": self.config.description,
            "position": round(self.position, 4),
            "min": self.config.min_rad,
            "max": self.config.max_rad,
            "online": self.online,
            "temperature": self.temperature,
            "load": self.load,
        }


@dataclass
class ArmState:
    """机械臂完整状态（14 关节）"""
    joints: Dict[str, JointState] = field(default_factory=dict)
    timestamp: float = 0.0

    def __post_init__(self):
        # 如果未初始化，则用注册表填充
        if not self.joints:
            for jc in JOINT_REGISTRY:
                self.joints[jc.name] = JointState(config=jc, position=jc.home_rad)

    # ----- 便捷属性 -----

    @property
    def left_joints(self) -> List[float]:
        return [self.joints[j.name].position for j in JOINT_REGISTRY
                if j.side == "left" and j.joint_type == "revolute"]

    @property
    def left_gripper(self) -> float:
        return self.joints["left_gripper_open"].position

    @property
    def right_joints(self) -> List[float]:
        return [self.joints[j.name].position for j in JOINT_REGISTRY
                if j.side == "right" and j.joint_type == "revolute"]

    @property
    def right_gripper(self) -> float:
        return self.joints["right_gripper_open"].position

    def to_array(self) -> np.ndarray:
        """转换为 14 维数组（按 index 排序）"""
        arr = np.zeros(14, dtype=np.float32)
        for js in self.joints.values():
            arr[js.config.index] = js.position
        return arr

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "ArmState":
        """从 14 维数组构建"""
        state = cls(timestamp=time.time())
        for idx, val in enumerate(arr[:14]):
            jc = JOINT_BY_INDEX[idx]
            state.joints[jc.name].position = float(val)
        return state

    def get_joint(self, name: str) -> Optional[JointState]:
        """按名称获取某个关节状态"""
        return self.joints.get(name)

    def to_dict(self) -> dict:
        """转换为字典 —— 包含每个关节的完整信息"""
        return {
            "timestamp": self.timestamp,
            "joints": {name: js.to_dict() for name, js in self.joints.items()},
            # 兼容旧格式的扁平数组
            "flat_array": self.to_array().tolist(),
        }

    def to_simple_dict(self) -> dict:
        """简洁格式（兼容旧版本）"""
        return {
            "left_joints_rad": self.left_joints,
            "left_gripper_open": self.left_gripper,
            "right_joints_rad": self.right_joints,
            "right_gripper_open": self.right_gripper,
            "timestamp": self.timestamp,
        }


# ============================================================
#  关节限位验证
# ============================================================

def validate_joint_value(name: str, value: float) -> float:
    """
    校验并钳位关节值到合法范围

    返回钳位后的值（如果超限会打印警告）
    """
    jc = JOINT_BY_NAME.get(name)
    if jc is None:
        raise ValueError(f"未知关节名: '{name}'，可用: {JOINT_NAMES}")
    clamped = max(jc.min_rad, min(jc.max_rad, value))
    if clamped != value:
        logger.warning(
            f"关节 '{name}'(ID={jc.servo_id}) 值 {value:.4f} 超出范围 "
            f"[{jc.min_rad}, {jc.max_rad}]，已钳位到 {clamped:.4f}"
        )
    return clamped


def validate_positions(positions: List[float], arm: "ArmSide") -> List[float]:
    """批量校验关节值列表"""
    if arm == ArmSide.LEFT:
        configs = [j for j in JOINT_REGISTRY if j.side == "left" and j.joint_type == "revolute"]
    elif arm == ArmSide.RIGHT:
        configs = [j for j in JOINT_REGISTRY if j.side == "right" and j.joint_type == "revolute"]
    else:
        configs = [j for j in JOINT_REGISTRY if j.joint_type == "revolute"]
    result = []
    for i, jc in enumerate(configs):
        if i < len(positions):
            result.append(validate_joint_value(jc.name, positions[i]))
        else:
            result.append(jc.home_rad)
    return result


# ============================================================
#  控制器基类
# ============================================================

class ArmController(ABC):
    """机械臂控制器抽象基类"""

    @abstractmethod
    def connect(self) -> bool:
        """连接机械臂，返回是否成功"""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """断开连接"""
        ...

    @abstractmethod
    def get_state(self) -> ArmState:
        """读取当前机械臂状态（全部 14 关节）"""
        ...

    @abstractmethod
    def get_single_joint(self, name: str) -> Optional[JointState]:
        """
        读取单个关节的当前状态

        参数:
            name: 关节名称，如 "left_arm_joint_3_rad"
        """
        ...

    @abstractmethod
    def set_joint_positions(self, positions: List[float], arm: ArmSide = ArmSide.BOTH) -> bool:
        """
        设置关节目标位置（批量）

        参数:
            positions: 关节角度列表（弧度）
                - arm=LEFT: 长度 6
                - arm=RIGHT: 长度 6
                - arm=BOTH: 长度 12（左6 + 右6）
            arm: 控制哪只手臂
        """
        ...

    @abstractmethod
    def set_single_joint(self, name: str, value: float) -> bool:
        """
        设置单个关节的目标位置

        参数:
            name: 关节名称，如 "left_arm_joint_3_rad" 或 "left_gripper_open"
            value: 目标值（弧度 / 夹爪开合度）
        """
        ...

    @abstractmethod
    def set_gripper(self, value: float, arm: ArmSide = ArmSide.LEFT) -> bool:
        """
        设置夹爪开合度

        参数:
            value: 0.0（完全关闭）到 1.0（完全打开）
            arm: 左臂或右臂
        """
        ...

    @abstractmethod
    def go_home(self) -> bool:
        """回到初始位置"""
        ...

    @abstractmethod
    def execute_trajectory(self, trajectory: np.ndarray, fps: int = 30) -> bool:
        """
        执行动作轨迹

        参数:
            trajectory: 形状 (T, 14) 的数组，T 为时间步数
            fps: 执行帧率
        """
        ...

    @abstractmethod
    def scan_servos(self) -> Dict[int, bool]:
        """
        扫描所有预期的舵机，返回在线状态

        返回:
            {servo_id: True/False} 字典
        """
        ...

    @abstractmethod
    def emergency_stop(self) -> None:
        """紧急停止"""
        ...


# ============================================================
#  SDK 直连控制器（飞特舵机 UART）
# ============================================================

class FeetechSDKController(ArmController):
    """
    通过飞特舵机 SDK 直接控制机械臂

    使用 UART 串口通信协议，直接读写舵机寄存器。
    适用于无 ROS2 环境的场景。

    增强功能：
      - ping / scan 单个或全部舵机
      - 逐关节读写 + 限位校验
      - SYNC_WRITE 同步写入多个舵机
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._connected = False

    def connect(self) -> bool:
        """建立串口连接"""
        try:
            import serial
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                write_timeout=1.0,
            )
            self._connected = True
            logger.info(f"SDK 控制器已连接: {self.port} @ {self.baudrate}")
            # 初始化舵机模式
            self._init_servos()
            return True
        except ImportError:
            logger.error("缺少 pyserial 库，请执行: pip install pyserial")
            return False
        except Exception as e:
            logger.error(f"串口连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """关闭串口连接"""
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._connected = False
        logger.info("SDK 控制器已断开连接")

    def _init_servos(self) -> None:
        """初始化所有舵机为位置模式"""
        for jc in JOINT_REGISTRY:
            self._write_servo_mode(jc.servo_id, mode=0)
        logger.info(f"已初始化 {len(JOINT_REGISTRY)} 个舵机为位置模式")

    # ================================================================
    #  底层协议：飞特 SCS/STS
    # ================================================================

    def _checksum(self, packet: bytes) -> int:
        """计算飞特协议校验和"""
        return (~sum(packet[2:])) & 0xFF

    def _build_packet(self, servo_id: int, instruction: int,
                      params: List[int]) -> bytes:
        """
        构建数据包

        帧格式: [0xFF, 0xFF, ID, Length, Instruction, Param1, ..., Checksum]
        """
        length = len(params) + 2
        packet = bytes([0xFF, 0xFF, servo_id, length, instruction] + params)
        chk = self._checksum(packet)
        return packet + bytes([chk])

    def _send_packet(self, packet: bytes) -> Optional[bytes]:
        """发送数据包并读取回复"""
        if not self._serial or not self._serial.is_open:
            logger.error("串口未打开")
            return None
        # 清空接收缓冲区
        self._serial.reset_input_buffer()
        self._serial.write(packet)
        time.sleep(0.003)  # 等待舵机响应
        if self._serial.in_waiting:
            return self._serial.read(self._serial.in_waiting)
        return None

    def _write_servo_mode(self, servo_id: int, mode: int = 0) -> None:
        """写入舵机工作模式（寄存器 0x21）"""
        packet = self._build_packet(servo_id, 0x03, [0x21, mode])
        self._send_packet(packet)

    # ================================================================
    #  单位转换（每个关节独立）
    # ================================================================

    def _rad_to_servo_pos(self, jc: JointConfig, rad: float) -> int:
        """
        弧度 → 舵机原始位置值 (0-4095)

        针对旋转关节和夹爪使用不同转换逻辑。
        """
        if jc.joint_type == "gripper":
            return self._gripper_to_servo_pos(rad)
        # 旋转关节：中心 2048 = 0 rad，范围 0-4095 = -180°~+180°
        deg = np.degrees(rad)
        pos = int(2048 + (deg / 360.0) * 4096)
        return max(0, min(4095, pos))

    def _servo_pos_to_value(self, jc: JointConfig, pos: int) -> float:
        """
        舵机原始位置值 → 弧度（旋转关节）或开合度（夹爪）
        """
        if jc.joint_type == "gripper":
            return self._servo_pos_to_gripper(pos)
        deg = ((pos - 2048) / 4096.0) * 360.0
        return float(np.radians(deg))

    @staticmethod
    def _gripper_to_servo_pos(value: float) -> int:
        """夹爪开合度 (0-1) → 舵机位置"""
        value = max(0.0, min(1.0, value))
        return int(value * 4095)

    @staticmethod
    def _servo_pos_to_gripper(pos: int) -> float:
        """舵机位置 → 夹爪开合度"""
        return max(0.0, min(1.0, pos / 4095.0))

    # ================================================================
    #  读写寄存器
    # ================================================================

    def _write_position(self, servo_id: int, position: int,
                        speed: Optional[int] = None) -> None:
        """
        写入目标位置

        寄存器 0x2A: 2字节位置 + 2字节时间 + 2字节速度
        """
        jc = JOINT_BY_ID.get(servo_id)
        spd = speed if speed is not None else (jc.max_speed if jc else 500)
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        spd_l = spd & 0xFF
        spd_h = (spd >> 8) & 0xFF
        params = [0x2A, pos_l, pos_h, 0, 0, spd_l, spd_h]
        packet = self._build_packet(servo_id, 0x03, params)
        self._send_packet(packet)

    def _read_position(self, servo_id: int) -> Optional[int]:
        """读取当前位置（寄存器 0x38，2字节）"""
        packet = self._build_packet(servo_id, 0x02, [0x38, 2])
        resp = self._send_packet(packet)
        if resp and len(resp) >= 8:
            pos = resp[5] | (resp[6] << 8)
            return pos
        return None

    def _read_temperature(self, servo_id: int) -> Optional[int]:
        """读取舵机温度（寄存器 0x3F，1字节）"""
        packet = self._build_packet(servo_id, 0x02, [0x3F, 1])
        resp = self._send_packet(packet)
        if resp and len(resp) >= 7:
            return resp[5]
        return None

    def _read_load(self, servo_id: int) -> Optional[int]:
        """读取舵机负载（寄存器 0x3C，2字节）"""
        packet = self._build_packet(servo_id, 0x02, [0x3C, 2])
        resp = self._send_packet(packet)
        if resp and len(resp) >= 8:
            return resp[5] | (resp[6] << 8)
        return None

    # ================================================================
    #  PING / SCAN
    # ================================================================

    def _ping(self, servo_id: int) -> bool:
        """
        Ping 单个舵机，判断是否在线

        发送 PING 指令 (0x01)，检查是否有有效响应。
        """
        packet = self._build_packet(servo_id, 0x01, [])
        resp = self._send_packet(packet)
        return resp is not None and len(resp) >= 6

    def scan_servos(self) -> Dict[int, bool]:
        """
        扫描所有 14 个预期舵机的在线状态

        返回:
            {servo_id: online} 字典
        """
        logger.info("开始扫描舵机...")
        result: Dict[int, bool] = {}
        for jc in JOINT_REGISTRY:
            online = self._ping(jc.servo_id)
            result[jc.servo_id] = online
            status = "在线 ✓" if online else "离线 ✗"
            logger.info(f"  ID={jc.servo_id:2d}  {jc.name:30s}  {jc.description:20s}  {status}")
        online_count = sum(1 for v in result.values() if v)
        logger.info(f"扫描完成: {online_count}/{len(JOINT_REGISTRY)} 个舵机在线")
        return result

    # ================================================================
    #  SYNC_WRITE —— 同时写入多个舵机
    # ================================================================

    def _sync_write_positions(self, targets: Dict[int, int]) -> None:
        """
        同步写入多个舵机的目标位置

        使用 SYNC_WRITE 指令 (0x83)，一次写入所有目标舵机。
        比逐个 _write_position 快得多。

        参数:
            targets: {servo_id: raw_position}
        """
        if not targets:
            return
        # SYNC_WRITE 格式:
        # [0xFF, 0xFF, 0xFE, Length, 0x83, StartAddr, DataLen, ID1, Data1..., ID2, Data2..., Checksum]
        start_addr = 0x2A
        data_per_servo = 6  # pos_l, pos_h, time_l, time_h, spd_l, spd_h
        ids_data = []
        for servo_id, position in targets.items():
            jc = JOINT_BY_ID.get(servo_id)
            spd = jc.max_speed if jc else 500
            pos_l = position & 0xFF
            pos_h = (position >> 8) & 0xFF
            spd_l = spd & 0xFF
            spd_h = (spd >> 8) & 0xFF
            ids_data.extend([servo_id, pos_l, pos_h, 0, 0, spd_l, spd_h])

        length = len(ids_data) + 4  # +4: instruction + start_addr + data_len + checksum
        packet = bytes([0xFF, 0xFF, 0xFE, length, 0x83, start_addr, data_per_servo] + ids_data)
        chk = self._checksum(packet)
        packet = packet + bytes([chk])

        if self._serial and self._serial.is_open:
            self._serial.write(packet)

    # ================================================================
    #  公共 API
    # ================================================================

    def get_state(self) -> ArmState:
        """读取全部 14 关节的当前状态"""
        state = ArmState(timestamp=time.time())
        for jc in JOINT_REGISTRY:
            js = state.joints[jc.name]
            pos = self._read_position(jc.servo_id)
            if pos is not None:
                js.position = self._servo_pos_to_value(jc, pos)
                js.online = True
            else:
                js.online = False
            # 可选：读取温度和负载（如需快速响应可跳过）
            temp = self._read_temperature(jc.servo_id)
            if temp is not None:
                js.temperature = temp
            load = self._read_load(jc.servo_id)
            if load is not None:
                js.load = load
        return state

    def get_single_joint(self, name: str) -> Optional[JointState]:
        """读取单个关节状态"""
        jc = JOINT_BY_NAME.get(name)
        if jc is None:
            logger.error(f"未知关节名: '{name}'")
            return None
        js = JointState(config=jc)
        pos = self._read_position(jc.servo_id)
        if pos is not None:
            js.position = self._servo_pos_to_value(jc, pos)
            js.online = True
        else:
            js.online = False
        temp = self._read_temperature(jc.servo_id)
        if temp is not None:
            js.temperature = temp
        load = self._read_load(jc.servo_id)
        if load is not None:
            js.load = load
        return js

    def set_single_joint(self, name: str, value: float) -> bool:
        """设置单个关节的目标位置（含限位校验）"""
        jc = JOINT_BY_NAME.get(name)
        if jc is None:
            logger.error(f"未知关节名: '{name}'，可用: {JOINT_NAMES}")
            return False
        value = validate_joint_value(name, value)
        raw_pos = self._rad_to_servo_pos(jc, value)
        self._write_position(jc.servo_id, raw_pos)
        logger.info(
            f"单关节已设置: {jc.name} (ID={jc.servo_id}, {jc.description}) → {value:.4f}"
        )
        return True

    def set_joint_positions(self, positions: List[float],
                            arm: ArmSide = ArmSide.BOTH) -> bool:
        """设置关节目标位置（使用 SYNC_WRITE + 逐关节限位）"""
        try:
            validated = validate_positions(positions, arm)
            targets: Dict[int, int] = {}

            if arm == ArmSide.LEFT:
                configs = [j for j in JOINT_REGISTRY
                           if j.side == "left" and j.joint_type == "revolute"]
            elif arm == ArmSide.RIGHT:
                configs = [j for j in JOINT_REGISTRY
                           if j.side == "right" and j.joint_type == "revolute"]
            else:
                configs = [j for j in JOINT_REGISTRY if j.joint_type == "revolute"]

            for i, jc in enumerate(configs):
                if i < len(validated):
                    targets[jc.servo_id] = self._rad_to_servo_pos(jc, validated[i])

            self._sync_write_positions(targets)
            logger.info(f"关节目标已同步写入: {len(targets)} 个关节 (arm={arm.value})")
            return True
        except Exception as e:
            logger.error(f"设置关节位置失败: {e}")
            return False

    def set_gripper(self, value: float, arm: ArmSide = ArmSide.LEFT) -> bool:
        """设置夹爪开合度"""
        try:
            value = max(0.0, min(1.0, value))
            raw_pos = self._gripper_to_servo_pos(value)
            if arm in (ArmSide.LEFT, ArmSide.BOTH):
                self._write_position(LEFT_GRIPPER_ID, raw_pos)
            if arm in (ArmSide.RIGHT, ArmSide.BOTH):
                self._write_position(RIGHT_GRIPPER_ID, raw_pos)
            logger.info(f"夹爪已设置: {value:.2f} (arm={arm.value})")
            return True
        except Exception as e:
            logger.error(f"设置夹爪失败: {e}")
            return False

    def go_home(self) -> bool:
        """回到初始位置（使用 SYNC_WRITE 同时驱动所有舵机）"""
        logger.info("正在回到初始位置...")
        targets: Dict[int, int] = {}
        for jc in JOINT_REGISTRY:
            targets[jc.servo_id] = self._rad_to_servo_pos(jc, jc.home_rad)
        self._sync_write_positions(targets)
        time.sleep(2.0)
        logger.info("已回到初始位置")
        return True

    def execute_trajectory(self, trajectory: np.ndarray, fps: int = 30) -> bool:
        """执行动作轨迹（使用 SYNC_WRITE 逐帧写入）"""
        if trajectory.ndim != 2 or trajectory.shape[1] != 14:
            logger.error(f"轨迹形状错误: 期望 (T, 14), 实际 {trajectory.shape}")
            return False

        dt = 1.0 / fps
        total_steps = trajectory.shape[0]
        logger.info(f"开始执行轨迹: {total_steps} 步, {fps} FPS")

        for step in range(total_steps):
            t_start = time.time()
            action = trajectory[step]

            # 构建同步写入目标（全部 14 关节）
            targets: Dict[int, int] = {}
            for jc in JOINT_REGISTRY:
                val = float(action[jc.index])
                clamped = max(jc.min_rad, min(jc.max_rad, val))
                targets[jc.servo_id] = self._rad_to_servo_pos(jc, clamped)

            self._sync_write_positions(targets)

            # 控制节拍
            elapsed = time.time() - t_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            if (step + 1) % 100 == 0 or step == total_steps - 1:
                logger.info(f"轨迹进度: {step + 1}/{total_steps}")

        logger.info("轨迹执行完成")
        return True

    def emergency_stop(self) -> None:
        """紧急停止：关闭所有舵机扭矩"""
        logger.warning("紧急停止!")
        for jc in JOINT_REGISTRY:
            packet = self._build_packet(jc.servo_id, 0x03, [0x28, 0])
            self._send_packet(packet)
        logger.warning("所有 14 个舵机扭矩已关闭")


# ============================================================
#  ROS2 控制器
# ============================================================

class ROS2Controller(ArmController):
    """
    通过 ROS2 接口控制机械臂

    使用 rclpy 发布关节目标位置，订阅关节状态。
    需要 ROS2 环境已安装且机械臂驱动节点已启动。

    增强功能：
      - 逐关节读写 + 限位校验
      - 使用 JOINT_REGISTRY 命名关节
    """

    # ROS2 话题名称
    JOINT_STATE_TOPIC = "/joint_states"
    JOINT_CMD_TOPIC = "/arm_controller/joint_trajectory"
    GRIPPER_LEFT_TOPIC = "/left_gripper_controller/command"
    GRIPPER_RIGHT_TOPIC = "/right_gripper_controller/command"

    def __init__(self, node_name: str = "openclaw_arm_control"):
        self._node_name = node_name
        self._node = None
        self._connected = False
        self._current_state = ArmState()

        # ROS2 对象占位
        self._state_sub = None
        self._joint_pub = None
        self._left_gripper_pub = None
        self._right_gripper_pub = None

    def connect(self) -> bool:
        """初始化 ROS2 节点"""
        try:
            import rclpy

            if not rclpy.ok():
                rclpy.init()

            self._node = rclpy.create_node(self._node_name)

            from sensor_msgs.msg import JointState as JointStateMsg
            from trajectory_msgs.msg import JointTrajectory
            from std_msgs.msg import Float64

            self._state_sub = self._node.create_subscription(
                JointStateMsg,
                self.JOINT_STATE_TOPIC,
                self._joint_state_callback,
                10,
            )
            self._joint_pub = self._node.create_publisher(
                JointTrajectory, self.JOINT_CMD_TOPIC, 10,
            )
            self._left_gripper_pub = self._node.create_publisher(
                Float64, self.GRIPPER_LEFT_TOPIC, 10,
            )
            self._right_gripper_pub = self._node.create_publisher(
                Float64, self.GRIPPER_RIGHT_TOPIC, 10,
            )

            self._connected = True
            logger.info(f"ROS2 控制器已初始化: 节点 '{self._node_name}'")
            return True
        except ImportError:
            logger.error("缺少 rclpy 库，请确保 ROS2 环境已正确安装和 source")
            return False
        except Exception as e:
            logger.error(f"ROS2 初始化失败: {e}")
            return False

    def disconnect(self) -> None:
        """销毁 ROS2 节点"""
        if self._node:
            self._node.destroy_node()
        self._connected = False
        logger.info("ROS2 控制器已断开")

    def _joint_state_callback(self, msg) -> None:
        """关节状态回调 —— 按 JOINT_REGISTRY 索引映射"""
        try:
            positions = list(msg.position)
            if len(positions) >= 14:
                state = ArmState(timestamp=time.time())
                for jc in JOINT_REGISTRY:
                    if jc.index < len(positions):
                        state.joints[jc.name].position = positions[jc.index]
                        state.joints[jc.name].online = True
                self._current_state = state
        except Exception as e:
            logger.warning(f"解析关节状态失败: {e}")

    def _spin_once(self, timeout_sec: float = 0.1) -> None:
        """执行一次 ROS2 回调"""
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def get_state(self) -> ArmState:
        """读取当前关节状态"""
        self._spin_once()
        return self._current_state

    def get_single_joint(self, name: str) -> Optional[JointState]:
        """读取单个关节状态"""
        self._spin_once()
        if name not in self._current_state.joints:
            logger.error(f"未知关节名: '{name}'")
            return None
        return self._current_state.joints[name]

    def set_single_joint(self, name: str, value: float) -> bool:
        """设置单个关节目标位置"""
        jc = JOINT_BY_NAME.get(name)
        if jc is None:
            logger.error(f"未知关节名: '{name}'")
            return False
        value = validate_joint_value(name, value)

        if jc.joint_type == "gripper":
            return self.set_gripper(
                value, arm=ArmSide.LEFT if jc.side == "left" else ArmSide.RIGHT,
            )

        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration

            msg = JointTrajectory()
            msg.joint_names = [jc.name]
            point = JointTrajectoryPoint()
            point.positions = [float(value)]
            point.time_from_start = Duration(sec=2, nanosec=0)
            msg.points = [point]
            self._joint_pub.publish(msg)
            logger.info(f"ROS2 单关节已设置: {jc.name} → {value:.4f}")
            return True
        except Exception as e:
            logger.error(f"ROS2 单关节设置失败: {e}")
            return False

    def scan_servos(self) -> Dict[int, bool]:
        """扫描在线关节（基于最新状态回调）"""
        self._spin_once(timeout_sec=0.5)
        result: Dict[int, bool] = {}
        for jc in JOINT_REGISTRY:
            js = self._current_state.joints.get(jc.name)
            online = js is not None and js.online
            result[jc.servo_id] = online
            status = "在线 ✓" if online else "离线 ✗"
            logger.info(f"  ID={jc.servo_id:2d}  {jc.name:30s}  {status}")
        online_count = sum(1 for v in result.values() if v)
        logger.info(f"扫描完成 (ROS2): {online_count}/{len(JOINT_REGISTRY)} 个关节在线")
        return result

    def set_joint_positions(self, positions: List[float],
                            arm: ArmSide = ArmSide.BOTH) -> bool:
        """通过 ROS2 发布关节目标位置（含逐关节限位校验）"""
        try:
            validated = validate_positions(positions, arm)

            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration

            msg = JointTrajectory()

            if arm == ArmSide.LEFT:
                configs = [j for j in JOINT_REGISTRY
                           if j.side == "left" and j.joint_type == "revolute"]
            elif arm == ArmSide.RIGHT:
                configs = [j for j in JOINT_REGISTRY
                           if j.side == "right" and j.joint_type == "revolute"]
            else:
                configs = [j for j in JOINT_REGISTRY if j.joint_type == "revolute"]

            msg.joint_names = [c.name for c in configs[:len(validated)]]

            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in validated]
            point.time_from_start = Duration(sec=2, nanosec=0)
            msg.points = [point]

            self._joint_pub.publish(msg)
            logger.info(f"ROS2 关节目标已发布: {len(msg.joint_names)} 个关节 (arm={arm.value})")
            return True
        except Exception as e:
            logger.error(f"ROS2 发布关节目标失败: {e}")
            return False

    def set_gripper(self, value: float, arm: ArmSide = ArmSide.LEFT) -> bool:
        """通过 ROS2 设置夹爪"""
        try:
            from std_msgs.msg import Float64
            msg = Float64()
            msg.data = float(max(0.0, min(1.0, value)))

            if arm in (ArmSide.LEFT, ArmSide.BOTH):
                self._left_gripper_pub.publish(msg)
            if arm in (ArmSide.RIGHT, ArmSide.BOTH):
                self._right_gripper_pub.publish(msg)

            logger.info(f"ROS2 夹爪命令已发布: {value:.2f} (arm={arm.value})")
            return True
        except Exception as e:
            logger.error(f"ROS2 夹爪设置失败: {e}")
            return False

    def go_home(self) -> bool:
        """通过 ROS2 回到初始位置"""
        logger.info("正在回到初始位置 (ROS2)...")
        home_positions = [jc.home_rad for jc in JOINT_REGISTRY
                          if jc.joint_type == "revolute"]
        success = self.set_joint_positions(home_positions, arm=ArmSide.BOTH)
        # 设置夹爪
        for jc in JOINT_REGISTRY:
            if jc.joint_type == "gripper":
                arm_side = ArmSide.LEFT if jc.side == "left" else ArmSide.RIGHT
                self.set_gripper(jc.home_rad, arm=arm_side)
        time.sleep(3.0)
        logger.info("已回到初始位置")
        return success

    def execute_trajectory(self, trajectory: np.ndarray, fps: int = 30) -> bool:
        """通过 ROS2 执行动作轨迹"""
        if trajectory.ndim != 2 or trajectory.shape[1] != 14:
            logger.error(f"轨迹形状错误: 期望 (T, 14), 实际 {trajectory.shape}")
            return False

        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration

            msg = JointTrajectory()
            # 使用注册表中的关节名称
            msg.joint_names = [jc.name for jc in JOINT_REGISTRY]

            dt = 1.0 / fps
            for step in range(trajectory.shape[0]):
                point = JointTrajectoryPoint()
                # 逐关节做限位截断
                positions = []
                for jc in JOINT_REGISTRY:
                    val = float(trajectory[step, jc.index])
                    positions.append(max(jc.min_rad, min(jc.max_rad, val)))
                point.positions = positions
                t = (step + 1) * dt
                point.time_from_start = Duration(
                    sec=int(t), nanosec=int((t % 1) * 1e9),
                )
                msg.points.append(point)

            self._joint_pub.publish(msg)
            total_time = trajectory.shape[0] / fps
            logger.info(
                f"ROS2 轨迹已发布: {trajectory.shape[0]} 步, 总时长 {total_time:.1f}s"
            )
            time.sleep(total_time + 1.0)
            return True
        except Exception as e:
            logger.error(f"ROS2 轨迹执行失败: {e}")
            return False

    def emergency_stop(self) -> None:
        """紧急停止：发布空轨迹以停止运动"""
        logger.warning("紧急停止 (ROS2)!")
        try:
            from trajectory_msgs.msg import JointTrajectory
            msg = JointTrajectory()
            msg.joint_names = []
            msg.points = []
            self._joint_pub.publish(msg)
        except Exception:
            pass


# ============================================================
#  工厂方法
# ============================================================

def create_controller(
    mode: str = "sdk",
    port: str = "/dev/ttyUSB0",
    baudrate: int = 1000000,
) -> ArmController:
    """
    创建机械臂控制器实例

    参数:
        mode: "sdk" 或 "ros2"
        port: 串口端口（仅 SDK 模式）
        baudrate: 串口波特率（仅 SDK 模式）

    返回:
        ArmController 实例
    """
    if mode == "sdk":
        return FeetechSDKController(port=port, baudrate=baudrate)
    elif mode == "ros2":
        return ROS2Controller()
    else:
        raise ValueError(f"不支持的控制模式: {mode}，请使用 'sdk' 或 'ros2'")


# ============================================================
#  高级操作：任务执行器
# ============================================================

class TaskExecutor:
    """
    任务执行器，封装常见的机械臂操作任务。
    由 OpenClaw Agent 调用。
    """

    def __init__(self, controller: ArmController):
        self.ctrl = controller

    def pick_object(self, arm: ArmSide = ArmSide.LEFT,
                    pre_grasp_joints: Optional[List[float]] = None) -> bool:
        """
        抓取物体

        步骤:
        1. 打开夹爪
        2. 移动到预抓取位置
        3. 关闭夹爪
        """
        logger.info(f"开始抓取物体 (arm={arm.value})")

        # 打开夹爪
        self.ctrl.set_gripper(1.0, arm=arm)
        time.sleep(1.0)

        # 移动到预抓取位置
        if pre_grasp_joints:
            self.ctrl.set_joint_positions(pre_grasp_joints, arm=arm)
            time.sleep(2.0)

        # 关闭夹爪
        self.ctrl.set_gripper(0.0, arm=arm)
        time.sleep(1.0)

        logger.info("抓取完成")
        return True

    def place_object(self, arm: ArmSide = ArmSide.LEFT,
                     place_joints: Optional[List[float]] = None) -> bool:
        """
        放置物体

        步骤:
        1. 移动到放置位置
        2. 打开夹爪
        3. 撤回
        """
        logger.info(f"开始放置物体 (arm={arm.value})")

        if place_joints:
            self.ctrl.set_joint_positions(place_joints, arm=arm)
            time.sleep(2.0)

        self.ctrl.set_gripper(1.0, arm=arm)
        time.sleep(1.0)

        logger.info("放置完成")
        return True

    def pour_water(self, trajectory: Optional[np.ndarray] = None) -> bool:
        """
        执行倒水动作

        如果提供了 Lerobot 生成的轨迹，则按轨迹执行；
        否则使用预设的倒水动作序列。
        """
        logger.info("开始执行倒水任务")

        if trajectory is not None:
            return self.ctrl.execute_trajectory(trajectory, fps=30)

        # 预设倒水动作序列（仅示例，实际需根据标定）
        logger.warning("未提供轨迹，使用预设倒水动作序列（仅示例）")
        steps = [
            # 步骤 1: 抬起手臂到倒水预备位
            [0.0, -0.5, 1.0, 0.0, 0.5, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
            # 步骤 2: 倾斜倒水
            [0.0, -0.5, 1.0, 0.0, 1.2, 0.3, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
            # 步骤 3: 回正
            [0.0, -0.5, 1.0, 0.0, 0.5, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
        ]

        for i, step in enumerate(steps):
            logger.info(f"倒水步骤 {i + 1}/{len(steps)}")
            state = np.array(step, dtype=np.float32)
            self.ctrl.set_joint_positions(state[:6].tolist(), arm=ArmSide.LEFT)
            self.ctrl.set_gripper(state[6], arm=ArmSide.LEFT)
            time.sleep(2.0)

        logger.info("倒水任务完成")
        return True

    def load_and_execute_lerobot_trajectory(self, parquet_path: str) -> bool:
        """
        从 Lerobot 数据集加载并执行动作轨迹

        参数:
            parquet_path: parquet 文件路径（包含 action 列）
        """
        try:
            import pandas as pd
            df = pd.read_parquet(parquet_path)

            if "action" not in df.columns:
                logger.error("parquet 文件中缺少 'action' 列")
                return False

            actions = np.stack(df["action"].values)
            logger.info(f"已加载轨迹: {actions.shape}")
            return self.ctrl.execute_trajectory(actions, fps=30)

        except ImportError:
            logger.error("缺少 pandas 库，请执行: pip install pandas pyarrow")
            return False
        except Exception as e:
            logger.error(f"加载轨迹失败: {e}")
            return False


# ============================================================
#  命令行入口
# ============================================================

def parse_args() -> argparse.Namespace:
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="OpenClaw 机械臂控制技能 —— 支持逐关节识别与控制",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 列出所有关节配置信息
  python main.py --mode sdk --action list_joints

  # 扫描所有舵机在线状态
  python main.py --mode sdk --port COM3 --action scan

  # 读取全部关节状态
  python main.py --mode sdk --port COM3 --action status

  # 读取单个关节
  python main.py --mode sdk --action single_joint --joint-name left_joint_3

  # 设置单个关节到指定弧度
  python main.py --mode sdk --action single_joint --joint-name left_joint_3 --joint-value 0.5

  # 回到初始位置
  python main.py --mode sdk --port /dev/ttyUSB0 --action home

  # 打开左臂夹爪
  python main.py --mode sdk --action gripper --arm left --gripper-value 1.0

  # 设置左臂 6 个关节
  python main.py --mode sdk --action move --arm left --joints 0.1 0.2 0.3 0.0 0.0 0.0

  # 执行倒水
  python main.py --mode sdk --action pour_water

  # ROS2 模式读取状态
  python main.py --mode ros2 --action status

  # 从 Lerobot 数据集执行轨迹
  python main.py --mode sdk --action trajectory --trajectory-file data.parquet
        """,
    )

    parser.add_argument(
        "--mode", type=str, default="sdk", choices=["sdk", "ros2"],
        help="控制模式: sdk (飞特舵机直连) 或 ros2 (ROS2接口)",
    )
    parser.add_argument(
        "--port", type=str, default="/dev/ttyUSB0",
        help="串口端口 (仅 SDK 模式，如 /dev/ttyUSB0 或 COM3)",
    )
    parser.add_argument(
        "--baudrate", type=int, default=1000000,
        help="串口波特率 (仅 SDK 模式，默认 1000000)",
    )
    parser.add_argument(
        "--action", type=str, required=True,
        choices=["status", "home", "move", "gripper", "pour_water",
                 "pick", "place", "trajectory", "stop",
                 "scan", "single_joint", "list_joints"],
        help="要执行的动作",
    )
    parser.add_argument(
        "--arm", type=str, default="left", choices=["left", "right", "both"],
        help="操作的手臂 (默认 left)",
    )
    parser.add_argument(
        "--joints", type=float, nargs="+",
        help="目标关节角度 (弧度)，左/右臂 6 个值，双臂 12 个值",
    )
    parser.add_argument(
        "--joint-name", type=str, default=None,
        help="关节名称 (用于 single_joint 动作，如 left_joint_3)",
    )
    parser.add_argument(
        "--joint-value", type=float, default=None,
        help="关节目标值 (弧度，用于 single_joint 动作；省略则仅读取)",
    )
    parser.add_argument(
        "--gripper-value", type=float, default=0.5,
        help="夹爪开合度: 0.0 (关闭) 到 1.0 (打开)",
    )
    parser.add_argument(
        "--trajectory-file", type=str,
        help="Lerobot 轨迹文件路径 (.parquet 或 .npy)",
    )

    return parser.parse_args()


def main():
    """主函数"""
    args = parse_args()

    # list_joints 不需要连接硬件
    if args.action == "list_joints":
        print("=" * 80)
        print(f"{'名称':<30s} {'ID':>3s}  {'侧':>4s}  {'类型':<8s}  {'索引':>3s}  "
              f"{'最小(rad)':>9s} {'最大(rad)':>9s} {'描述'}")
        print("-" * 80)
        for jc in JOINT_REGISTRY:
            print(f"{jc.name:<30s} {jc.servo_id:3d}  {jc.side:>4s}  {jc.joint_type:<8s}  "
                  f"{jc.index:3d}  {jc.min_rad:9.4f} {jc.max_rad:9.4f} {jc.description}")
        print("=" * 80)
        print(f"共 {len(JOINT_REGISTRY)} 个关节")
        return

    # 创建控制器
    controller = create_controller(
        mode=args.mode,
        port=args.port,
        baudrate=args.baudrate,
    )

    # 连接
    if not controller.connect():
        logger.error("连接失败，退出")
        sys.exit(1)

    try:
        arm_side = ArmSide(args.arm)
        executor = TaskExecutor(controller)

        if args.action == "status":
            state = controller.get_state()
            print(json.dumps(state.to_dict(), indent=2, ensure_ascii=False))

        elif args.action == "scan":
            result = controller.scan_servos()
            print(json.dumps(
                {str(k): v for k, v in result.items()},
                indent=2, ensure_ascii=False,
            ))

        elif args.action == "single_joint":
            if not args.joint_name:
                logger.error("请通过 --joint-name 指定关节名称")
                logger.info(f"可选关节: {JOINT_NAMES}")
                sys.exit(1)
            if args.joint_value is not None:
                # 写入模式
                ok = controller.set_single_joint(args.joint_name, args.joint_value)
                if ok:
                    logger.info(f"已成功设置 {args.joint_name} → {args.joint_value:.4f}")
                else:
                    logger.error("设置失败")
                    sys.exit(1)
            else:
                # 只读模式
                js = controller.get_single_joint(args.joint_name)
                if js:
                    print(json.dumps(js.to_dict(), indent=2, ensure_ascii=False))
                else:
                    logger.error(f"读取关节 {args.joint_name} 失败")
                    sys.exit(1)

        elif args.action == "home":
            controller.go_home()

        elif args.action == "move":
            if not args.joints:
                logger.error("请通过 --joints 参数提供目标关节角度")
                sys.exit(1)
            controller.set_joint_positions(args.joints, arm=arm_side)

        elif args.action == "gripper":
            controller.set_gripper(args.gripper_value, arm=arm_side)

        elif args.action == "pour_water":
            executor.pour_water()

        elif args.action == "pick":
            executor.pick_object(arm=arm_side, pre_grasp_joints=args.joints)

        elif args.action == "place":
            executor.place_object(arm=arm_side, place_joints=args.joints)

        elif args.action == "trajectory":
            if not args.trajectory_file:
                logger.error("请通过 --trajectory-file 参数提供轨迹文件路径")
                sys.exit(1)
            if args.trajectory_file.endswith(".npy"):
                traj = np.load(args.trajectory_file)
                controller.execute_trajectory(traj)
            else:
                executor.load_and_execute_lerobot_trajectory(args.trajectory_file)

        elif args.action == "stop":
            controller.emergency_stop()

    except KeyboardInterrupt:
        logger.warning("用户中断，执行紧急停止")
        controller.emergency_stop()
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
