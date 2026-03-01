#!/usr/bin/env python3
"""
arm-control skill 主脚本
=========================
SO100 机械臂控制技能，直接通过本地 SDK（飞特舵机协议）与硬件底层通信。

机型：SO100 (6-DOF 单臂)
关节：shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
通信：USB 串口 → 飞特舵机控制板 (STS3215)
位置范围：0 ~ 4095（原始舵机值），中位 2047

用法:
    python main.py --port COM7 --action status
    python main.py --port COM7 --action home
    python main.py --port COM7 --action move --joint shoulder_pan --value 2047
    python main.py --port COM7 --action gripper --value 3000
    python main.py --port COM7 --action sequence --file actions.json
    python main.py --port COM7 --action trajectory --file traj.npy --fps 10
    python main.py --port COM7 --action freedrag
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

import os as _os  # noqa: E402 — needed before SDK path insertion
_PROJECT_ROOT = str(Path(__file__).resolve().parents[4])
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from SDK import PortHandler, PacketHandler  # noqa: E402

# ============================================================
#  日志配置
# ============================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("arm-control")

# ============================================================
#  STS3215 寄存器地址
# ============================================================

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_MOVING_SPEED     = 46
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED    = 58

# ============================================================
#  关节注册表
# ============================================================

JOINT_NAMES: List[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


@dataclass
class JointConfig:
    """单个关节的静态配置"""
    name: str           # 关节名称
    motor_id: int       # 电机 ID (1-6)
    min_pos: int        # 最小位置值
    max_pos: int        # 最大位置值
    home_pos: int       # 零位位置
    description: str    # 中文描述


JOINT_REGISTRY: List[JointConfig] = [
    JointConfig("shoulder_pan",  1, 0, 4095, 2047, "底部旋转关节"),
    JointConfig("shoulder_lift", 2, 0, 4095, 2047, "大臂升降关节"),
    JointConfig("elbow_flex",    3, 0, 4095, 2047, "小臂弯曲关节"),
    JointConfig("wrist_flex",    4, 0, 4095, 2047, "手腕俯仰关节"),
    JointConfig("wrist_roll",    5, 0, 4095, 2047, "手腕旋转关节"),
    JointConfig("gripper",       6, 0, 4095, 2047, "夹爪开合"),
]

JOINT_BY_NAME: Dict[str, JointConfig] = {jc.name: jc for jc in JOINT_REGISTRY}
JOINT_BY_ID: Dict[int, JointConfig] = {jc.motor_id: jc for jc in JOINT_REGISTRY}


def clamp_position(name: str, value: int) -> int:
    """将位置值裁剪到安全范围 [0, 4095]"""
    jc = JOINT_BY_NAME.get(name)
    lo = jc.min_pos if jc else 0
    hi = jc.max_pos if jc else 4095
    clamped = max(lo, min(hi, value))
    if clamped != value:
        logger.warning(f"关节 {name} 值 {value} 超出 [{lo}, {hi}]，已裁剪为 {clamped}")
    return clamped


# ============================================================
#  状态数据结构
# ============================================================

@dataclass
class JointState:
    """单个关节运行时状态"""
    name: str
    motor_id: int = 0
    position: int = 0
    description: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "motor_id": self.motor_id,
            "position": self.position,
            "description": self.description,
        }


@dataclass
class ArmState:
    """机械臂完整状态"""
    timestamp: float = 0.0
    joints: Dict[str, JointState] = field(default_factory=dict)

    def __post_init__(self):
        if not self.joints:
            for jc in JOINT_REGISTRY:
                self.joints[jc.name] = JointState(
                    name=jc.name, motor_id=jc.motor_id,
                    description=jc.description,
                )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "joints": {k: v.to_dict() for k, v in self.joints.items()},
        }


# ============================================================
#  SO100 机械臂控制器（SDK 直连）
# ============================================================

class SO100Controller:
    """
    SO100 机械臂控制器 — 直接通过飞特舵机 SDK 底层通信

    架构:
        SO100Controller  →  SDK (PacketHandler + PortHandler)  →  串口  →  舵机

    使用方式:
        ctrl = SO100Controller(port="COM7")
        ctrl.connect()
        state = ctrl.get_state()          # 读取全部关节
        ctrl.set_joint("shoulder_pan", 2047)  # 写入关节
        ctrl.move_smooth({"gripper": 3000})   # 平滑移动
        ctrl.disconnect()
    """

    def __init__(self, port: str = "COM7", baudrate: int = 1_000_000):
        self.port = port
        self.baudrate = baudrate
        self._port_handler: Optional[PortHandler] = None
        self._packet_handler: Optional[PacketHandler] = None
        self._connected = False

    # ----------------------------------------------------------------
    #  连接管理
    # ----------------------------------------------------------------

    def connect(self) -> bool:
        """打开串口并连接到机械臂"""
        try:
            self._port_handler = PortHandler(self.port)
            self._packet_handler = PacketHandler(0.0)

            if not self._port_handler.openPort():
                logger.error(f"无法打开端口 {self.port}")
                return False
            if not self._port_handler.setBaudRate(self.baudrate):
                logger.error(f"无法设置波特率 {self.baudrate}")
                return False

            self._connected = True
            logger.info(f"已连接: port={self.port}, baud={self.baudrate}")

            found = self.scan_motors()
            logger.info(f"检测到 {len(found)}/6 个电机: {found}")
            return True

        except Exception as e:
            logger.error(f"连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """断开连接（自动禁用扭矩）"""
        if self._connected and self._port_handler:
            try:
                self.disable_all_torque()
            except Exception:
                pass
            self._port_handler.closePort()
        self._connected = False
        logger.info("已断开连接")

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ----------------------------------------------------------------
    #  底层电机操作（SDK 直调）
    # ----------------------------------------------------------------

    def ping(self, motor_id: int) -> bool:
        """检测电机是否在线"""
        self._check()
        _, result, _ = self._packet_handler.ping(self._port_handler, motor_id)
        return result == 0

    def scan_motors(self) -> List[int]:
        """扫描 6 个电机，返回在线列表"""
        self._check()
        return [m for m in range(1, 7) if self.ping(m)]

    def enable_torque(self, motor_id: int) -> None:
        """启用电机扭矩"""
        self._check()
        self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self, motor_id: int) -> None:
        """禁用电机扭矩"""
        self._check()
        self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)

    def enable_all_torque(self) -> None:
        for m in range(1, 7):
            self.enable_torque(m)

    def disable_all_torque(self) -> None:
        for m in range(1, 7):
            self.disable_torque(m)

    def read_position(self, motor_id: int) -> int:
        """读取电机当前位置 (0-4095)"""
        self._check()
        data, result, _ = self._packet_handler.read2ByteTxRx(
            self._port_handler, motor_id, ADDR_PRESENT_POSITION)
        return data if result == 0 else 0

    def write_position(self, motor_id: int, position: int) -> None:
        """写入电机目标位置 (0-4095)"""
        self._check()
        position = max(0, min(4095, position))
        self._packet_handler.write2ByteTxRx(
            self._port_handler, motor_id, ADDR_GOAL_POSITION, position)

    def read_all_positions(self) -> Dict[int, int]:
        """读取全部电机位置 → {motor_id: pos}"""
        return {m: self.read_position(m) for m in range(1, 7)}

    def write_all_positions(self, positions: Dict[int, int]) -> None:
        """写入多个电机位置 {motor_id: pos}"""
        for m, p in positions.items():
            self.write_position(m, p)

    # ----------------------------------------------------------------
    #  高级状态读取（按关节名称）
    # ----------------------------------------------------------------

    def get_state(self) -> ArmState:
        """读取全部关节状态"""
        self._check()
        state = ArmState(timestamp=time.time())
        for jc in JOINT_REGISTRY:
            state.joints[jc.name].position = self.read_position(jc.motor_id)
        return state

    def get_joint(self, name: str) -> Optional[JointState]:
        """读取单个关节状态"""
        jc = JOINT_BY_NAME.get(name)
        if jc is None:
            logger.error(f"未知关节: '{name}'，可用: {JOINT_NAMES}")
            return None
        pos = self.read_position(jc.motor_id)
        return JointState(name=jc.name, motor_id=jc.motor_id,
                          position=pos, description=jc.description)

    # ----------------------------------------------------------------
    #  关节控制（按名称，自动启用扭矩）
    # ----------------------------------------------------------------

    def set_joint(self, name: str, value: int) -> bool:
        """设置单个关节位置（自动启用扭矩）"""
        jc = JOINT_BY_NAME.get(name)
        if jc is None:
            logger.error(f"未知关节: '{name}'，可用: {JOINT_NAMES}")
            return False
        value = clamp_position(name, value)
        self.enable_torque(jc.motor_id)
        self.write_position(jc.motor_id, value)
        logger.info(f"已设置: {name} ({jc.description}) → {value}")
        return True

    def set_joints(self, positions: Dict[str, int]) -> bool:
        """设置多个关节 {"名称": 位置}"""
        for name, val in positions.items():
            if not self.set_joint(name, val):
                return False
        return True

    def set_all_joints(self, values: List[int]) -> bool:
        """设置全部 6 个关节 [s_pan, s_lift, e_flex, w_flex, w_roll, gripper]"""
        if len(values) != 6:
            logger.error(f"需要 6 个值，收到 {len(values)} 个")
            return False
        for i, jc in enumerate(JOINT_REGISTRY):
            self.set_joint(jc.name, values[i])
        return True

    # ----------------------------------------------------------------
    #  夹爪
    # ----------------------------------------------------------------

    def open_gripper(self, value: int = 3000) -> bool:
        """打开夹爪（值越大越张开）"""
        return self.set_joint("gripper", value)

    def close_gripper(self, value: int = 1000) -> bool:
        """关闭夹爪（值越小越闭合）"""
        return self.set_joint("gripper", value)

    # ----------------------------------------------------------------
    #  平滑运动
    # ----------------------------------------------------------------

    def move_smooth(self, target: Dict[str, int],
                    steps: int = 30, dt: float = 0.02) -> bool:
        """
        平滑移动到目标位置（线性插值）

        参数:
            target: {关节名: 目标位置}
            steps:  插值步数（越大越平滑）
            dt:     每步间隔秒数
        """
        self._check()
        self.enable_all_torque()

        # 记录起始位置
        start = {}
        for name in target:
            jc = JOINT_BY_NAME.get(name)
            if jc:
                start[name] = self.read_position(jc.motor_id)

        # 逐步插值
        for i in range(1, steps + 1):
            ratio = i / steps
            for name, tgt in target.items():
                jc = JOINT_BY_NAME.get(name)
                if jc:
                    cur = start.get(name, jc.home_pos)
                    pos = int(cur + (tgt - cur) * ratio)
                    self.write_position(jc.motor_id, max(0, min(4095, pos)))
            time.sleep(dt)

        logger.info(f"平滑移动完成 → {list(target.keys())}")
        return True

    # ----------------------------------------------------------------
    #  预设动作
    # ----------------------------------------------------------------

    def go_home(self) -> bool:
        """回到零位（全部 2047）"""
        logger.info("回到初始位置...")
        home = {jc.name: jc.home_pos for jc in JOINT_REGISTRY}
        self.move_smooth(home)
        logger.info("已回到初始位置")
        return True

    # ----------------------------------------------------------------
    #  自由拖动
    # ----------------------------------------------------------------

    def freedrag(self) -> Dict[str, int]:
        """释放扭矩并返回当前位置"""
        self.disable_all_torque()
        return {jc.name: self.read_position(jc.motor_id) for jc in JOINT_REGISTRY}

    # ----------------------------------------------------------------
    #  动作序列 / 轨迹
    # ----------------------------------------------------------------

    def execute_sequence(self, actions: List[Dict[str, int]],
                         interval: float = 2.0) -> bool:
        """执行动作序列 [{关节名: 位置}, ...]"""
        self._check()
        self.enable_all_torque()
        total = len(actions)
        logger.info(f"执行动作序列: {total} 步")
        for i, action in enumerate(actions, 1):
            logger.info(f"步骤 {i}/{total}")
            self.move_smooth(action)
            time.sleep(interval)
        logger.info("序列完成")
        return True

    def execute_trajectory(self, trajectory: np.ndarray,
                           fps: int = 10) -> bool:
        """
        逐帧回放轨迹

        参数:
            trajectory: shape=(T, 6)  每行 [s_pan, s_lift, e_flex, w_flex, w_roll, gripper]
            fps: 帧率
        """
        self._check()
        self.enable_all_torque()
        if trajectory.ndim != 2 or trajectory.shape[1] != 6:
            logger.error(f"轨迹形状错误: 期望 (T,6), 实际 {trajectory.shape}")
            return False

        dt = 1.0 / fps
        total = trajectory.shape[0]
        logger.info(f"执行轨迹: {total} 帧, {fps} FPS")

        for step in range(total):
            t0 = time.time()
            for i, jc in enumerate(JOINT_REGISTRY):
                val = int(trajectory[step, i])
                self.write_position(jc.motor_id, clamp_position(jc.name, val))
            elapsed = time.time() - t0
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)
            if (step + 1) % 50 == 0 or step == total - 1:
                logger.info(f"进度: {step + 1}/{total}")

        logger.info("轨迹完成")
        return True

    # ----------------------------------------------------------------
    #  高级任务
    # ----------------------------------------------------------------

    def pick_object(self, pre_grasp: Optional[Dict[str, int]] = None,
                    grasp: Optional[Dict[str, int]] = None) -> bool:
        """抓取: 张爪 → 预抓取 → 抓取 → 闭爪 → 抬起"""
        logger.info("开始抓取")
        self.open_gripper()
        time.sleep(1.0)
        if pre_grasp:
            self.move_smooth(pre_grasp)
            time.sleep(0.5)
        if grasp:
            self.move_smooth(grasp)
            time.sleep(0.5)
        self.close_gripper()
        time.sleep(1.0)
        if pre_grasp:
            self.move_smooth(pre_grasp)
        logger.info("抓取完成")
        return True

    def place_object(self, place_pos: Optional[Dict[str, int]] = None) -> bool:
        """放置: 移到位 → 张爪"""
        logger.info("开始放置")
        if place_pos:
            self.move_smooth(place_pos)
            time.sleep(0.5)
        self.open_gripper()
        time.sleep(1.0)
        logger.info("放置完成")
        return True

    def pour_water(self) -> bool:
        """预设倒水动作（示例值，需根据实际标定调整）"""
        logger.warning("使用预设倒水序列（仅示例）")
        actions = [
            {"shoulder_pan": 2047, "shoulder_lift": 2500, "elbow_flex": 1500,
             "wrist_flex": 1700, "wrist_roll": 2047, "gripper": 1200},
            {"shoulder_pan": 2047, "shoulder_lift": 2500, "elbow_flex": 1500,
             "wrist_flex": 1700, "wrist_roll": 3000, "gripper": 1200},
            {"shoulder_pan": 2047, "shoulder_lift": 2500, "elbow_flex": 1500,
             "wrist_flex": 1700, "wrist_roll": 2047, "gripper": 1200},
            {"shoulder_pan": 2047, "shoulder_lift": 2047, "elbow_flex": 2047,
             "wrist_flex": 2047, "wrist_roll": 2047, "gripper": 2047},
        ]
        return self.execute_sequence(actions, interval=2.5)

    # ----------------------------------------------------------------
    #  文件加载
    # ----------------------------------------------------------------

    def load_and_execute(self, file_path: str, fps: int = 10) -> bool:
        """从文件加载并执行 (.json 序列 / .npy 轨迹)"""
        path = Path(file_path)
        if not path.exists():
            logger.error(f"文件不存在: {file_path}")
            return False

        ext = path.suffix.lower()
        if ext == ".json":
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if isinstance(data, list):
                return self.execute_sequence(data, interval=2.0)
            logger.error("JSON 应为列表格式")
            return False
        elif ext == ".npy":
            traj = np.load(file_path)
            return self.execute_trajectory(traj.astype(np.float64), fps=fps)
        else:
            logger.error(f"不支持的格式: {ext}")
            return False

    # ----------------------------------------------------------------
    #  工具
    # ----------------------------------------------------------------

    def _check(self) -> None:
        if not self._connected:
            raise RuntimeError("机械臂未连接，请先调用 connect()")

    def emergency_stop(self) -> None:
        """紧急停止"""
        logger.warning("紧急停止!")
        self.disconnect()


# ============================================================
#  工厂函数
# ============================================================

def create_controller(port: str = "COM7",
                      baudrate: int = 1_000_000) -> SO100Controller:
    """创建控制器实例"""
    return SO100Controller(port=port, baudrate=baudrate)


# ============================================================
#  命令行入口
# ============================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SO100 机械臂控制技能 (SDK 直连)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python main.py --action list_joints
  python main.py --port COM7 --action status
  python main.py --port COM7 --action home
  python main.py --port COM7 --action move --joint shoulder_pan --value 2047
  python main.py --port COM7 --action move_all --values 2047 2047 2047 2047 2047 2047
  python main.py --port COM7 --action gripper --value 3000
  python main.py --port COM7 --action freedrag
  python main.py --port COM7 --action pour_water
  python main.py --port COM7 --action pick
  python main.py --port COM7 --action sequence --file actions.json
  python main.py --port COM7 --action stop
  python main.py --port COM7 --action scan
        """,
    )
    parser.add_argument("--port", default="COM7", help="串口端口号")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="波特率")
    parser.add_argument(
        "--action", required=True,
        choices=["list_joints", "status", "home", "move", "move_all",
                 "gripper", "freedrag", "pour_water", "pick", "place",
                 "sequence", "trajectory", "stop", "scan"],
        help="要执行的动作",
    )
    parser.add_argument("--joint", default=None, help="关节名称")
    parser.add_argument("--value", type=int, default=None, help="目标位置 (0-4095)")
    parser.add_argument("--values", type=int, nargs=6, default=None, help="6 个关节目标值")
    parser.add_argument("--file", default=None, help="轨迹/序列文件路径")
    parser.add_argument("--fps", type=int, default=10, help="轨迹回放帧率")
    return parser.parse_args()


def main():
    args = parse_args()

    # list_joints 不需要硬件
    if args.action == "list_joints":
        print("=" * 65)
        print(f"{'名称':<16} {'电机ID':>5}  {'范围':>14}  {'零位':>4}  {'描述'}")
        print("-" * 65)
        for jc in JOINT_REGISTRY:
            print(f"{jc.name:<16} {jc.motor_id:5d}  [{jc.min_pos:>4d}, {jc.max_pos:>4d}]"
                  f"  {jc.home_pos:>4d}  {jc.description}")
        print("=" * 65)
        return

    ctrl = create_controller(port=args.port, baudrate=args.baudrate)
    if not ctrl.connect():
        sys.exit(1)

    try:
        if args.action == "scan":
            found = ctrl.scan_motors()
            print(f"在线电机: {found} ({len(found)}/6)")

        elif args.action == "status":
            state = ctrl.get_state()
            print(json.dumps(state.to_dict(), indent=2, ensure_ascii=False))

        elif args.action == "home":
            ctrl.go_home()

        elif args.action == "move":
            if not args.joint:
                logger.error(f"需要 --joint, 可选: {JOINT_NAMES}")
                sys.exit(1)
            if args.value is None:
                js = ctrl.get_joint(args.joint)
                if js:
                    print(json.dumps(js.to_dict(), indent=2, ensure_ascii=False))
            else:
                ctrl.set_joint(args.joint, args.value)

        elif args.action == "move_all":
            if not args.values:
                logger.error("需要 --values (6 个值)")
                sys.exit(1)
            ctrl.set_all_joints(args.values)

        elif args.action == "gripper":
            if args.value is None:
                logger.error("需要 --value (0-4095)")
                sys.exit(1)
            ctrl.set_joint("gripper", args.value)

        elif args.action == "freedrag":
            ctrl.disable_all_torque()
            print("扭矩已释放，可自由拖动。Ctrl+C 退出。\n")
            try:
                while True:
                    parts = []
                    for jc in JOINT_REGISTRY:
                        pos = ctrl.read_position(jc.motor_id)
                        parts.append(f"{jc.name}: {pos:>4d}")
                    print("\r" + "  ".join(parts), end="", flush=True)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n已退出")

        elif args.action == "pour_water":
            ctrl.pour_water()

        elif args.action == "pick":
            ctrl.pick_object()

        elif args.action == "place":
            ctrl.place_object()

        elif args.action == "sequence":
            if not args.file:
                logger.error("需要 --file (.json)")
                sys.exit(1)
            ctrl.load_and_execute(args.file)

        elif args.action == "trajectory":
            if not args.file:
                logger.error("需要 --file (.npy)")
                sys.exit(1)
            ctrl.load_and_execute(args.file, fps=args.fps)

        elif args.action == "stop":
            ctrl.emergency_stop()

    except KeyboardInterrupt:
        ctrl.emergency_stop()
    except RuntimeError as e:
        logger.error(f"错误: {e}")
        ctrl.emergency_stop()
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
