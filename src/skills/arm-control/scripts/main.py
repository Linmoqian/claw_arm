#!/usr/bin/env python3
"""
arm-control skill 主脚本
=========================
SO100 机械臂控制技能，基于 LeRobot 框架的 SO100Follower API。

机型：SO100 (6-DOF 单臂)
关节：shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
通信：USB 串口 → LeRobot SO100Follower
位置范围：-100 ~ 100（夹爪 0 ~ 100）

用法:
    python main.py --port COM7 --action status
    python main.py --port COM7 --action home
    python main.py --port COM7 --action move --joint shoulder_pan --value 30
    python main.py --port COM7 --action gripper --value 80
    python main.py --port COM7 --action sequence
    python main.py --port COM7 --action trajectory --file data.parquet
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

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
#  关节注册表
# ============================================================

# SO100 机械臂的 6 个关节（与 LeRobot SO100Follower 一致）
JOINT_NAMES: List[str] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# 每个关节的位置键名（LeRobot API 使用 "关节名.pos" 格式）
JOINT_POS_KEYS: List[str] = [f"{name}.pos" for name in JOINT_NAMES]


@dataclass
class JointConfig:
    """单个关节的静态配置"""
    name: str           # 关节名称
    motor_id: int       # 电机 ID (1-6)
    pos_key: str        # LeRobot API 位置键名 (e.g. "shoulder_pan.pos")
    min_pos: float      # 最小位置值
    max_pos: float      # 最大位置值
    home_pos: float     # 初始/零位位置
    description: str    # 中文描述


# 关节注册表：定义 SO100 全部 6 个关节
JOINT_REGISTRY: List[JointConfig] = [
    JointConfig("shoulder_pan",  1, "shoulder_pan.pos",  -100.0, 100.0, 0.0, "底部旋转关节"),
    JointConfig("shoulder_lift", 2, "shoulder_lift.pos", -100.0, 100.0, 0.0, "大臂升降关节"),
    JointConfig("elbow_flex",    3, "elbow_flex.pos",    -100.0, 100.0, 0.0, "小臂弯曲关节"),
    JointConfig("wrist_flex",    4, "wrist_flex.pos",    -100.0, 100.0, 0.0, "手腕俯仰关节"),
    JointConfig("wrist_roll",    5, "wrist_roll.pos",    -100.0, 100.0, 0.0, "手腕旋转关节"),
    JointConfig("gripper",       6, "gripper.pos",        0.0,   100.0, 0.0, "夹爪开合"),
]

# 快速查找字典
JOINT_BY_NAME: Dict[str, JointConfig] = {jc.name: jc for jc in JOINT_REGISTRY}
JOINT_BY_KEY: Dict[str, JointConfig] = {jc.pos_key: jc for jc in JOINT_REGISTRY}


def clamp_joint_value(name: str, value: float) -> float:
    """将关节值裁剪到其安全范围内"""
    jc = JOINT_BY_NAME.get(name)
    if jc is None:
        return value
    clamped = max(jc.min_pos, min(jc.max_pos, value))
    if clamped != value:
        logger.warning(
            f"关节 {name} ({jc.description}) 值 {value:.2f} 超出范围 "
            f"[{jc.min_pos}, {jc.max_pos}]，已裁剪为 {clamped:.2f}"
        )
    return clamped


# ============================================================
#  状态数据结构
# ============================================================

@dataclass
class JointState:
    """单个关节运行时状态"""
    name: str
    position: float = 0.0
    description: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "position": round(self.position, 2),
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
                    name=jc.name, description=jc.description,
                )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "joints": {k: v.to_dict() for k, v in self.joints.items()},
        }

    def to_action_dict(self) -> Dict[str, float]:
        """转换为 LeRobot send_action 格式"""
        return {
            f"{name}.pos": js.position
            for name, js in self.joints.items()
        }


# ============================================================
#  SO100 机械臂控制器
# ============================================================

class SO100Controller:
    """
    SO100 机械臂控制器

    封装 LeRobot 的 SO100Follower API，提供：
      - 连接/断开管理
      - 单个关节读写
      - 全关节状态读取
      - 夹爪控制
      - 动作序列执行
      - 轨迹回放
      - 安全限位

    使用方式:
        ctrl = SO100Controller(port="COM7")
        ctrl.connect()
        state = ctrl.get_state()
        ctrl.set_joint("shoulder_pan", 30.0)
        ctrl.disconnect()
    """

    def __init__(
        self,
        port: str = "COM7",
        robot_id: str = "my_so100_arm",
        calibrate: bool = False,
        max_relative_target: Optional[float] = None,
        lerobot_src_path: Optional[str] = None,
    ):
        """
        参数:
            port: USB 串口端口号
            robot_id: 机械臂唯一标识符（校准数据关联）
            calibrate: 连接时是否执行校准
            max_relative_target: 单次最大相对移动幅度（None=不限制）
            lerobot_src_path: LeRobot 源码 src 目录路径（None=自动检测）
        """
        self.port = port
        self.robot_id = robot_id
        self.calibrate = calibrate
        self.max_relative_target = max_relative_target
        self.lerobot_src_path = lerobot_src_path

        self._robot = None
        self._connected = False

    # ----------------------------------------------------------------
    #  连接管理
    # ----------------------------------------------------------------

    def connect(self) -> bool:
        """
        连接到 SO100 机械臂

        返回:
            True=成功, False=失败
        """
        try:
            # 确保 lerobot 可导入
            self._ensure_lerobot_importable()

            from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

            config = SO100FollowerConfig(
                port=self.port,
                id=self.robot_id,
                disable_torque_on_disconnect=True,
            )
            if self.max_relative_target is not None:
                config.max_relative_target = self.max_relative_target

            self._robot = SO100Follower(config)
            self._robot.connect(calibrate=self.calibrate)
            self._connected = True

            logger.info(
                f"SO100 机械臂已连接: port={self.port}, id={self.robot_id}"
            )
            return True

        except ConnectionError as e:
            logger.error(f"串口连接失败 (端口可能被占用): {e}")
            return False
        except RuntimeError as e:
            if "calibration" in str(e).lower():
                logger.error(
                    f"机械臂未校准: {e}\n"
                    f"请运行: lerobot-calibrate --robot.type=so100_follower "
                    f"--robot.port={self.port} --robot.id={self.robot_id}"
                )
            else:
                logger.error(f"连接运行时错误: {e}")
            return False
        except ImportError as e:
            logger.error(
                f"无法导入 LeRobot: {e}\n"
                "请确保已安装: pip install -e '.[feetech]'"
            )
            return False
        except Exception as e:
            logger.error(f"连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """断开连接（自动禁用扭矩）"""
        if self._robot:
            try:
                self._robot.disconnect()
            except Exception as e:
                logger.warning(f"断开时出错: {e}")
        self._connected = False
        logger.info("SO100 机械臂已断开连接")

    def _ensure_lerobot_importable(self) -> None:
        """确保 lerobot 模块可导入"""
        try:
            import lerobot  # noqa: F401
            return
        except ImportError:
            pass

        # 尝试自动查找 lerobot/src 目录
        search_paths = []
        if self.lerobot_src_path:
            search_paths.append(self.lerobot_src_path)
        # 从当前项目向上查找
        search_paths.extend([
            str(Path(__file__).resolve().parents[4] / "src"),  # claw_arm/src 的上级
            str(Path.cwd() / "src"),
            os.path.expanduser("~/lerobot/src"),
        ])
        for p in search_paths:
            if Path(p).exists() and (Path(p) / "lerobot").exists():
                sys.path.insert(0, p)
                logger.info(f"已自动添加 LeRobot 路径: {p}")
                return

        raise ImportError("未找到 lerobot 模块，请设置 lerobot_src_path 或安装 lerobot")

    @property
    def is_connected(self) -> bool:
        return self._connected and self._robot is not None

    # ----------------------------------------------------------------
    #  状态读取
    # ----------------------------------------------------------------

    def get_state(self) -> ArmState:
        """读取机械臂全部关节当前状态"""
        self._check_connected()
        obs = self._robot.get_observation()

        state = ArmState(timestamp=time.time())
        for jc in JOINT_REGISTRY:
            if jc.pos_key in obs:
                state.joints[jc.name].position = float(obs[jc.pos_key])
        return state

    def get_joint(self, name: str) -> Optional[JointState]:
        """
        读取单个关节状态

        参数:
            name: 关节名称 (如 "shoulder_pan", "gripper")
        """
        if name not in JOINT_BY_NAME:
            logger.error(f"未知关节名: '{name}'，可用: {JOINT_NAMES}")
            return None

        state = self.get_state()
        return state.joints.get(name)

    def get_observation_raw(self) -> Dict[str, float]:
        """获取 LeRobot 原始观测数据"""
        self._check_connected()
        return self._robot.get_observation()

    # ----------------------------------------------------------------
    #  关节控制
    # ----------------------------------------------------------------

    def set_joint(self, name: str, value: float) -> bool:
        """
        设置单个关节的目标位置

        参数:
            name: 关节名称
            value: 目标位置值 (-100~100, 夹爪 0~100)
        """
        self._check_connected()
        jc = JOINT_BY_NAME.get(name)
        if jc is None:
            logger.error(f"未知关节名: '{name}'，可用: {JOINT_NAMES}")
            return False

        value = clamp_joint_value(name, value)

        # 构建完整动作（其他关节保持当前位置）
        obs = self._robot.get_observation()
        action = {}
        for j in JOINT_REGISTRY:
            if j.name == name:
                action[j.pos_key] = value
            elif j.pos_key in obs:
                action[j.pos_key] = float(obs[j.pos_key])
            else:
                action[j.pos_key] = j.home_pos

        self._robot.send_action(action)
        logger.info(f"关节已设置: {name} ({jc.description}) → {value:.2f}")
        return True

    def set_joints(self, positions: Dict[str, float]) -> bool:
        """
        设置多个关节的目标位置

        参数:
            positions: {关节名: 目标值} 字典，如 {"shoulder_pan": 30, "elbow_flex": -45}
        """
        self._check_connected()

        # 从当前观测构建基础动作
        obs = self._robot.get_observation()
        action = {}
        for jc in JOINT_REGISTRY:
            if jc.name in positions:
                action[jc.pos_key] = clamp_joint_value(jc.name, positions[jc.name])
            elif jc.pos_key in obs:
                action[jc.pos_key] = float(obs[jc.pos_key])
            else:
                action[jc.pos_key] = jc.home_pos

        self._robot.send_action(action)
        logger.info(f"多关节已设置: {list(positions.keys())}")
        return True

    def set_all_joints(self, values: List[float]) -> bool:
        """
        设置全部 6 个关节的位置

        参数:
            values: 长度为 6 的列表，按 shoulder_pan, shoulder_lift, elbow_flex,
                    wrist_flex, wrist_roll, gripper 顺序
        """
        if len(values) != 6:
            logger.error(f"需要 6 个关节值，实际提供 {len(values)} 个")
            return False

        positions = {}
        for i, jc in enumerate(JOINT_REGISTRY):
            positions[jc.name] = values[i]
        return self.set_joints(positions)

    def send_action(self, action: Dict[str, float]) -> Dict[str, float]:
        """
        直接发送原始 LeRobot 动作（键为 "关节名.pos"）

        参数:
            action: {"shoulder_pan.pos": 10.0, "gripper.pos": 50.0, ...}

        返回:
            实际发送的动作（可能被安全限制裁剪）
        """
        self._check_connected()

        # 值裁剪
        safe_action = {}
        for key, val in action.items():
            jc = JOINT_BY_KEY.get(key)
            if jc:
                safe_action[key] = clamp_joint_value(jc.name, val)
            else:
                safe_action[key] = val

        result = self._robot.send_action(safe_action)
        return result if result else safe_action

    # ----------------------------------------------------------------
    #  夹爪控制
    # ----------------------------------------------------------------

    def open_gripper(self, value: float = 80.0) -> bool:
        """打开夹爪（默认 80%）"""
        return self.set_joint("gripper", value)

    def close_gripper(self, value: float = 0.0) -> bool:
        """关闭夹爪（默认完全关闭）"""
        return self.set_joint("gripper", value)

    # ----------------------------------------------------------------
    #  预设动作
    # ----------------------------------------------------------------

    def go_home(self) -> bool:
        """
        回到初始位置（全部关节归零）
        """
        logger.info("正在回到初始位置...")
        home_action = {jc.pos_key: jc.home_pos for jc in JOINT_REGISTRY}
        self._check_connected()
        self._robot.send_action(home_action)
        time.sleep(2.0)
        logger.info("已回到初始位置")
        return True

    def move_to(self, action: Dict[str, float], wait: float = 2.0) -> bool:
        """
        移动到指定位置并等待

        参数:
            action: 目标动作 (可以是部分关节)
            wait: 等待时间（秒）
        """
        self._check_connected()

        # 补全缺失关节
        obs = self._robot.get_observation()
        full_action = {}
        for jc in JOINT_REGISTRY:
            if jc.pos_key in action:
                full_action[jc.pos_key] = clamp_joint_value(jc.name, action[jc.pos_key])
            elif jc.name in action:
                full_action[jc.pos_key] = clamp_joint_value(jc.name, action[jc.name])
            elif jc.pos_key in obs:
                full_action[jc.pos_key] = float(obs[jc.pos_key])
            else:
                full_action[jc.pos_key] = jc.home_pos

        self._robot.send_action(full_action)
        time.sleep(wait)
        return True

    # ----------------------------------------------------------------
    #  动作序列
    # ----------------------------------------------------------------

    def execute_sequence(self, actions: List[Dict[str, float]],
                         interval: float = 2.0) -> bool:
        """
        执行动作序列

        参数:
            actions: 动作列表，每个动作是 {"关节名.pos": 值} 字典
            interval: 动作间等待时间（秒）
        """
        self._check_connected()
        total = len(actions)
        logger.info(f"开始执行动作序列: {total} 步")

        for i, action in enumerate(actions, 1):
            logger.info(f"步骤 {i}/{total}")
            safe_action = {}
            for key, val in action.items():
                jc = JOINT_BY_KEY.get(key)
                if jc:
                    safe_action[key] = clamp_joint_value(jc.name, val)
                else:
                    safe_action[key] = val
            self._robot.send_action(safe_action)
            time.sleep(interval)

        logger.info("动作序列执行完成")
        return True

    def execute_trajectory(self, trajectory: np.ndarray,
                           fps: int = 10) -> bool:
        """
        执行动作轨迹（逐帧回放）

        参数:
            trajectory: shape = (T, 6)，每行按
                        [shoulder_pan, shoulder_lift, elbow_flex,
                         wrist_flex, wrist_roll, gripper] 排列
            fps: 回放帧率
        """
        self._check_connected()

        if trajectory.ndim != 2 or trajectory.shape[1] != 6:
            logger.error(
                f"轨迹形状错误: 期望 (T, 6), 实际 {trajectory.shape}"
            )
            return False

        dt = 1.0 / fps
        total_steps = trajectory.shape[0]
        logger.info(f"开始执行轨迹: {total_steps} 步, {fps} FPS")

        for step in range(total_steps):
            t_start = time.time()

            action = {}
            for i, jc in enumerate(JOINT_REGISTRY):
                val = float(trajectory[step, i])
                action[jc.pos_key] = clamp_joint_value(jc.name, val)

            self._robot.send_action(action)

            # 控制节拍
            elapsed = time.time() - t_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            if (step + 1) % 50 == 0 or step == total_steps - 1:
                logger.info(f"轨迹进度: {step + 1}/{total_steps}")

        logger.info("轨迹执行完成")
        return True

    # ----------------------------------------------------------------
    #  高级任务
    # ----------------------------------------------------------------

    def pick_object(self, pre_grasp: Optional[Dict[str, float]] = None,
                    grasp: Optional[Dict[str, float]] = None) -> bool:
        """
        抓取物体

        步骤:
          1. 打开夹爪
          2. 移动到预抓取位置
          3. 下降到抓取位置
          4. 关闭夹爪
          5. 抬起（回到预抓取位置）
        """
        logger.info("开始抓取物体")

        self.open_gripper()
        time.sleep(1.0)

        if pre_grasp:
            logger.info("[2/5] 移动到预抓取位置")
            self.move_to(pre_grasp, wait=2.0)

        if grasp:
            logger.info("[3/5] 下降到抓取位置")
            self.move_to(grasp, wait=1.5)

        logger.info("[4/5] 关闭夹爪")
        self.close_gripper()
        time.sleep(1.0)

        if pre_grasp:
            logger.info("[5/5] 抬起物体")
            self.move_to(pre_grasp, wait=2.0)

        logger.info("抓取完成")
        return True

    def place_object(self, place_pos: Optional[Dict[str, float]] = None) -> bool:
        """
        放置物体

        步骤:
          1. 移动到放置位置
          2. 打开夹爪
        """
        logger.info("开始放置物体")

        if place_pos:
            self.move_to(place_pos, wait=2.0)

        self.open_gripper()
        time.sleep(1.0)

        logger.info("放置完成")
        return True

    def pour_water(self, trajectory: Optional[np.ndarray] = None) -> bool:
        """
        执行倒水动作

        如果提供了轨迹则按轨迹执行，否则使用预设动作序列。
        """
        logger.info("开始执行倒水任务")

        if trajectory is not None:
            return self.execute_trajectory(trajectory, fps=10)

        # 预设倒水动作序列（示例值，需根据实际标定调整）
        logger.warning("未提供轨迹，使用预设倒水动作序列（仅示例）")
        actions = [
            {   # 步骤 1: 抬起手臂到倒水预备位
                "shoulder_pan.pos": 0.0,
                "shoulder_lift.pos": 30.0,
                "elbow_flex.pos": -45.0,
                "wrist_flex.pos": -20.0,
                "wrist_roll.pos": 0.0,
                "gripper.pos": 10.0,
            },
            {   # 步骤 2: 倾斜倒水
                "shoulder_pan.pos": 0.0,
                "shoulder_lift.pos": 30.0,
                "elbow_flex.pos": -45.0,
                "wrist_flex.pos": -20.0,
                "wrist_roll.pos": 60.0,
                "gripper.pos": 10.0,
            },
            {   # 步骤 3: 回正
                "shoulder_pan.pos": 0.0,
                "shoulder_lift.pos": 30.0,
                "elbow_flex.pos": -45.0,
                "wrist_flex.pos": -20.0,
                "wrist_roll.pos": 0.0,
                "gripper.pos": 10.0,
            },
            {   # 步骤 4: 放回
                "shoulder_pan.pos": 0.0,
                "shoulder_lift.pos": 0.0,
                "elbow_flex.pos": 0.0,
                "wrist_flex.pos": 0.0,
                "wrist_roll.pos": 0.0,
                "gripper.pos": 10.0,
            },
        ]
        return self.execute_sequence(actions, interval=2.5)

    # ----------------------------------------------------------------
    #  Lerobot 轨迹加载
    # ----------------------------------------------------------------

    def load_and_execute_trajectory(self, file_path: str,
                                    fps: int = 10) -> bool:
        """
        从文件加载并执行动作轨迹

        支持格式:
          - .parquet (Lerobot 数据集)
          - .npy (numpy 数组)
          - .json (动作序列列表)

        参数:
            file_path: 文件路径
            fps: 回放帧率（仅 parquet/npy）
        """
        path = Path(file_path)
        if not path.exists():
            logger.error(f"文件不存在: {file_path}")
            return False

        suffix = path.suffix.lower()

        if suffix == ".npy":
            traj = np.load(file_path)
            logger.info(f"已加载 numpy 轨迹: {traj.shape}")
            return self.execute_trajectory(traj, fps=fps)

        elif suffix == ".parquet":
            return self._load_parquet_trajectory(file_path, fps)

        elif suffix == ".json":
            return self._load_json_sequence(file_path)

        else:
            logger.error(f"不支持的文件格式: {suffix}")
            return False

    def _load_parquet_trajectory(self, file_path: str, fps: int) -> bool:
        """从 parquet 加载轨迹"""
        try:
            import pandas as pd
            df = pd.read_parquet(file_path)

            # 尝试查找动作列
            if "action" in df.columns:
                actions = np.stack(df["action"].values)
            else:
                # 尝试按关节名提取列
                cols = [f"{name}.pos" for name in JOINT_NAMES]
                available = [c for c in cols if c in df.columns]
                if not available:
                    logger.error(f"parquet 中未找到动作数据，可用列: {list(df.columns)}")
                    return False
                actions = df[available].values

            if actions.ndim == 2 and actions.shape[1] >= 6:
                actions = actions[:, :6]

            logger.info(f"已加载 parquet 轨迹: {actions.shape}")
            return self.execute_trajectory(actions.astype(np.float64), fps=fps)

        except ImportError:
            logger.error("缺少 pandas 库，请执行: pip install pandas pyarrow")
            return False
        except Exception as e:
            logger.error(f"加载 parquet 失败: {e}")
            return False

    def _load_json_sequence(self, file_path: str) -> bool:
        """从 JSON 加载动作序列"""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            if isinstance(data, list):
                return self.execute_sequence(data, interval=2.0)
            else:
                logger.error("JSON 文件应为动作列表格式")
                return False
        except Exception as e:
            logger.error(f"加载 JSON 失败: {e}")
            return False

    # ----------------------------------------------------------------
    #  工具方法
    # ----------------------------------------------------------------

    def _check_connected(self) -> None:
        """检查连接状态"""
        if not self.is_connected:
            raise RuntimeError("机械臂未连接，请先调用 connect()")

    def emergency_stop(self) -> None:
        """紧急停止：断开连接（自动禁用扭矩）"""
        logger.warning("紧急停止!")
        self.disconnect()


# ============================================================
#  工厂函数
# ============================================================

def create_controller(
    port: str = "COM7",
    robot_id: str = "my_so100_arm",
    calibrate: bool = False,
    max_relative_target: Optional[float] = None,
    lerobot_src_path: Optional[str] = None,
) -> SO100Controller:
    """
    创建 SO100 控制器实例

    参数:
        port: 串口端口号
        robot_id: 机械臂 ID
        calibrate: 是否校准
        max_relative_target: 安全移动限制
        lerobot_src_path: LeRobot src 路径
    """
    return SO100Controller(
        port=port,
        robot_id=robot_id,
        calibrate=calibrate,
        max_relative_target=max_relative_target,
        lerobot_src_path=lerobot_src_path,
    )


# ============================================================
#  命令行入口
# ============================================================

def parse_args() -> argparse.Namespace:
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="SO100 机械臂控制技能 (LeRobot)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 查看所有关节配置
  python main.py --action list_joints

  # 读取当前状态
  python main.py --port COM7 --action status

  # 回到初始位置
  python main.py --port COM7 --action home

  # 设置单个关节
  python main.py --port COM7 --action move --joint shoulder_pan --value 30

  # 设置全部关节
  python main.py --port COM7 --action move_all --values 0 30 -45 -20 0 50

  # 打开夹爪
  python main.py --port COM7 --action gripper --value 80

  # 关闭夹爪
  python main.py --port COM7 --action gripper --value 0

  # 执行预设倒水动作
  python main.py --port COM7 --action pour_water

  # 执行预设抓取动作
  python main.py --port COM7 --action pick

  # 从文件执行轨迹
  python main.py --port COM7 --action trajectory --file data.parquet

  # 紧急停止
  python main.py --port COM7 --action stop
        """,
    )

    parser.add_argument(
        "--port", type=str, default="COM7",
        help="串口端口号 (默认 COM7)",
    )
    parser.add_argument(
        "--robot-id", type=str, default="my_so100_arm",
        help="机械臂唯一标识符 (默认 my_so100_arm)",
    )
    parser.add_argument(
        "--calibrate", action="store_true", default=False,
        help="连接时执行校准",
    )
    parser.add_argument(
        "--max-relative-target", type=float, default=None,
        help="单次最大相对移动幅度 (安全限制)",
    )
    parser.add_argument(
        "--lerobot-src", type=str, default=None,
        help="LeRobot 源码 src 目录路径",
    )
    parser.add_argument(
        "--action", type=str, required=True,
        choices=[
            "list_joints", "status", "home", "move", "move_all",
            "gripper", "pour_water", "pick", "place",
            "sequence", "trajectory", "stop",
        ],
        help="要执行的动作",
    )
    parser.add_argument(
        "--joint", type=str, default=None,
        help="关节名称 (用于 move 动作，如 shoulder_pan)",
    )
    parser.add_argument(
        "--value", type=float, default=None,
        help="目标值 (用于 move/gripper 动作)",
    )
    parser.add_argument(
        "--values", type=float, nargs=6, default=None,
        help="全部 6 个关节目标值 (用于 move_all 动作)",
    )
    parser.add_argument(
        "--file", type=str, default=None,
        help="轨迹 / 动作序列文件路径 (.parquet / .npy / .json)",
    )
    parser.add_argument(
        "--fps", type=int, default=10,
        help="轨迹回放帧率 (默认 10)",
    )

    return parser.parse_args()


def main():
    """主函数"""
    args = parse_args()

    # list_joints 不需要连接硬件
    if args.action == "list_joints":
        print("=" * 75)
        print(
            f"{'名称':<16s} {'电机ID':>5s}  {'API键名':<24s}  "
            f"{'范围':>14s}  {'描述'}"
        )
        print("-" * 75)
        for jc in JOINT_REGISTRY:
            rng = f"[{jc.min_pos:6.1f}, {jc.max_pos:5.1f}]"
            print(
                f"{jc.name:<16s} {jc.motor_id:5d}  {jc.pos_key:<24s}  "
                f"{rng:>14s}  {jc.description}"
            )
        print("=" * 75)
        print(f"共 {len(JOINT_REGISTRY)} 个关节")
        return

    # 创建控制器
    ctrl = create_controller(
        port=args.port,
        robot_id=args.robot_id,
        calibrate=args.calibrate,
        max_relative_target=args.max_relative_target,
        lerobot_src_path=args.lerobot_src,
    )

    # 连接
    if not ctrl.connect():
        logger.error("连接失败，退出")
        sys.exit(1)

    try:
        if args.action == "status":
            state = ctrl.get_state()
            print(json.dumps(state.to_dict(), indent=2, ensure_ascii=False))

        elif args.action == "home":
            ctrl.go_home()

        elif args.action == "move":
            if not args.joint:
                logger.error(f"请通过 --joint 指定关节名，可选: {JOINT_NAMES}")
                sys.exit(1)
            if args.value is None:
                # 只读模式
                js = ctrl.get_joint(args.joint)
                if js:
                    print(json.dumps(js.to_dict(), indent=2, ensure_ascii=False))
                else:
                    sys.exit(1)
            else:
                ctrl.set_joint(args.joint, args.value)

        elif args.action == "move_all":
            if not args.values:
                logger.error("请通过 --values 提供 6 个关节目标值")
                sys.exit(1)
            ctrl.set_all_joints(args.values)

        elif args.action == "gripper":
            if args.value is None:
                logger.error("请通过 --value 指定夹爪开合度 (0-100)")
                sys.exit(1)
            ctrl.set_joint("gripper", args.value)

        elif args.action == "pour_water":
            ctrl.pour_water()

        elif args.action == "pick":
            ctrl.pick_object()

        elif args.action == "place":
            ctrl.place_object()

        elif args.action == "sequence":
            if not args.file:
                logger.error("请通过 --file 指定动作序列文件 (.json)")
                sys.exit(1)
            ctrl.load_and_execute_trajectory(args.file)

        elif args.action == "trajectory":
            if not args.file:
                logger.error("请通过 --file 指定轨迹文件 (.parquet / .npy)")
                sys.exit(1)
            ctrl.load_and_execute_trajectory(args.file, fps=args.fps)

        elif args.action == "stop":
            ctrl.emergency_stop()

    except KeyboardInterrupt:
        logger.warning("用户中断，执行紧急停止")
        ctrl.emergency_stop()
    except RuntimeError as e:
        logger.error(f"运行错误: {e}")
        ctrl.emergency_stop()
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
