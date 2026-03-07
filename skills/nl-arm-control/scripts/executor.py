#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自然语言控制机械臂 - 代码执行器
解析自然语言意图，生成并执行控制代码

注意：直接使用 SDK，避免 importlib 与 dataclass 的兼容性问题
"""

import os
import sys
import time
import re
from pathlib import Path
from typing import Optional, Dict, List, Tuple
from datetime import datetime

# 添加项目根目录到路径
PROJECT_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT_ROOT))

# 直接导入 SDK（避免 dataclass 兼容性问题）
from SDK import PortHandler, PacketHandler

# ============================================================
#  STS3215 寄存器地址（复制自 arm-control）
# ============================================================
ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56

# 关节配置
JOINT_CONFIG = {
    "shoulder_pan":  {"id": 1, "name": "底部旋转", "home": 2047},
    "shoulder_lift": {"id": 2, "name": "大臂升降", "home": 2047},
    "elbow_flex":    {"id": 3, "name": "小臂弯曲", "home": 2047},
    "wrist_flex":    {"id": 4, "name": "手腕俯仰", "home": 2047},
    "wrist_roll":    {"id": 5, "name": "手腕旋转", "home": 2047},
    "gripper":       {"id": 6, "name": "夹爪",    "home": 2047},
}

JOINT_NAMES = list(JOINT_CONFIG.keys())
MOTOR_NAME_TO_ID = {cfg["name"]: i for i, cfg in enumerate(JOINT_CONFIG.values(), start=1)}


# ============================================================
#  简化的机械臂控制器（直接使用 SDK）
# ============================================================

class SimpleArmController:
    """简化的机械臂控制器，直接使用 SDK 通信"""

    def __init__(self, port: str = "COM7", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self._port_handler = None
        self._packet_handler = None
        self._connected = False

    def connect(self) -> bool:
        """连接到机械臂"""
        try:
            self._port_handler = PortHandler(self.port)
            self._packet_handler = PacketHandler(0.0)

            if not self._port_handler.openPort():
                print(f"错误: 无法打开端口 {self.port}")
                return False
            if not self._port_handler.setBaudRate(self.baudrate):
                print(f"错误: 无法设置波特率 {self.baudrate}")
                return False

            self._connected = True
            print(f"已连接: {self.port} @ {self.baudrate}")

            # 扫描电机
            motors = self.scan_motors()
            print(f"检测到 {len(motors)}/6 个电机: {motors}")
            return True

        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """断开连接"""
        if self._connected and self._port_handler:
            self.disable_all_torque()
            self._port_handler.closePort()
        self._connected = False
        print("已断开连接")

    def scan_motors(self) -> List[int]:
        """扫描在线电机"""
        return [m for m in range(1, 7) if self.ping(m)]

    def ping(self, motor_id: int) -> bool:
        """检测电机是否在线"""
        _, result, _ = self._packet_handler.ping(self._port_handler, motor_id)
        return result == 0

    def enable_torque(self, motor_id: int) -> None:
        """启用电机扭矩"""
        self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self, motor_id: int) -> None:
        """禁用电机扭矩"""
        self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)

    def enable_all_torque(self) -> None:
        for m in range(1, 7):
            self.enable_torque(m)

    def disable_all_torque(self) -> None:
        for m in range(1, 7):
            self.disable_torque(m)

    def read_position(self, motor_id: int) -> int:
        """读取电机位置"""
        data, result, _ = self._packet_handler.read2ByteTxRx(
            self._port_handler, motor_id, ADDR_PRESENT_POSITION)
        return data if result == 0 else 0

    def write_position(self, motor_id: int, position: int) -> None:
        """写入电机位置"""
        position = max(0, min(4095, position))
        self._packet_handler.write2ByteTxRx(
            self._port_handler, motor_id, ADDR_GOAL_POSITION, position)

    def get_joint_id(self, joint_name: str) -> Optional[int]:
        """获取关节对应的电机 ID"""
        cfg = JOINT_CONFIG.get(joint_name)
        return cfg["id"] if cfg else None

    def set_joint(self, joint_name: str, value: int) -> bool:
        """设置关节位置"""
        motor_id = self.get_joint_id(joint_name)
        if motor_id is None:
            print(f"错误: 未知关节 '{joint_name}'，可用: {JOINT_NAMES}")
            return False

        value = max(0, min(4095, value))
        self.enable_torque(motor_id)
        self.write_position(motor_id, value)
        cfg = JOINT_CONFIG[joint_name]
        print(f"已设置: {joint_name} ({cfg['name']}) -> {value}")
        return True

    def move_smooth(self, targets: Dict[str, int], steps: int = 30, dt: float = 0.02) -> bool:
        """平滑移动到目标位置"""
        if not targets:
            return False

        # 记录起始位置
        start = {}
        for name in targets:
            motor_id = self.get_joint_id(name)
            if motor_id:
                start[name] = self.read_position(motor_id)

        self.enable_all_torque()

        # 逐步插值
        for i in range(1, steps + 1):
            ratio = i / steps
            for name, tgt in targets.items():
                motor_id = self.get_joint_id(name)
                if motor_id:
                    cur = start.get(name, 2047)
                    pos = int(cur + (tgt - cur) * ratio)
                    self.write_position(motor_id, max(0, min(4095, pos)))
            time.sleep(dt)

        print(f"平滑移动完成: {list(targets.keys())}")
        return True

    def open_gripper(self, value: int = 3000) -> bool:
        """打开夹爪"""
        return self.set_joint("gripper", value)

    def close_gripper(self, value: int = 1000) -> bool:
        """关闭夹爪"""
        return self.set_joint("gripper", value)

    def go_home(self) -> bool:
        """回到零位"""
        home_pos = {name: cfg["home"] for name, cfg in JOINT_CONFIG.items()}
        print("回到零位...")
        self.move_smooth(home_pos)
        print("已回到零位")
        return True

    def pick_object(self, pre_grasp: Optional[Dict[str, int]] = None,
                    grasp: Optional[Dict[str, int]] = None) -> bool:
        """抓取物体"""
        print("开始抓取...")
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
        print("抓取完成")
        return True

    def place_object(self, place_pos: Optional[Dict[str, int]] = None) -> bool:
        """放置物体"""
        print("开始放置...")
        if place_pos:
            self.move_smooth(place_pos)
            time.sleep(0.5)
        self.open_gripper()
        time.sleep(1.0)
        print("放置完成")
        return True

    def freedrag(self) -> Dict[str, int]:
        """自由拖动模式"""
        self.disable_all_torque()
        print("扭矩已释放，可自由拖动 (Ctrl+C 退出)...")

        try:
            while True:
                positions = {}
                for name, cfg in JOINT_CONFIG.items():
                    pos = self.read_position(cfg["id"])
                    positions[name] = pos
                print(f"\r位置: {positions}", end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n退出自由拖动")
            self.enable_all_torque()
            return positions


# ============================================================
#  意图解析器
# ============================================================

class IntentParser:
    """自然语言意图解析器"""

    JOINT_ALIASES = {
        # shoulder_pan (ID: 1)
        "底座": "shoulder_pan", "底部旋转": "shoulder_pan", "底座旋转": "shoulder_pan",
        "转盘": "shoulder_pan", "水平旋转": "shoulder_pan", "旋转": "shoulder_pan",
        "关节1": "shoulder_pan", "电机1": "shoulder_pan",

        # shoulder_lift (ID: 2)
        "大臂": "shoulder_lift", "大臂升降": "shoulder_lift", "肩部抬起": "shoulder_lift",
        "大臂抬起": "shoulder_lift", "肩部": "shoulder_lift",
        "关节2": "shoulder_lift", "电机2": "shoulder_lift",

        # elbow_flex (ID: 3)
        "小臂": "elbow_flex", "肘部": "elbow_flex", "小臂弯曲": "elbow_flex",
        "肘关节": "elbow_flex",
        "关节3": "elbow_flex", "电机3": "elbow_flex",

        # wrist_flex (ID: 4)
        "手腕俯仰": "wrist_flex", "手腕上下": "wrist_flex", "手腕弯曲": "wrist_flex",
        "关节4": "wrist_flex", "电机4": "wrist_flex",

        # wrist_roll (ID: 5)
        "手腕旋转": "wrist_roll", "手腕转动": "wrist_roll", "手腕扭转": "wrist_roll",
        "关节5": "wrist_roll", "电机5": "wrist_roll",

        # gripper (ID: 6)
        "夹爪": "gripper", "抓手": "gripper", "手爪": "gripper",
        "关节6": "gripper", "电机6": "gripper",
    }

    POSITION_MAP = {
        "最左": 0, "最右": 4095,
        "左": 1500, "右": 2600,
        "最高": 0, "最低": 4095,
        "上": 1500, "下": 2600,
        "中间": 2047, "中间位置": 2047, "初始位置": 2047, "零位": 2047,
        "张开": 3000, "打开": 3000,
        "闭合": 1000, "关闭": 1000, "合上": 1000,
    }

    ACTION_KEYWORDS = {
        "gripper_open": ["张开", "打开", "松开", "松爪"],
        "gripper_close": ["闭合", "关闭", "合上", "抓紧", "闭爪"],
        "home": ["回零", "回零位", "回到初始", "回到中间", "复位", "归位"],
        "move": ["移动", "转到", "移到", "转到位置", "设置", "设为"],
        "pick": ["抓", "抓取", "抓起", "拿起", "夹取", "夹起"],
        "place": ["放", "放置", "放下", "松开物体"],
        "pour": ["倒", "倾倒", "翻转"],
        "freedrag": ["自由拖动", "释放", "手动模式"],
        "scan": ["扫描", "检测", "检查电机"],
    }

    def parse(self, text: str) -> Dict:
        """解析自然语言输入"""
        result = {"action": None, "joints": [], "values": {}, "target": None, "raw": text}

        # 1. 识别动作类型（按优先级排序，更具体的在前）
        action_priority = [
            "gripper_open", "gripper_close",
            "home", "move", "pick", "place", "pour",
            "freedrag", "scan",
        ]

        for action in action_priority:
            keywords = self.ACTION_KEYWORDS.get(action, [])
            if any(kw in text for kw in keywords):
                result["action"] = action
                break

        # 2. 识别关节
        for alias, joint_name in self.JOINT_ALIASES.items():
            if alias in text:
                if joint_name not in result["joints"]:
                    result["joints"].append(joint_name)

        # 3. 识别位置值
        for pos_name, pos_value in self.POSITION_MAP.items():
            if pos_name in text:
                if result["joints"]:
                    for joint in result["joints"]:
                        result["values"][joint] = pos_value
                elif pos_name in ["张开", "打开", "闭合", "关闭", "合上"]:
                    result["values"]["gripper"] = pos_value

        # 4. 提取数字作为位置值
        numbers = re.findall(r'\d+', text)
        if numbers and result["joints"]:
            for i, num in enumerate(numbers):
                if i < len(result["joints"]):
                    result["values"][result["joints"][i]] = int(num)

        # 5. 识别目标物体
        objects = ["杯子", "杯", "瓶子", "球", "方块", "物体", "东西", "item", "cup", "bottle", "ball"]
        for obj in objects:
            if obj in text:
                result["target"] = obj
                break

        # 6. 默认动作推断
        if result["action"] is None:
            if result["joints"]:
                result["action"] = "move"
            elif "零" in text or "初始" in text or "中间" in text:
                result["action"] = "home"

        return result


# ============================================================
#  执行器
# ============================================================

class ArmExecutor:
    """机械臂执行器 - 直接执行命令，不生成代码"""

    def __init__(self, port: str = "COM7"):
        self.port = port
        self.parser = IntentParser()
        self.controller = None

    def execute(self, user_input: str) -> Dict:
        """解析并执行用户指令"""
        result = {
            "input": user_input,
            "intent": None,
            "success": False,
            "message": "",
        }

        # 解析意图
        intent = self.parser.parse(user_input)
        result["intent"] = intent

        print(f"\n[解析] 动作: {intent.get('action')}")
        print(f"[解析] 关节: {intent.get('joints')}")
        print(f"[解析] 值: {intent.get('values')}")
        if intent.get('target'):
            print(f"[解析] 目标: {intent.get('target')}")

        # 获取动作
        action = intent.get("action")
        if not action:
            result["message"] = "未能理解指令，请重试"
            return result

        # 创建控制器
        if self.controller is None:
            self.controller = SimpleArmController(self.port)

        try:
            # 连接（如果未连接）
            if not self.controller._connected:
                if not self.controller.connect():
                    result["message"] = "连接失败"
                    return result

            # 执行动作
            self._dispatch(action, intent, result)
            result["success"] = True

        except KeyboardInterrupt:
            print("\n中断执行")
            result["message"] = "用户中断"
        except Exception as e:
            result["message"] = f"执行错误: {e}"
            print(f"错误: {e}")

        return result

    def _dispatch(self, action: str, intent: Dict, result: Dict):
        """分发动作到具体执行方法"""
        handlers = {
            "gripper_open": self._gripper_open,
            "gripper_close": self._gripper_close,
            "home": self._home,
            "move": self._move,
            "pick": self._pick,
            "place": self._place,
            "freedrag": self._freedrag,
            "scan": self._scan,
        }

        handler = handlers.get(action)
        if handler:
            handler(intent, result)
        else:
            result["message"] = f"动作 '{action}' 暂未实现"

    def _gripper_open(self, intent: Dict, result: Dict):
        self.controller.open_gripper()
        result["message"] = "夹爪已张开"

    def _gripper_close(self, intent: Dict, result: Dict):
        self.controller.close_gripper()
        result["message"] = "夹爪已闭合"

    def _home(self, intent: Dict, result: Dict):
        self.controller.go_home()
        result["message"] = "已回到零位"

    def _move(self, intent: Dict, result: Dict):
        joints = intent.get("joints", [])
        values = intent.get("values", {})

        if not joints:
            result["message"] = "请指定要移动的关节"
            return

        # 构建目标位置
        targets = {}
        for joint in joints:
            targets[joint] = values.get(joint, 2047)

        self.controller.move_smooth(targets)
        result["message"] = f"已移动: {targets}"

    def _pick(self, intent: Dict, result: Dict):
        target = intent.get("target", "物体")
        self.controller.pick_object()
        result["message"] = f"抓取{target}完成"

    def _place(self, intent: Dict, result: Dict):
        self.controller.place_object()
        result["message"] = "放置完成"

    def _freedrag(self, intent: Dict, result: Dict):
        self.controller.freedrag()
        result["message"] = "自由拖动已退出"

    def _scan(self, intent: Dict, result: Dict):
        motors = self.controller.scan_motors()
        print(f"\n在线电机: {motors} ({len(motors)}/6)")
        for motor_id in motors:
            pos = self.controller.read_position(motor_id)
            print(f"  电机{motor_id}: {pos}")
        result["message"] = f"扫描完成: {len(motors)}/6 在线"

    def disconnect(self):
        """断开连接"""
        if self.controller:
            self.controller.disconnect()


# ============================================================
#  命令行入口
# ============================================================

def main():
    """命令行入口"""
    import argparse

    parser = argparse.ArgumentParser(description="自然语言控制 SO100 机械臂")
    parser.add_argument("input", nargs="?", help="自然语言指令")
    parser.add_argument("--port", default="COM7", help="串口端口号")
    args = parser.parse_args()

    executor = ArmExecutor(port=args.port)

    try:
        if args.input:
            # 直接执行指令
            result = executor.execute(args.input)
            print(f"\n[结果] {result['message']}")
        else:
            # 交互式模式
            print("=" * 50)
            print("自然语言控制 SO100 机械臂")
            print("=" * 50)
            print("\n可用指令示例:")
            print("  - 张开夹爪 / 闭合夹爪")
            print("  - 回到初始位置 / 回零位")
            print("  - 把大臂移动到 2500")
            print("  - 抓起杯子 / 放置物体")
            print("  - 进入自由拖动模式")
            print("  - 扫描电机")
            print("\n输入 'quit' 或 'exit' 退出\n")

            while True:
                try:
                    user_input = input("nl-arm> ").strip()
                    if not user_input:
                        continue
                    if user_input.lower() in ['quit', 'exit', 'q']:
                        print("再见！")
                        break

                    result = executor.execute(user_input)
                    print(f"\n[结果] {result['message']}\n")

                except KeyboardInterrupt:
                    print("\n再见！")
                    break
                except Exception as e:
                    print(f"错误: {e}\n")

    finally:
        executor.disconnect()


if __name__ == "__main__":
    main()
