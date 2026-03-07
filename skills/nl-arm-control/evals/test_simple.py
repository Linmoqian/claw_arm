#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化测试：仅测试意图解析和代码生成，不依赖 arm-control 模块
"""

import re
from typing import Dict, List, Optional, Tuple


class IntentParser:
    """自然语言意图解析器"""

    JOINT_ALIASES = {
        "底座": "shoulder_pan", "底部旋转": "shoulder_pan", "底座旋转": "shoulder_pan",
        "转盘": "shoulder_pan", "水平旋转": "shoulder_pan", "旋转": "shoulder_pan",
        "大臂": "shoulder_lift", "大臂升降": "shoulder_lift", "肩部抬起": "shoulder_lift",
        "小臂": "elbow_flex", "肘部": "elbow_flex", "小臂弯曲": "elbow_flex",
        "手腕俯仰": "wrist_flex", "手腕上下": "wrist_flex",
        "手腕旋转": "wrist_roll", "手腕转动": "wrist_roll",
        "夹爪": "gripper", "抓手": "gripper", "手爪": "gripper",
    }

    POSITION_MAP = {
        "最左": 0, "最右": 4095, "左": 1500, "右": 2600,
        "最高": 0, "最低": 4095, "上": 1500, "下": 2600,
        "中间": 2047, "中间位置": 2047, "初始位置": 2047, "零位": 2047,
        "张开": 3000, "打开": 3000, "闭合": 1000, "关闭": 1000, "合上": 1000,
    }

    ACTION_KEYWORDS = {
        "gripper_open": ["张开", "打开", "松开"],
        "gripper_close": ["闭合", "关闭", "合上", "抓紧"],
        "home": ["回零", "回零位", "回到初始", "回到中间", "复位"],
        "move": ["移动", "转到", "移到"],
        "pick": ["抓", "抓取", "抓起", "拿起"],
        "place": ["放", "放置", "放下"],
        "freedrag": ["自由拖动", "释放", "手动模式"],
    }

    def parse(self, text: str) -> Dict:
        result = {"action": None, "joints": [], "values": {}, "target": None}

        # 识别动作
        for action, keywords in self.ACTION_KEYWORDS.items():
            if any(kw in text for kw in keywords):
                result["action"] = action
                break

        # 识别关节
        for alias, joint_name in self.JOINT_ALIASES.items():
            if alias in text:
                result["joints"].append(joint_name)

        # 识别位置值
        for pos_name, pos_value in self.POSITION_MAP.items():
            if pos_name in text:
                for joint in result["joints"]:
                    result["values"][joint] = pos_value
                if not result["joints"] and pos_name in ["张开", "打开", "闭合", "关闭", "合上"]:
                    result["values"]["gripper"] = pos_value

        # 提取数字
        numbers = re.findall(r'\d+', text)
        if numbers and result["joints"]:
            for i, num in enumerate(numbers):
                if i < len(result["joints"]):
                    result["values"][result["joints"][i]] = int(num)

        # 识别目标物体
        objects = ["杯子", "杯", "瓶子", "球", "物体"]
        for obj in objects:
            if obj in text:
                result["target"] = obj
                break

        # 默认动作推断
        if result["action"] is None:
            if result["joints"]:
                result["action"] = "move"
            elif "零" in text or "初始" in text or "中间" in text:
                result["action"] = "home"

        return result


class CodeGenerator:
    """代码生成器"""

    def generate(self, intent: Dict) -> Tuple[str, str]:
        action = intent.get("action", "home")

        if action == "gripper_open":
            return self._gripper_code(True), "张开夹爪"
        elif action == "gripper_close":
            return self._gripper_code(False), "闭合夹爪"
        elif action == "home":
            return self._home_code(), "回到零位"
        elif action == "move":
            return self._move_code(intent), f"移动关节"
        elif action == "pick":
            return self._pick_code(intent), "抓取物体"
        elif action == "freedrag":
            return self._freedrag_code(), "自由拖动模式"
        else:
            return self._home_code(), "回到零位"

    def _gripper_code(self, open: bool) -> str:
        action = "open_gripper(3000)" if open else "close_gripper(1000)"
        return f'''ctrl.connect()
ctrl.{action}
ctrl.disconnect()'''

    def _home_code(self) -> str:
        return '''ctrl.connect()
ctrl.go_home()
ctrl.disconnect()'''

    def _move_code(self, intent: Dict) -> str:
        joints = intent.get("joints", ["shoulder_pan"])
        values = intent.get("values", {joints[0]: 2047} if joints else {})
        params = ", ".join([f'"{j}": {values.get(j, 2047)}' for j in joints])
        return f'''ctrl.connect()
ctrl.move_smooth({{{params}}})
ctrl.disconnect()'''

    def _pick_code(self, intent: Dict) -> str:
        target = intent.get("target", "物体")
        return f'''ctrl.connect()
print("抓取{target}...")
ctrl.pick_object()
ctrl.disconnect()'''

    def _freedrag_code(self) -> str:
        return '''ctrl.connect()
print("自由拖动模式 (Ctrl+C 退出)")
ctrl.freedrag()
ctrl.disconnect()'''


def test_parser():
    """测试解析器"""
    print("=" * 60)
    print("测试意图解析器")
    print("=" * 60)

    parser = IntentParser()
    test_cases = [
        ("张开夹爪", "gripper_open", ["gripper"]),
        ("闭合夹爪", "gripper_close", ["gripper"]),
        ("回到初始位置", "home", []),
        ("把大臂移动到 2500", "move", ["shoulder_lift"]),
        ("抓起杯子", "pick", []),
        ("进入自由拖动模式", "freedrag", []),
    ]

    passed = 0
    for input_text, expected_action, expected_joints in test_cases:
        result = parser.parse(input_text)
        action_ok = result["action"] == expected_action
        joints_ok = set(result["joints"]) == set(expected_joints)

        status = "[OK]" if (action_ok and joints_ok) else "[FAIL]"
        print(f"\n{status} 输入: {input_text}")
        print(f"  预期: action={expected_action}, joints={expected_joints}")
        print(f"  实际: action={result['action']}, joints={result['joints']}")

        if action_ok and joints_ok:
            passed += 1

    print(f"\n解析器: {passed}/{len(test_cases)} 通过")
    return passed == len(test_cases)


def test_generator():
    """测试代码生成器"""
    print("\n" + "=" * 60)
    print("测试代码生成器")
    print("=" * 60)

    generator = CodeGenerator()
    parser = IntentParser()

    test_inputs = [
        "张开夹爪",
        "回到初始位置",
        "把大臂移动到 2500",
        "抓起杯子",
        "进入自由拖动模式",
    ]

    passed = 0
    for user_input in test_inputs:
        intent = parser.parse(user_input)
        code, desc = generator.generate(intent)

        # 基本验证
        has_ctrl = "ctrl" in code
        has_connect = "connect" in code
        has_disconnect = "disconnect" in code

        ok = has_ctrl and has_connect and has_disconnect
        status = "[OK]" if ok else "[FAIL]"
        print(f"\n{status} 输入: {user_input}")
        print(f"  描述: {desc}")
        print(f"  代码: {code[:50]}...")

        if ok:
            passed += 1

    print(f"\n代码生成器: {passed}/{len(test_inputs)} 通过")
    return passed == len(test_inputs)


def main():
    """运行所有测试"""
    print("自然语言控制机械臂技能 - 简化测试\n")

    r1 = test_parser()
    r2 = test_generator()

    print("\n" + "=" * 60)
    print("总结")
    print("=" * 60)
    print(f"  解析器: {'[OK] 通过' if r1 else '[FAIL] 失败'}")
    print(f"  代码生成: {'[OK] 通过' if r2 else '[FAIL] 失败'}")
    print(f"\n  整体: {'[OK] 全部通过' if r1 and r2 else '[FAIL] 部分失败'}")

    return 0 if (r1 and r2) else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
