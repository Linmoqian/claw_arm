#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试意图解析器"""

import re
from typing import Dict, List, Optional

class IntentParser:
    """自然语言意图解析器"""

    JOINT_ALIASES = {
        "底座": "shoulder_pan", "底部旋转": "shoulder_pan", "底座旋转": "shoulder_pan",
        "转盘": "shoulder_pan", "水平旋转": "shoulder_pan", "旋转": "shoulder_pan",
        "大臂": "shoulder_lift", "大臂升降": "shoulder_lift", "肩部抬起": "shoulder_lift",
        "小臂": "elbow_flex", "肘部": "elbow_flex", "小臂弯曲": "elbow_flex",
        "手腕俯仰": "wrist_flex", "手腕上下": "wrist_flex", "手腕弯曲": "wrist_flex",
        "手腕旋转": "wrist_roll", "手腕转动": "wrist_roll", "手腕扭转": "wrist_roll",
        "夹爪": "gripper", "抓手": "gripper", "手爪": "gripper",
    }

    POSITION_MAP = {
        "最左": 0, "最右": 4095, "左": 1500, "右": 2600,
        "最高": 0, "最低": 4095, "上": 1500, "下": 2600,
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
        "freedrag": ["自由拖动", "释放", "手动模式"],
        "scan": ["扫描", "检测", "检查电机"],
    }

    def parse(self, text: str) -> Dict:
        result = {"action": None, "joints": [], "values": {}, "target": None, "raw": text}

        # 1. 识别动作类型
        for action, keywords in self.ACTION_KEYWORDS.items():
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

        # 4. 提取数字
        numbers = re.findall(r'\d+', text)
        if numbers and result["joints"]:
            for i, num in enumerate(numbers):
                if i < len(result["joints"]):
                    result["values"][result["joints"][i]] = int(num)

        # 5. 识别目标
        objects = ["杯子", "杯", "瓶子", "球", "方块", "物体"]
        for obj in objects:
            if obj in text:
                result["target"] = obj
                break

        # 6. 默认推断
        if result["action"] is None:
            if result["joints"]:
                result["action"] = "move"
            elif "零" in text or "初始" in text or "中间" in text:
                result["action"] = "home"

        return result


# 测试
parser = IntentParser()

test_cases = [
    "张开夹爪",
    "回到初始位置",
    "把大臂移动到 2500",
    "抓起杯子",
    "进入自由拖动模式",
    "扫描电机",
]

print("=" * 60)
print("意图解析测试")
print("=" * 60)

all_pass = True
for text in test_cases:
    result = parser.parse(text)
    action = result['action']
    is_none = action is None

    status = "[FAIL]" if is_none else "[OK]"
    if is_none:
        all_pass = False

    print(f"\n{status} 输入: {text}")
    print(f"     动作: {action}")
    print(f"     关节: {result['joints']}")
    print(f"     值: {result['values']}")

print("\n" + "=" * 60)
print(f"结果: {'全部通过' if all_pass else '部分失败'}")
