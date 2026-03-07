#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试自然语言控制技能的代码生成器
无需硬件，仅验证代码生成的正确性
"""

import sys
import os
from pathlib import Path

# 添加项目根目录到路径
PROJECT_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT_ROOT))

# 直接导入（解决模块名带连字符的问题）
import importlib.util
spec = importlib.util.spec_from_file_location(
    "executor",
    PROJECT_ROOT / "skills/nl-arm-control/scripts/executor.py"
)
executor_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(executor_module)

IntentParser = executor_module.IntentParser
CodeGenerator = executor_module.CodeGenerator


def test_intent_parser():
    """测试意图解析器"""
    print("=" * 60)
    print("测试意图解析器")
    print("=" * 60)

    parser = IntentParser()

    test_cases = [
        ("张开夹爪", {"action": "gripper_open", "joints": [], "values": {"gripper": 3000}}),
        ("闭合夹爪", {"action": "gripper_close", "joints": [], "values": {"gripper": 1000}}),
        ("回到初始位置", {"action": "home"}),
        ("把大臂移动到 2500", {"action": "move", "joints": ["shoulder_lift"], "values": {"shoulder_lift": 2500}}),
        ("抓起杯子", {"action": "pick", "target": "杯子"}),
        ("进入自由拖动模式", {"action": "freedrag"}),
    ]

    passed = 0
    for input_text, expected_partial in test_cases:
        result = parser.parse(input_text)
        print(f"\n输入: {input_text}")
        print(f"  解析结果: action={result['action']}, joints={result['joints']}, values={result['values']}, target={result.get('target')}")

        # 简单验证
        success = True
        for key, expected_value in expected_partial.items():
            if result.get(key) != expected_value:
                print(f"  ✗ 预期 {key}={expected_value}, 实际={result.get(key)}")
                success = False

        if success:
            print(f"  ✓ 通过")
            passed += 1

    print(f"\n解析器测试: {passed}/{len(test_cases)} 通过")
    return passed == len(test_cases)


def test_code_generator():
    """测试代码生成器"""
    print("\n" + "=" * 60)
    print("测试代码生成器")
    print("=" * 60)

    generator = CodeGenerator()

    test_cases = [
        ("张开夹爪", "gripper_open", [], {}),
        ("回到初始位置", "home", [], {}),
        ("移动大臂", "move", ["shoulder_lift"], {"shoulder_lift": 2500}),
        ("抓取", "pick", [], {"target": "物体"}),
    ]

    passed = 0
    for desc, action, joints, values in test_cases:
        intent = {"action": action, "joints": joints, "values": values}
        code, description = generator.generate(intent)

        print(f"\n生成: {desc}")
        print(f"  描述: {description}")

        # 验证关键元素
        checks = [
            ("导入模块", "import" in code or "from" in code),
            ("创建控制器", "create_controller" in code),
            ("连接", "connect()" in code),
            ("断开连接", "disconnect()" in code),
        ]

        all_pass = True
        for check_name, check_result in checks:
            if check_result:
                print(f"  ✓ {check_name}")
            else:
                print(f"  ✗ {check_name}")
                all_pass = False

        if all_pass:
            passed += 1

    print(f"\n代码生成器测试: {passed}/{len(test_cases)} 通过")
    return passed == len(test_cases)


def test_end_to_end():
    """端到端测试（代码生成，不执行）"""
    print("\n" + "=" * 60)
    print("端到端测试")
    print("=" * 60)

    parser = IntentParser()
    generator = CodeGenerator()

    test_inputs = [
        "张开夹爪",
        "回到初始位置",
        "把大臂移动到 2500",
        "抓起杯子",
        "进入自由拖动模式",
    ]

    passed = 0
    for user_input in test_inputs:
        print(f"\n用户输入: {user_input}")

        intent = parser.parse(user_input)
        code, description = generator.generate(intent)

        print(f"  解析动作: {intent.get('action')}")
        print(f"  生成描述: {description}")

        # 基本验证
        if code and description:
            print(f"  ✓ 代码生成成功")
            passed += 1
        else:
            print(f"  ✗ 代码生成失败")

    print(f"\n端到端测试: {passed}/{len(test_inputs)} 通过")
    return passed == len(test_inputs)


def main():
    """运行所有测试"""
    print("自然语言控制机械臂技能 - 代码生成测试\n")

    results = []

    results.append(("意图解析器", test_intent_parser()))
    results.append(("代码生成器", test_code_generator()))
    results.append(("端到端测试", test_end_to_end()))

    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)
    for name, passed in results:
        status = "✓ 通过" if passed else "✗ 失败"
        print(f"  {name}: {status}")

    all_passed = all(r[1] for r in results)
    print("\n" + ("全部测试通过！" if all_passed else "部分测试失败"))
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
