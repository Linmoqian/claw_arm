"""
机械臂点头动作
"""
import sys
import time
sys.path.insert(0, 'D:/project/claw_arm/.claude/skills/arm-control/scripts')

from main import create_controller

ctrl = create_controller(port='COM7')
ctrl.connect()

try:
    print('开始点头动作...')

    # 先回到中位
    ctrl.set_joint('wrist_flex', 2047)
    time.sleep(0.5)

    # 点头动作：低头→抬起→低头→抬起→回中位
    positions = [1500, 2600, 1500, 2600, 2047]  # 低头、抬起、低头、抬起、中位
    for pos in positions:
        ctrl.set_joint('wrist_flex', pos)
        time.sleep(0.4)

    print('点头完成!')

finally:
    ctrl.disconnect()
