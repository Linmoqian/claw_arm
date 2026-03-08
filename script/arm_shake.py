"""
机械臂摇头动作
"""
import sys
import time
sys.path.insert(0, 'D:/project/claw_arm/.claude/skills/arm-control/scripts')

from main import create_controller

ctrl = create_controller(port='COM7')
ctrl.connect()

try:
    print('开始摇头动作...')

    # 先回到中位
    ctrl.set_joint('shoulder_pan', 2047)
    time.sleep(0.5)

    # 摇头动作：左→右→左→右→回中位
    positions = [1500, 2600, 1500, 2600, 2047]
    for pos in positions:
        ctrl.set_joint('shoulder_pan', pos)
        time.sleep(0.4)

    print('摇头完成!')

finally:
    ctrl.disconnect()
