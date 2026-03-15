"""
机械臂挥手打招呼
"""
import sys
import time
sys.path.insert(0, 'D:/project/claw_arm/.claude/skills/arm-control/scripts')

from main import create_controller

ctrl = create_controller()
ctrl.connect()

try:
    print('机械臂打招呼...')

    # 移动到打招呼的姿势
    ctrl.move_smooth({
        'shoulder_pan': 2800,    # 转向右侧
        'shoulder_lift': 2800,   # 抬起大臂
        'elbow_flex': 1200,      # 弯曲小臂
        'wrist_flex': 2047,
        'wrist_roll': 2047,
        'gripper': 3000          # 张开夹爪
    })
    time.sleep(0.5)

    # 挥手动作（手腕旋转）
    for _ in range(3):
        ctrl.set_joint('wrist_roll', 1500)
        time.sleep(0.2)
        ctrl.set_joint('wrist_roll', 2600)
        time.sleep(0.2)

    # 回到校准零位
    ctrl.go_home()
    print('打招呼完成!')

finally:
    ctrl.disconnect()
