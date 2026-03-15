"""
机械臂跳舞动作 - 多关节协同舞蹈
"""
import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from SDK import PortHandler, PacketHandler

# STS3215 寄存器
TORQUE_ENABLE = 40
GOAL_POSITION = 42
PRESENT_POSITION = 56

class Dancer:
    """机械臂舞者"""

    def __init__(self, port="COM7", baudrate=1000000):
        self.port = PortHandler(port)
        self.pkt = PacketHandler(0.0)
        self.port.setBaudRate(baudrate)

    def connect(self):
        if not self.port.openPort():
            raise Exception("无法打开串口")
        print("串口已连接")

    def enable_all(self):
        """启用所有关节扭矩"""
        for i in range(1, 7):
            self.pkt.write1ByteTxRx(self.port, i, TORQUE_ENABLE, 1)
        print("所有关节扭矩已启用")

    def disable_all(self):
        """禁用所有关节扭矩"""
        for i in range(1, 7):
            self.pkt.write1ByteTxRx(self.port, i, TORQUE_ENABLE, 0)
        print("所有关节扭矩已释放")

    def move_smooth(self, positions: list, duration: float = 0.8):
        """平滑移动到目标位置"""
        # 读取当前位置
        current = []
        for i in range(1, 7):
            data, result, _ = self.pkt.read2ByteTxRx(self.port, i, PRESENT_POSITION)
            current.append(data if result == 0 else 2047)

        steps = int(duration * 50)  # 50Hz 更新频率
        for step in range(steps + 1):
            t = step / steps
            # 使用正弦插值实现平滑运动
            ease = (1 - (1 + (t - 0.5) * 2) ** 4) / 2 if t < 0.5 else (1 + (1 - (1 - (t - 0.5) * 2) ** 4)) / 2
            for i in range(6):
                pos = int(current[i] + (positions[i] - current[i]) * ease)
                self.pkt.write2ByteTxRx(self.port, i + 1, GOAL_POSITION, pos)
            time.sleep(0.02)

    def wave_arm(self, times: int = 3):
        """挥手动作"""
        print("[挥手]")
        for _ in range(times):
            # 抬起手臂
            self.move_smooth([2047, 1500, 2500, 1800, 2047, 3000], 0.6)
            time.sleep(0.3)
            # 挥动手腕
            self.move_smooth([2047, 1500, 2500, 1800, 2800, 3000], 0.4)
            self.move_smooth([2047, 1500, 2500, 1800, 1300, 3000], 0.4)
        # 回位
        self.move_smooth([2047, 2047, 2047, 2047, 2047, 2047], 0.8)

    def twist_body(self, times: int = 2):
        """扭动身体"""
        print("[扭动]")
        for _ in range(times):
            # 左转
            self.move_smooth([2800, 1800, 2200, 2000, 2047, 2500], 0.5)
            time.sleep(0.2)
            # 右转
            self.move_smooth([1300, 1800, 2200, 2000, 2047, 2500], 0.5)
            time.sleep(0.2)
        # 回位
        self.move_smooth([2047, 2047, 2047, 2047, 2047, 2047], 0.8)

    def bow(self):
        """鞠躬动作"""
        print("[鞠躬]")
        # 前倾
        self.move_smooth([2047, 2800, 2800, 2600, 2047, 2047], 0.8)
        time.sleep(0.5)
        # 恢复
        self.move_smooth([2047, 2047, 2047, 2047, 2047, 2047], 0.8)
        print("[鞠躬完成]")

    def dance(self):
        """完整舞蹈"""
        print("\n>>> 开始跳舞！\n")
        self.wave_arm(3)
        time.sleep(0.5)
        self.twist_body(2)
        time.sleep(0.5)
        self.bow()
        print("\n>>> 舞蹈完成！\n")

    def disconnect(self):
        self.disable_all()
        self.port.closePort()
        print("已断开连接")


def main():
    dancer = Dancer()
    try:
        dancer.connect()
        dancer.enable_all()
        time.sleep(0.5)
        dancer.dance()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        dancer.disconnect()


if __name__ == "__main__":
    main()
