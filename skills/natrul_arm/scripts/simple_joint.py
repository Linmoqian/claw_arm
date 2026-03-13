"""
单关节控制示例 - 最简单的关节对象控制
"""
import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from SDK import PortHandler, PacketHandler

class Joint:
    """单个关节控制器"""
    # STS3215 寄存器地址
    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56

    def __init__(self, port_handler, packet_handler, joint_id: int):
        self.port = port_handler
        self.pkt = packet_handler
        self.id = joint_id

    def enable(self):
        """启用扭矩"""
        self.pkt.write1ByteTxRx(self.port, self.id, self.TORQUE_ENABLE, 1)

    def disable(self):
        """禁用扭矩"""
        self.pkt.write1ByteTxRx(self.port, self.id, self.TORQUE_ENABLE, 0)

    def move(self, position: int):
        """移动到指定位置 (0-4095)"""
        position = max(0, min(4095, position))
        self.pkt.write2ByteTxRx(self.port, self.id, self.GOAL_POSITION, position)

    def position(self) -> int:
        """读取当前位置"""
        data, result, _ = self.pkt.read2ByteTxRx(
            self.port, self.id, self.PRESENT_POSITION)
        return data if result == 0 else -1


def main():
    # 连接串口
    port = PortHandler("COM12")
    port.setBaudRate(1000000)
    pkt = PacketHandler(0.0)
    # 创建关节对象 (ID=1, 底部旋转关节)
    joint = Joint(port, pkt, joint_id=2)
    try:
        joint.enable()
        print(f"关节 {joint.id} 扭矩已启用")
        # 读取当前位置
        cur = joint.position()
        print(f"当前位置: {cur}")
        # 移动到新位置
        target = 2047  # 中间位置
        print(f"移动到: {target}")
        joint.move(target)
        time.sleep(1)

        # 读取新位置
        print(f"当前位置: {joint.position()}")
    finally:
        joint.disable()
        port.closePort()
        print("已断开连接")


if __name__ == "__main__":
    main()