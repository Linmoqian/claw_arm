"""
拍照并记录电机位置 - 空格拍照，q退出
"""
import cv2
import os
import sys
import json
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from SDK import PortHandler, PacketHandler

PHOTO_DIR = os.path.join(os.path.dirname(__file__), "..", "photo-base")
os.makedirs(PHOTO_DIR, exist_ok=True)


class Motors:
    """电机读取器"""

    def __init__(self, port="COM12", baudrate=1000000):
        self.port = PortHandler(port)
        self.pkt = PacketHandler(0.0)
        self.connected = False

    def connect(self):
        if self.port.openPort() and self.port.setBaudRate(1000000):
            self.connected = True
            return True
        return False

    def get_positions(self) -> dict:
        """读取6个电机位置"""
        positions = {}
        for i in range(1, 7):
            data, result, _ = self.pkt.read2ByteTxRx(self.port, i, 56)
            positions[i] = data if result == 0 else -1
        return positions

    def disconnect(self):
        if self.connected:
            self.port.closePort()


def main():
    motors = Motors()
    if not motors.connect():
        print("警告: 无法连接电机，位置将记录为 -1")

    cap = cv2.VideoCapture(1)

    # 起始编号
    count = 1
    for f in os.listdir(PHOTO_DIR):
        if f.endswith('.jpg') and f[:-4].isdigit():
            count = max(count, int(f[:-4]) + 1)

    print(f"起始编号: {count}")
    print("空格: 拍照  q: 退出\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imshow("Camera", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                # 读取电机位置
                positions = motors.get_positions()

                # 保存图片
                img_path = os.path.join(PHOTO_DIR, f"{count}.jpg")
                cv2.imwrite(img_path, frame)

                # 保存数据
                data_path = os.path.join(PHOTO_DIR, f"{count}.json")
                data = {
                    "positions": positions,
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
                }
                with open(data_path, 'w') as f:
                    json.dump(data, f, indent=2)

                print(f"[{count}] 电机: {list(positions.values())}")
                count += 1

            elif key == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        motors.disconnect()


if __name__ == "__main__":
    main()
