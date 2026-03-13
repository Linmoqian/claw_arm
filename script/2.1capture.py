"""
相机拍照 - 空格拍照，q退出
"""
import cv2
import os

SAVE_DIR = os.path.join(os.path.dirname(__file__), "..", "photo")
os.makedirs(SAVE_DIR, exist_ok=True)

def main():
    cap = cv2.VideoCapture(1)
    count = 1

    # 查找已有文件的最大编号
    for f in os.listdir(SAVE_DIR):
        if f.endswith('.jpg') and f[:-4].isdigit():
            count = max(count, int(f[:-4]) + 1)

    print(f"起始编号: {count}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("Camera", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):  # 空格拍照
            path = os.path.join(SAVE_DIR, f"{count}.jpg")
            cv2.imwrite(path, frame)
            print(f"已保存: {path}")
            count += 1
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
