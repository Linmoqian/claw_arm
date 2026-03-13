import cv2

cap = cv2.VideoCapture(0)# 0 表示默认摄像头，若连接机械臂的摄像头则可能为1。

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取画面")
        break

    cv2.imshow("Camera", frame)

    # 按 q 键退出
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

# 保存一帧画面：
# cv2.imwrite("frame.jpg", frame)
