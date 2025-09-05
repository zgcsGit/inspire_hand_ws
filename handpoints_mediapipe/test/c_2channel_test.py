import cv2

# 左边摄像头（本地）和右边摄像头（IP摄像头）
cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture("http://192.168.0.101:8080/video")

cv2.namedWindow("双目摄像头", cv2.WINDOW_NORMAL)

while True:
    ret_l, frame_l = cap_left.read()
    ret_r, frame_r = cap_right.read()

    if not ret_l or not ret_r:
        print("某个摄像头读取失败")
        continue

    # 统一尺寸
    frame_l = cv2.resize(frame_l, (640, 480))
    frame_r = cv2.resize(frame_r, (640, 480))

    # 合并左右图像水平拼接
    combined = cv2.hconcat([frame_l, frame_r])

    # 显示画面
    cv2.imshow("双目摄像头", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
