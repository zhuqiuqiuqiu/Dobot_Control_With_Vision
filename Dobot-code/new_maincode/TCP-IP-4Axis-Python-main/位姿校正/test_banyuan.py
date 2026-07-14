import cv2
import numpy as np
import math


def get_half_circle_angle(contour, img):

    # ===== 1. 最小外接矩形 =====
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int32(box)

    center = (int(rect[0][0]), int(rect[0][1]))

    w, h = rect[1]
    angle = rect[2]

    # ===== 2. 统一角度方向 =====
    if w < h:
        angle = angle + 90

    # ===== 3. 画轮廓框 =====
    cv2.drawContours(img, [box], 0, (0, 255, 0), 2)

    # ===== 4. 画中心点 =====
    cv2.circle(img, center, 5, (0, 0, 255), -1)

    # ===== 5. 画方向箭头（关键！）=====
    length = 100

    theta = math.radians(angle)

    x2 = int(center[0] + length * math.cos(theta))
    y2 = int(center[1] + length * math.sin(theta))

    cv2.arrowedLine(
        img,
        center,
        (x2, y2),
        (255, 0, 0),
        3,
        tipLength=0.2
    )

    # ===== 6. 显示角度 =====
    cv2.putText(
        img,
        f"angle: {angle:.2f}",
        (center[0] + 20, center[1] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    return angle


# =========================
# 主程序（一定要有）
# =========================

img = cv2.imread("test2.png")

if img is None:
    print("图片读取失败！")
    exit()

draw = img.copy()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

_, thresh = cv2.threshold(
    gray, 0, 255,
    cv2.THRESH_BINARY + cv2.THRESH_OTSU
)

contours, _ = cv2.findContours(
    thresh,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_NONE
)

cnt = max(contours, key=cv2.contourArea)

angle = get_half_circle_angle(cnt, draw)

print("最终角度 =", angle)

cv2.imshow("Result", draw)
cv2.imshow("Binary", thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()