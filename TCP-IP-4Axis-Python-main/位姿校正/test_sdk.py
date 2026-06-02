# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
from ultralytics import YOLO


MODEL_PATH = r"E:\Dobot-code\new_maincode\TCP-IP-4Axis-Python-main\best.pt"
IMAGE_PATH = "1.jpg"


def draw_axis(img, p, q, color):

    p = (int(p[0]), int(p[1]))
    q = (int(q[0]), int(q[1]))

    cv2.line(
        img,
        p,
        q,
        color,
        2,
        cv2.LINE_AA
    )

    angle = math.atan2(
        p[1] - q[1],
        p[0] - q[0]
    )

    hook_len = 15

    p1 = (
        int(q[0] + hook_len * math.cos(angle + math.pi / 4)),
        int(q[1] + hook_len * math.sin(angle + math.pi / 4))
    )

    p2 = (
        int(q[0] + hook_len * math.cos(angle - math.pi / 4)),
        int(q[1] + hook_len * math.sin(angle - math.pi / 4))
    )

    cv2.line(img, q, p1, color, 2)
    cv2.line(img, q, p2, color, 2)


def pca_angle(roi):

    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    blur = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    _, thresh = cv2.threshold(
        blur,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    contours, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )

    if len(contours) == 0:
        raise Exception("No contour")

    contour = max(
        contours,
        key=cv2.contourArea
    )

    data_pts = contour.reshape(
        -1,
        2
    ).astype(np.float32)

    mean, eigenvectors, eigenvalues = cv2.PCACompute2(
        data_pts,
        mean=None
    )

    cx = int(mean[0, 0])
    cy = int(mean[0, 1])

    vx = eigenvectors[0, 0]
    vy = eigenvectors[0, 1]

    angle = math.degrees(
        math.atan2(vy, vx)
    )

    return (
        angle,
        contour,
        (cx, cy),
        (vx, vy),
        thresh
    )


# ====================================
# 加载模型
# ====================================

model = YOLO(MODEL_PATH)

img = cv2.imread(IMAGE_PATH)

if img is None:
    raise Exception("图片读取失败")

# ====================================
# YOLO检测
# ====================================

results = model(
    img,
    conf=0.4,
    verbose=False
)

boxes = results[0].boxes

if len(boxes) == 0:
    raise Exception("未检测到目标")

best_box = None
best_conf = 0

for box in boxes:

    conf = float(box.conf[0])

    if conf > best_conf:

        best_conf = conf
        best_box = box

x1, y1, x2, y2 = best_box.xyxy[0].tolist()

x1 = int(x1)
y1 = int(y1)
x2 = int(x2)
y2 = int(y2)

cls_id = int(best_box.cls[0])

name = model.names[cls_id]

print("目标:", name)
print("置信度:", best_conf)

# ====================================
# ROI
# ====================================

roi = img[y1:y2, x1:x2].copy()

angle, contour, center, vec, thresh = pca_angle(roi)

print("PCA角度 =", angle)

# ====================================
# 画轮廓
# ====================================

debug_roi = roi.copy()

cv2.drawContours(
    debug_roi,
    [contour],
    -1,
    (0, 255, 0),
    2
)

cx, cy = center

vx, vy = vec

length = 150

end_x = cx + length * vx
end_y = cy + length * vy

draw_axis(
    debug_roi,
    (cx, cy),
    (end_x, end_y),
    (0, 0, 255)
)

cv2.circle(
    debug_roi,
    (cx, cy),
    5,
    (255, 0, 0),
    -1
)

cv2.putText(
    debug_roi,
    f"PCA={angle:.2f}",
    (20, 40),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (0, 0, 255),
    2
)

# ====================================
# 原图显示检测框
# ====================================

show_img = img.copy()

cv2.rectangle(
    show_img,
    (x1, y1),
    (x2, y2),
    (0, 255, 0),
    2
)

cv2.putText(
    show_img,
    name,
    (x1, y1 - 10),
    cv2.FONT_HERSHEY_SIMPLEX,
    1,
    (0, 255, 0),
    2
)

# ====================================
# 显示
# ====================================

cv2.imshow("Original", show_img)
cv2.imshow("ROI", roi)
cv2.imshow("Threshold", thresh)
cv2.imshow("PCA Result", debug_roi)

cv2.waitKey(0)
cv2.destroyAllWindows()