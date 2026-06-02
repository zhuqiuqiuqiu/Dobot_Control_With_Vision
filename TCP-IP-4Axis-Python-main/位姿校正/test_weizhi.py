import cv2
import numpy as np
import math


def get_pca_angle_from_image(img_path, debug=False):
    """
    输入图像路径 -> 输出目标轮廓PCA主方向角度（degree）
    """

    # 读取图像
    img = cv2.imread(img_path)
    if img is None:
        raise ValueError("Image not found: " + img_path)

    draw_img = img.copy()

    # 灰度 + 去噪
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # OTSU二值化
    _, thresh = cv2.threshold(
        blur, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # 找轮廓
    contours, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )

    if len(contours) == 0:
        raise ValueError("No contour found")

    # 最大轮廓
    max_contour = max(contours, key=cv2.contourArea)

    # PCA计算方向
    data_pts = max_contour.reshape(-1, 2).astype(np.float32)

    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean=None)

    vx = eigenvectors[0, 0]
    vy = eigenvectors[0, 1]

    angle = math.degrees(math.atan2(vy, vx))

    # 统一角度范围 [-180, 180]
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360

    # debug可视化
    if debug:
        center = (int(mean[0, 0]), int(mean[0, 1]))

        length = 120
        x2 = int(center[0] + length * vx)
        y2 = int(center[1] + length * vy)

        cv2.drawContours(draw_img, [max_contour], -1, (0, 255, 255), 2)
        cv2.circle(draw_img, center, 5, (0, 0, 255), -1)
        cv2.arrowedLine(draw_img, center, (x2, y2), (255, 0, 0), 3, tipLength=0.2)

        cv2.putText(
            draw_img,
            f"Angle: {angle:.2f}",
            (center[0] + 20, center[1] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.imshow("debug", draw_img)
        cv2.imshow("thresh", thresh)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return angle


# ======================
# 使用示例
# ======================
angle = get_pca_angle_from_image("test3.jpg", debug=True)
print("视觉角度 θ_visual =", angle)