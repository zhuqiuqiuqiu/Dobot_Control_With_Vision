# -*- coding: utf-8 -*-

"""
MG400 TCP/IP GUI demo with manual movement and optional YOLO detection.

Run:
    python 5_28版本.py

The robot side uses dobot_api.py in this repository. Vision features need:
    pip install opencv-python pillow ultralytics
"""

from ctypes import *
import math
import numpy as np
from Python.MvImport.MvCameraControl_class import *
from Python.MvImport.CameraParams_header import *
from Python.MvImport.MvErrorDefine_const import *
import threading
import time
import tkinter as tk
import xml.etree.ElementTree as ET
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText
try:
    import cv2
except ImportError:
    cv2 = None
try:
    from PIL import Image, ImageTk
except ImportError:
    Image = None
    ImageTk = None
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None
from dobot_api import DobotApiDashboard, DobotApiMove


DEFAULT_IP = "192.168.1.6"
## TCP_IP远程控制的文档，指令下发必须是29999端口
DEFAULT_DASHBOARD_PORT = 29999
DEFAULT_MOVE_PORT = 30003
DEFAULT_MODEL_PATH = "best.pt"
DEFAULT_CALIB_XML = "\u6807\u5b9a\u8f6c\u6362.xml"


'''
根据TCP_IP远程控制接口文档
29999服务器端口：上位机可以通过29999端口直接发送一些设置相关指令给机器人，或者主动获取机器人的某些状态，这些功能被称为Dashboard。
30003服务器端口：上位机可以通过30003端口直接发送一些机器人运动相关指令给机器人，控制机器人进行运动。
30004、30005以及30006服务器端口：30004端口即实时反馈端口，客户端每8ms能收到一次机器人实时状态信息。30005端口每200ms反馈机器人的信息。30006端口为可配置的反
馈机器人信息端口(默认为每50ms反馈，如需修改，请联系技术支持)。通过实时反馈端口每次收到的数据包有1440个字节，这些字节以标准的格式排列。
'''


class Setting:
    '''
    这里是我的一些参数设计
    pick_high:     物块的z轴的高度，需要在施教模式下找出
    safe_high：    需要机械臂的安全高度
    pause_pick:    吸嘴 r轴 角度
    count : 视觉抓取的次数
    需要标定转换的文件：这里需要进行标定转换
    do1：   吸气的 数字开关  1开  0关
    do2：   呼气的 数字开关

    '''
    # 需要修改的参数
    safe_high = -50
    pick_high = -155.05
    pause_pick = 0  # 0°
    pick_hold = 0.6

    Count = 6

    ## 位姿校正的参数
    angle_offset = 136.75
    K = 1  #视觉与物理的方向是否相同
    angle_target = -103  # 0° 表示手机壳漕的度数


    ##

    # 放置坐标. 抓取后的放置坐标
    place_x = 368.02
    place_y = -93.49
    place_z = -139.04
    place_r = None
    # Place pose by YOLO class name. Change these names and coordinates to
    # match the class names in your best.pt model. If a class is not listed,
    # the code falls back to place_x/place_y/place_z above. Place R follows
    # the corrected pick R so the part orientation does not change.
    place_targets = {
        "WIFI": {"x": 306.8, "y": -108.62, "z": -144.49},
        "Dianzi": {"x": 358.08, "y": -119.99, "z": -144.71},
        "GPS": {"x": 348.08, "y": -119.99, "z": -144.71},
        "ZhonglI": {"x": 306.8, "y": -120.62, "z": -144.49},
        "Guangmin": {"x": 338.08, "y": -112.99, "z": -144.71},
        "Wubian": {"x": 328.08, "y": -114.99, "z": -144.71},

    }

    # 固定参数
    do1 = 1
    do2 = 2
    do1_yes = 1
    do1_no = 0
    do2_yes = 1
    do2_no = 0

    @classmethod
    def get_place_pose(cls, target_name=None):
        pose = cls.place_targets.get(target_name, {})
        return {
            "x": pose.get("x", cls.place_x),
            "y": pose.get("y", cls.place_y),
            "z": pose.get("z", cls.place_z),
            "r": pose.get("r", cls.place_r),
        }


def get_pca_angle_from_frame(img, bbox=None, debug=False):
    """
    输入：OpenCV frame
    输出：PCA主方向角（degree）
    这里是输出视觉角度的
    视觉角度
    """
    if bbox is not None:
        height, width = img.shape[:2]
        x1, y1, x2, y2 = bbox
        x1 = max(0, min(width - 1, int(round(x1))))
        y1 = max(0, min(height - 1, int(round(y1))))
        x2 = max(0, min(width, int(round(x2))))
        y2 = max(0, min(height, int(round(y2))))
        if x2 <= x1 or y2 <= y1:
            raise ValueError("Invalid bbox for PCA")
        img = img[y1:y2, x1:x2]
        if img.size == 0:
            raise ValueError("Empty bbox image for PCA")

    draw_img = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(
        blur, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    contours, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )
    if len(contours) == 0:
        raise ValueError("No contour found")
    max_contour = max(contours, key=cv2.contourArea)
    data_pts = max_contour.reshape(-1, 2).astype(np.float32)
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean=None)
    vx = eigenvectors[0, 0]
    vy = eigenvectors[0, 1]
    angle = math.degrees(math.atan2(vy, vx))
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360
    angle_vision = angle   #视觉角度
    ##物理角度的差   定义缺口朝向  我的右边为 物理的 θ_0=0°
    ## 在这个0°下找到视觉的度数 θ_1
    ## 因此，得到 θ_offset  =  θ_0 - θ_1
    ## 定义放置的目标槽角
    ##  顺时针为正度数
    angle_robot =  Setting.K *angle_vision + Setting.angle_offset
    # 计算补偿角度   末端绝对姿态
    angle_end= Setting.angle_target - angle_robot
    return angle_end


def normalize_angle_deg(angle):
    """Wrap an angle to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0

class Calibration:
    def __init__(self, matrix):
        self.matrix = matrix
    @classmethod
    def load(cls, xml_path):
        tree = ET.parse(xml_path)
        values = [
            float(node.text)
            for node in tree.findall(".//CalibFloatListParam[@ParamName='CalibMatrix']/ParamValue")
        ]
        if len(values) != 9:
            raise ValueError(f"Expected 9 calibration values, got {len(values)}")
        return cls(np.array(values, dtype=float).reshape(3, 3))

    def image_to_robot(self, x_px, y_px):
        vec = np.array([float(x_px), float(y_px), 1.0], dtype=float)
        out = self.matrix @ vec
        w = float(out[2])
        if abs(w) < 1e-9:
            w = 1.0
        return float(out[0] / w), float(out[1] / w)

class HikCamera:

    def __init__(self):

        self.cam = MvCamera()

        self.deviceList = MV_CC_DEVICE_INFO_LIST()

    # 转16进制错误码
    def to_hex_str(self, num):

        chaDic = {
            10: 'a',
            11: 'b',
            12: 'c',
            13: 'd',
            14: 'e',
            15: 'f'
        }

        hexStr = ""

        if num < 0:
            num = num + 2 ** 32

        while num >= 16:
            digit = num % 16
            hexStr = chaDic.get(digit, str(digit)) + hexStr
            num //= 16

        hexStr = chaDic.get(num, str(num)) + hexStr

        return hexStr

    # 打开相机
    def open_camera(self):

        # 初始化SDK
        MvCamera.MV_CC_Initialize()

        # 枚举设备
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

        ret = MvCamera.MV_CC_EnumDevices(
            tlayerType,
            self.deviceList
        )

        if ret != 0:
            print("Enum Devices Failed! ret =", self.to_hex_str(ret))
            return False

        if self.deviceList.nDeviceNum == 0:
            print("No Camera Found!")
            return False

        print("Find %d devices!" % self.deviceList.nDeviceNum)

        # 获取第一台设备
        stDeviceList = cast(
            self.deviceList.pDeviceInfo[0],
            POINTER(MV_CC_DEVICE_INFO)
        ).contents

        # 创建句柄
        ret = self.cam.MV_CC_CreateHandle(stDeviceList)

        if ret != 0:
            print("Create Handle Failed! ret =", self.to_hex_str(ret))
            return False

        # 打开设备
        ret = self.cam.MV_CC_OpenDevice(
            MV_ACCESS_Exclusive,
            0
        )

        if ret != 0:
            print("Open Device Failed! ret =", self.to_hex_str(ret))
            return False

        print("Camera Open Success")

        # 设置连续采集模式
        ret = self.cam.MV_CC_SetEnumValue(
            "AcquisitionMode",
            2
        )

        if ret != 0:
            print("Set AcquisitionMode Failed! ret =", self.to_hex_str(ret))

        # 关闭触发模式
        ret = self.cam.MV_CC_SetEnumValue(
            "TriggerMode",
            0
        )

        if ret != 0:
            print("Set TriggerMode Failed! ret =", self.to_hex_str(ret))

        # 开始取流
        ret = self.cam.MV_CC_StartGrabbing()

        if ret != 0:
            print("Start Grabbing Failed! ret =", self.to_hex_str(ret))
            return False

        print("Start Grabbing Success")

        # 等待相机稳定
        time.sleep(1)

        return True

    # 拍照
    def take_photo(self):

        stOutFrame = MV_FRAME_OUT()

        memset(byref(stOutFrame), 0, sizeof(stOutFrame))

        # 获取一帧图像
        ret = self.cam.MV_CC_GetImageBuffer(
            stOutFrame,
            3000
        )

        if ret != 0:
            print(
                "Get Image Failed! ret =",
                self.to_hex_str(ret)
            )
            return None

        try:

            width = stOutFrame.stFrameInfo.nWidth
            height = stOutFrame.stFrameInfo.nHeight
            frame_len = stOutFrame.stFrameInfo.nFrameLen

            print("Width =", width)
            print("Height =", height)
            print("FrameLen =", frame_len)

            # 获取图像数据
            pData = cast(
                stOutFrame.pBufAddr,
                POINTER(c_ubyte * frame_len)
            ).contents

            image = np.frombuffer(
                pData,
                dtype=np.uint8
            )

            pixel_type = stOutFrame.stFrameInfo.enPixelType

            print("PixelType =", pixel_type)

            # Mono8灰度图
            if pixel_type == PixelType_Gvsp_Mono8:

                frame = image.reshape(height, width)

            else:

                # 彩色图像
                frame = image.reshape(height, width, 3)

                # RGB转BGR
                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_RGB2BGR
                )

            print("Photo Captured")

            return frame

        finally:

            # 释放buffer
            self.cam.MV_CC_FreeImageBuffer(stOutFrame)

    # 关闭相机
    def close_camera(self):

        self.cam.MV_CC_StopGrabbing()

        self.cam.MV_CC_CloseDevice()

        self.cam.MV_CC_DestroyHandle()

        MvCamera.MV_CC_Finalize()

        print("Camera Closed")


class DobotClient:
    """Small wrapper around dashboard and move ports."""

    def __init__(self, logger):
        self.dashboard = None
        self.move = None
        self.logger = logger

    @property
    def connected(self):
        return self.dashboard is not None and self.move is not None

    def connect(self, ip, dashboard_port, move_port):
        self.disconnect()
        self.logger(f"Connecting to {ip} ...")
        self.dashboard = DobotApiDashboard(ip, dashboard_port)
        self.move = DobotApiMove(ip, move_port)
        self.logger("Robot connected.")

    def disconnect(self):
        for client in (self.dashboard, self.move):
            if client is not None:
                try:
                    client.close()
                except Exception as exc:
                    self.logger(f"Close failed: {exc}")
        self.dashboard = None
        self.move = None

    def require_connected(self):
        if not self.connected:
            raise RuntimeError("Please connect the robot first.")

    def enable(self):
        self.require_connected()
        return self.dashboard.EnableRobot()

    def disable(self):
        self.require_connected()
        return self.dashboard.DisableRobot()

    def clear_error(self):
        '''清除机械臂报警'''  # 525修改: 错别字 清楚 → 清除
        self.require_connected()
        return self.dashboard.ClearError()

    def set_speed(self, speed):
        '''
        设置机械臂速度
        '''
        self.require_connected()
        return self.dashboard.SpeedFactor(speed)

    def get_pose(self):
        '''获取当前的位姿'''
        self.require_connected()
        return self.dashboard.GetPose()

    def rel_move(self, dx=0.0, dy=0.0, dz=0.0, dr=0.0, straight=False):
        '''
        机械臂的移动模式，关节模式？直线模式？
        '''
        self.require_connected()
        if straight:
            return self.move.RelMovL(dx, dy, dz, dr)
        return self.move.RelMovJ(dx, dy, dz, dr)

    def stop_jog(self):
        '''
        停止手动控制的代码
        '''
        if self.connected:
            return self.move.MoveJog()
        return None

    def go_zero(self):
     ###  回原点的函数，使用关节模式movJ 移动到安全姿态 J1=0°,J2=0°,J3=0°,J4=0°  # 525修改: 注释与实际参数不符，原写的"回到0°"会误导
        self.require_connected()
        return self.move.JointMovJ(0, 0, 0, 0)

    def wait_move_done(self):
        try:
            return self.move.Sync()
        except Exception as exc:
            self.logger(f"Sync failed: {exc}")
            return None

    def vision_pick(self, x, y, r=None, r_delta=0.0, target_name=None):
        self.require_connected()
        r = Setting.pause_pick if r is None else r
        self.logger(f"Vision pick target: name={target_name}, X={x:.3f}, Y={y:.3f}, R={r:.3f}")
        self.move.MovJ(x, y, Setting.safe_high, r)
        self.wait_move_done()

        self.move.MovL(x, y, Setting.pick_high, r)
        self.wait_move_done()
        time.sleep(0.1)
        self.dashboard.DO(Setting.do1, Setting.do1_yes)
        time.sleep(Setting.pick_hold)
        self.move.MovL(x, y, Setting.safe_high, r)
        self.wait_move_done()
        time.sleep(0.1)

        if abs(r_delta) > 1e-6:
            self.logger(f"Post-pick relative R correction: dR={r_delta:.3f}")
            self.rel_move(dr=r_delta, straight=False)
            self.wait_move_done()
            try:
                r = self.get_current_r()
                self.logger(f"R after post-pick correction = {r:.3f}")
            except Exception as exc:
                r = normalize_angle_deg(r + r_delta)
                self.logger(f"Read R failed after correction: {exc}; fallback R={r:.3f}")
        #



        # 回到安全位置
        #  这里需要修改为 根据 不同的识别目标 放到不同的安全位置。  改为条件判断函数
        place_pose = Setting.get_place_pose(target_name)
        place_x = place_pose["x"]
        place_y = place_pose["y"]
        if place_x is not None and place_y is not None:
            place_z = place_pose["z"] if place_pose["z"] is not None else Setting.safe_high
            place_r = r
            self.logger(
                f"Move to place: name={target_name}, X={place_x:.3f}, "
                f"Y={place_y:.3f}, Z={place_z:.3f}, R={place_r:.3f}"
            )
            self.move.MovJ(place_x, place_y, Setting.safe_high, place_r)
            self.wait_move_done()
            self.move.MovL(place_x, place_y, place_z, place_r)
            self.wait_move_done()
            self.dashboard.DO(Setting.do1, Setting.do1_no)
            time.sleep(0.1)
            self.move.MovL(place_x, place_y, Setting.safe_high, place_r)
            self.wait_move_done()
        return "Vision pick done"

    def get_current_r(self):

        pose = self.dashboard.GetPose()

        start = pose.find("{")
        end = pose.find("}")

        data = pose[start + 1:end]

        nums = data.split(",")

        # MG400
        r = float(nums[3])

        return r






class VisionPanel:
    def __init__(self, parent, client, log_callback):  # 525修改: 补上 log_callback 参数，原来只传 client 导致 self.log 缺失
        self.parent = parent
        self.log = log_callback  # 525修改: 原来这行被注释掉，导致 self.log(...) 全部报 AttributeError
        self.model = None
        self.client = client
        self.calibration = None
        self.last_frame = None
        self.photo = None

        self.frame = ttk.LabelFrame(parent, text="视觉控制", style="Custom.TLabelframe")
        self.frame.columnconfigure(0, weight=3)

        self.model_path = tk.StringVar(value=DEFAULT_MODEL_PATH)
        ttk.Label(self.frame, text="模型").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 2))
        ttk.Entry(self.frame, textvariable=self.model_path, width=24).grid(
            row=1, column=0, sticky="ew", padx=8
        )
        ttk.Button(self.frame, text="加载视觉模型", command=self.load_model).grid(
            row=1, column=1, sticky="ew", padx=(0, 8)
        )
        self.calib_path = tk.StringVar(value=DEFAULT_CALIB_XML)
        self.auto_pick = tk.BooleanVar(value=False)
        ttk.Label(self.frame, text="标定文件").grid(row=2, column=0, sticky="w", padx=8, pady=(8, 2))
        ttk.Entry(self.frame, textvariable=self.calib_path, width=24).grid(
            row=3, column=0, sticky="ew", padx=8
        )
        ttk.Button(self.frame, text="加载标定", command=self.load_calibration).grid(
            row=3, column=1, sticky="ew", padx=(0, 8)
        )

        self.canvas_width = 360
        self.canvas_height = 260
        self.canvas = tk.Canvas(
            self.frame,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="#202020",
            highlightthickness=1,
            highlightbackground="#999999",
        )
        self.canvas.grid(row=4, column=0, columnspan=2, padx=8, pady=8)
        self.canvas.create_text(
            self.canvas_width // 2,
            self.canvas_height // 2,
            fill="#dddddd",
            text="Camera preview",
        )
        ###==================================================
        ###  初始的监控界面的图片
        # 525修改: 加 Image/文件存在性判断，避免 PIL 没装或 page.png 缺失时直接崩
        if Image is not None and ImageTk is not None:
            try:
                self.img = Image.open('page.png')
                img = self._resize_to_fit(self.img)
                self.photo_image = ImageTk.PhotoImage(img)
                self.canvas.delete("all")  # 清空旧图
                self.canvas.create_image(
                    self.canvas_width // 2,
                    self.canvas_height // 2,
                    image=self.photo_image,
                    anchor="center"
                )
                self.current_image = self.photo_image
            except Exception as exc:  # 525修改: 容错处理
                print(f"加载初始图片 page.png 失败: {exc}")
        ###===================================================


        ttk.Button(self.frame, text="拍照", command=self.take_photo_async).grid(
            row=5, column=0, sticky="ew", padx=8, pady=(0, 8)
        )
        ttk.Button(self.frame, text="Detect Once", command=self.detect_once_async).grid(
            row=5, column=1, sticky="ew", padx=(0, 8), pady=(0, 8)
        )
        ttk.Checkbutton(self.frame, text="Auto pick after detection", variable=self.auto_pick).grid(
            row=6, column=0, columnspan=2, sticky="w", padx=8, pady=(0, 8)
        )
        self.load_calibration()
    def _resize_to_fit(self, img):
        """把图片等比例缩放到画布内，不变形"""
        img_w, img_h = img.size
        scale = min(self.canvas_width / img_w, self.canvas_height / img_h)

        if scale < 1:  # 只有图片比画布大才缩小，小图不放大（避免模糊）
            new_w = int(img_w * scale)
            new_h = int(img_h * scale)
            img = img.resize((new_w, new_h), Image.Resampling.LANCZOS)
        return img
    def load_model(self):
        if YOLO is None:
            messagebox.showerror("Missing dependency", "ultralytics is not installed.")
            return
        try:
            self.model = YOLO(self.model_path.get().strip())
            self.model.to("cpu")
            self.log(f"YOLO model loaded: {self.model_path.get().strip()}")
        except Exception as exc:
            messagebox.showerror("YOLO error", str(exc))
            self.log(f"YOLO load failed: {exc}")

    def load_calibration(self):
        path = self.calib_path.get().strip()
        try:
            self.calibration = Calibration.load(path)
            self.log(f"Calibration loaded: {path}")
        except Exception as exc:
            self.calibration = None
            self.log(f"Calibration load failed: {exc}")

    def take_photo_async(self):
        threading.Thread(target=self.take_photo, daemon=True).start()

    def detect_once_async(self):
        threading.Thread(target=self.detect_once, daemon=True).start()

    def take_photo(self):
        cam = HikCamera()
        # 打开相机
        if not cam.open_camera():
            self.log("相机打开失败")
            return None
        # 拍照
        frame = cam.take_photo()
        if frame is not None:
            self.last_frame = frame
            # 更新到 tkinter 画布
            self.parent.after(
                0,
                lambda: self.show_frame(frame)
            )
            self.log("拍照成功")
            # 如果你还想额外弹opencv窗口
            # cv2.imshow("Photo", frame)
            # cv2.waitKey(1)
        else:
            self.log("拍照失败")
        # 关闭相机
        cam.close_camera()


        return frame



    def detect_once(self):

        success_count = 0
        while success_count < Setting.Count:
            if self.model is None:
                self.parent.after(0, self.load_model)
                time.sleep(0.5)
                if self.model is None:
                    return
            frame = self.take_photo()
            if frame is None:
                return
            try:
                results = self.model(frame, conf=0.4, verbose=False)
                annotated = results[0].plot()
                boxes = results[0].boxes
                if len(boxes) == 0:
                    self.log("未检测到目标")
                    continue
                best_pick = None
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    name = self.model.names.get(cls_id, str(cls_id))
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    self.log(f"recevied: {name},{conf:.2f}, {cx:.1f}, {cy:.1f}")
                    if best_pick is None or conf > best_pick["conf"]:
                        best_pick = {
                            "name": name,
                            "conf": conf,
                            "center": (cx, cy),
                            "bbox": (x1, y1, x2, y2),
                        }
##### 新加入 5/28 加入位姿补偿
                angle = 0
                try:
                    angle = get_pca_angle_from_frame(frame, bbox=best_pick["bbox"])
                    self.log(f"PCA angle = {angle:.2f}")
                except Exception as e:
                    self.log(f"PCA失败: {e}")
#####
                if self.auto_pick.get() and best_pick is not None:
##### 修改这里
                    # self._pick_from_detection(best_pick)
                    self._pick_from_detection(best_pick, angle)

                    success_count += 1
                    self.log(
                        f"已完成抓取 {success_count}/{Setting.Count}"
                    )

                self.parent.after(0, lambda: self.show_frame(annotated))
            except Exception as exc:
                self.log(f"视觉检测错误: {exc}")
            # time.sleep(0.1)
        self.log('recevied:End')

    def _pick_from_detection(self, detection, angle=0):
        if self.calibration is None:
            raise RuntimeError("Calibration is not loaded")
        cx, cy = detection["center"]
        robot_x, robot_y = self.calibration.image_to_robot(cx, cy)
        self.log(
            f"Image ({cx:.1f}, {cy:.1f}) -> Robot ({robot_x:.3f}, {robot_y:.3f})"
        )
#### 5/28 修改位姿补偿
        # self.client.vision_pick(robot_x, robot_y, r=Setting.pause_pick)
#  需要修改Setting.pause_pick为实时的r角度
        self.log(f"R relative correction delta = {angle:.3f}")
        self.client.vision_pick(
            robot_x,
            robot_y,
            r=Setting.pause_pick,
            r_delta=angle,
            target_name=detection.get("name"),
        )




    def show_frame(self, frame):
        if cv2 is None or Image is None or ImageTk is None:
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        image.thumbnail((self.canvas_width, self.canvas_height), Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(image)
        self.canvas.delete("all")
        self.canvas.create_image(
            self.canvas_width // 2,
            self.canvas_height // 2,
            image=self.photo,
            anchor="center",
        )











class RobotApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("控制界面")
        self.root.geometry("1000x820")
        self.root.resizable(False, False)

        # 标题样式
        self.style = ttk.Style()
        # 下面是我的标题的格式
        self.style.configure("Custom.TLabelframe.Label",
                             font=("微软雅黑", 16, "bold"),
                             foreground="black")
        self.style.configure("Custom.TLabelframe",
                             background="#f0f0f0",
                             borderwidth=2,
                             relief="solid")


        self.client = DobotClient(self.log)

        self.ip = tk.StringVar(value=DEFAULT_IP)
        self.dashboard_port = tk.IntVar(value=DEFAULT_DASHBOARD_PORT)
        self.move_port = tk.IntVar(value=DEFAULT_MOVE_PORT)
        self.step = tk.DoubleVar(value=8.0)
        self.speed = tk.IntVar(value=30)
        self.use_line = tk.BooleanVar(value=False)

        self.buttons_need_robot = []
        self.build_ui()
        self.set_robot_buttons(False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        # self.root.load_image_from_file("test.png")
    # def load_image_from_file(self, path):
    #     """加载本地图片文件（jpg/png/bmp 都可以）"""
    #     # 打开图片
    #     img = Image.open(path)
    #     # 等比例缩放到画布大小
    #     img = self._resize_to_fit(img)
    #     # 转成 tkinter 格式
    #     self.photo_image = ImageTk.PhotoImage(img)
    #     # 在 Canvas 中央显示
    #     self.canvas.delete("all")  # 清空旧图
    #     self.canvas.create_image(
    #         self.canvas_width // 2,
    #         self.canvas_height // 2,
    #         image=self.photo_image,
    #         anchor="center"
    #     )
    #     self.current_image = self.photo_image

    def build_ui(self):
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)

        left = ttk.Frame(self.root, padding=10)
        left.grid(row=0, column=0, sticky="ns")

        right = ttk.Frame(self.root, padding=(0, 10, 10, 10))
        right.grid(row=0, column=1, sticky="nsew")

        conn = ttk.LabelFrame(left, text="机器人连接")
        conn.grid(row=0, column=0, sticky="ew", pady=(0, 10))

        ttk.Label(conn, text="IP").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 2))
        ttk.Entry(conn, textvariable=self.ip, width=18).grid(row=1, column=0, padx=8)

        ttk.Label(conn, text="Dashboard").grid(row=0, column=1, sticky="w", padx=8, pady=(8, 2))
        ttk.Entry(conn, textvariable=self.dashboard_port, width=8).grid(row=1, column=1, padx=8)

        ttk.Label(conn, text="Move").grid(row=0, column=2, sticky="w", padx=8, pady=(8, 2))
        ttk.Entry(conn, textvariable=self.move_port, width=8).grid(row=1, column=2, padx=8)

        self.connect_button = ttk.Button(conn, text="连接", command=self.toggle_connect)
        self.connect_button.grid(row=2, column=0, columnspan=3, sticky="ew", padx=8, pady=8)

        '''
        机器控制  分区里面的ui
        '''

        action = ttk.LabelFrame(left, text="机器使能")
        action.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        self.add_robot_button(action, "使能", self.enable, 0, 0)
        self.add_robot_button(action, "下使能", self.disable, 0, 1)
        self.add_robot_button(action, "清除报警", self.clear_error, 0, 2)  # 525修改: 错别字 清楚 → 清除
        self.add_robot_button(action, "当前位置", self.get_pose, 1, 0)

        ttk.Label(action, text="速度 %").grid(row=1, column=1, sticky="e", padx=(8, 2), pady=8)
        speed_box = ttk.Spinbox(action, from_=1, to=100, textvariable=self.speed, width=6)
        speed_box.grid(row=1, column=2, padx=(0, 8), pady=8)
        self.add_robot_button(action, "设置速度", self.set_speed, 2, 0, columnspan=3)

        '''
        move 分区里面的ui
        '''
        move = ttk.LabelFrame(left, text="机器人控制", style="Custom.TLabelframe")

        move.grid(row=2, column=0, sticky="ew")
        ttk.Label(move, text="步长").grid(row=0, column=0, padx=8, pady=(8, 2), sticky="w")
        ttk.Spinbox(move, from_=0.1, to=100.0, increment=0.5, textvariable=self.step, width=8).grid(
            row=1, column=0, padx=8, sticky="ew"
        )
        ttk.Checkbutton(move, text="Linear", variable=self.use_line).grid(
            row=1, column=1, padx=8, sticky="w"
        )

        controls = [
            ("X+", 2, 0, dict(dx=1)),
            ("X-", 2, 1, dict(dx=-1)),
            ("Y+", 3, 0, dict(dy=1)),
            ("Y-", 3, 1, dict(dy=-1)),
            ("Z+", 4, 0, dict(dz=1)),
            ("Z-", 4, 1, dict(dz=-1)),
            ("R+", 5, 0, dict(dr=1)),
            ("R-", 5, 1, dict(dr=-1)),
        ]
        for text, row, col, kwargs in controls:
            self.add_robot_button(
                move,
                text,
                lambda values=kwargs: self.relative_move(**values),
                row,
                col,
            )


        ##  回原点的 设计 可以控制机器人回到安全姿态（J1=3°、J2=12°、J3=24°、J4=12°）  # 525修改: 注释与 go_zero 实际参数对齐
        self.add_robot_button(move, "回原点", self.go_zero, 6, 0)



        self.vision = VisionPanel(right, self.client, self.log)  # 525修改: 把 client 和 log 都传进去，原来只传了 log 一个参数
        self.vision.frame.grid(row=0, column=0, sticky="new")

        log_frame = ttk.LabelFrame(right, text="监控界面", style="Custom.TLabelframe")
        log_frame.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self.log_text = ScrolledText(log_frame, width=68, height=13)
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

    def add_robot_button(self, parent, text, command, row, column, columnspan=1):
        button = ttk.Button(parent, text=text, command=command)
        button.grid(row=row, column=column, columnspan=columnspan, sticky="ew", padx=8, pady=6)
        self.buttons_need_robot.append(button)
        return button

    def set_robot_buttons(self, enabled):
        state = "normal" if enabled else "disabled"
        for button in self.buttons_need_robot:
            button.configure(state=state)

    def log(self, message):
        def write():
            timestamp = time.strftime("%H:%M:%S")
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)

        if hasattr(self, "log_text"):
            self.root.after(0, write)
        else:
            print(message)

    def run_robot_task(self, func):
        def task():
            try:
                result = func()
                if result:
                    self.log(str(result).strip())
            except Exception as exc:
                self.log(f"Robot command failed: {exc}")
                self.root.after(0, lambda: messagebox.showerror("Robot error", str(exc)))

        threading.Thread(target=task, daemon=True).start()

    def toggle_connect(self):
        if self.client.connected:
            self.client.disconnect()
            self.connect_button.configure(text="连接")
            self.set_robot_buttons(False)
            self.log("Robot disconnected.")
            return

        def task():
            try:
                self.client.connect(
                    self.ip.get().strip(),
                    int(self.dashboard_port.get()),
                    int(self.move_port.get()),
                )
                self.root.after(0, lambda: self.connect_button.configure(text="Disconnect"))
                self.root.after(0, lambda: self.set_robot_buttons(True))
            except Exception as exc:
                self.log(f"Connect failed: {exc}")
                self.root.after(0, lambda: messagebox.showerror("Connect failed", str(exc)))

        threading.Thread(target=task, daemon=True).start()

    def enable(self):
        self.run_robot_task(self.client.enable)

    def disable(self):
        self.run_robot_task(self.client.disable)

    def clear_error(self):
        self.run_robot_task(self.client.clear_error)

    def set_speed(self):
        speed = max(1, min(100, int(self.speed.get())))
        self.run_robot_task(lambda: self.client.set_speed(speed))

    def get_pose(self):
        self.run_robot_task(self.client.get_pose)

    def relative_move(self, dx=0, dy=0, dz=0, dr=0):
        step = float(self.step.get())
        self.run_robot_task(
            lambda: self.client.rel_move(
                dx=dx * step,
                dy=dy * step,
                dz=dz * step,
                dr=dr * step,
                straight=self.use_line.get(),
            )
        )

    def on_close(self):
        self.client.disconnect()
        self.root.destroy()

    def mainloop(self):
        self.root.mainloop()

    def go_zero(self):
        self.run_robot_task(self.client.go_zero)







if __name__ == "__main__":
    RobotApp().mainloop()
