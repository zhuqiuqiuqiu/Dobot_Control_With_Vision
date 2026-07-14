# -*- coding: utf-8 -*-

"""
MG400 TCP/IP GUI demo with manual movement and optional YOLO detection.

Run:
    python 5_28版本.py

The robot side uses dobot_api.py in this repository. Vision features need:
    pip install opencv-python pillow ultralytics
"""

from ctypes import *
import numpy as np
from Python.MvImport.MvCameraControl_class import *
from Python.MvImport.CameraParams_header import *
from Python.MvImport.MvErrorDefine_const import *
import threading
import time
import tkinter as tk
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
    需要标定转换的文件：这里需要进行标定转换
    do1：   吸气的 数字开关  1开  0关
    do2：   呼气的 数字开关

    '''
    # 需要修改的参数
    safe_high = 20
    pick_high = 10
    pause_pick = 0  # 0°


    # 固定参数
    do1 = 1
    do2 = 2
    do1_yes = 1
    do1_no = 0
    do2_yes = 1
    do2_no = 0

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
        '''清楚机械臂报警'''
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
     ###  回原点的函数，是J1-J4四个轴回到0°  ， 使用关节模式movJ
        self.require_connected()
        return self.move.JointMovJ(3, 12, 24, 12)

    # def vision_pick(self, x, y, r):
    #
    #     self.require_connected()
    #     self.move.MovJ(x, y, Setting.safe_high, r) # 移动到点上面的安全距离
    #     self.move.MovL(x, y, Setting.pick_high, r) # 点的位置
    #     time.sleep(0.1)
    #     self.dashboard.DO(Setting.do1, Setting.do1_yes)  #打开吸气
    #     time.sleep(0.3) # 持续吸气
    #     self.move.MovL(x, y, Setting.safe_high, r)  # 抬起








class VisionPanel:
    def __init__(self, parent, client, log_callback):  # 525修改: 补上 log_callback 参数，原来只传 client 导致 self.log 缺失
        self.parent = parent
        self.log = log_callback  # 525修改: 原来这行被注释掉，导致 self.log(...) 全部报 AttributeError
        self.model = None
        self.client = client
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
        self.canvas.grid(row=2, column=0, columnspan=2, padx=8, pady=8)
        self.canvas.create_text(
            self.canvas_width // 2,
            self.canvas_height // 2,
            fill="#dddddd",
            text="Camera preview",
        )
        ###==================================================
        ###  初始的监控界面的图片
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
        ###===================================================


        ttk.Button(self.frame, text="拍照", command=self.take_photo_async).grid(
            row=3, column=0, sticky="ew", padx=8, pady=(0, 8)
        )
        ttk.Button(self.frame, text="Detect Once", command=self.detect_once_async).grid(
            row=3, column=1, sticky="ew", padx=(0, 8), pady=(0, 8)
        )
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
        if self.model is None:
            self.parent.after(0, self.load_model)
            time.sleep(0.5)
            if self.model is None:
                return

        frame = self.last_frame if self.last_frame is not None else self.take_photo()
        if frame is None:
            return

        try:
            results = self.model(frame, conf=0.5, verbose=False)
            annotated = results[0].plot()
            boxes = results[0].boxes
            if len(boxes) == 0:
                self.log("未检测到目标")
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.model.names.get(cls_id, str(cls_id))
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                self.log(f"recevied: {name},{conf:.2f}, {cx:.1f}, {cy:.1f}")

###     这里需要进行标定转换 像素坐标->标定矩阵->机械臂抓取坐标

                # self.client.vision_pick(cx,cy,r=0)


            self.parent.after(0, lambda: self.show_frame(annotated))






        except Exception as exc:
            self.log(f"视觉检测错误: {exc}")

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
        self.root.geometry("980x620")
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
        self.add_robot_button(action, "清楚报警", self.clear_error, 0, 2)
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


        ##  回原点的 设计 可以控制机器人回到原点姿态（J1轴 0°、J2轴 0°、J3轴0°、J4轴 0°）
        self.add_robot_button(move, "回原点", self.go_zero, 6, 0)



        self.vision = VisionPanel(right, self.log)
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
