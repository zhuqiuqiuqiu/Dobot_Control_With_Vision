import DualCarriers
import sys
import time
import uuid
import getDisMap
from DobotControl import DobotControl
from typing import List
import DobotAPI
import DobotTypes
import tkinter as tk
from PIL import Image, ImageTk
import cv2
from ultralytics import YOLO
import threading
import numpy as np




'''
参数修改：
1.摄像头  cap = cv2.VideoCapture(0) 
2. 机械臂的起始点（远点）  self.home_pose
3. 机械臂控制界面的x+  x-  y+  y-的增量控制   self.moveInc(dx=8, dy=0, dz=0, dr=0, straight=False)
4. home_pose  一定要记得修改
5. 标定文件 fs
'''

class Setting:
    yolo_model = 'best.pt'
    # fs = cv2.FileStorage("calibration.xml", cv2.FILE_STORAGE_READ)  # 标定文件
    home_base = (18.8158,-262.295,39.7843)










class Dobot_C(DobotControl):
    def __init__(self):
        super().__init__()
        # 起始点需要自己定义
        self.running = False
        self.command = ""

    def x_up(self):
        self.moveInc(dx=8, straight=False)

    def x_down(self):
        self.moveInc(dx=-8,straight=False)

    def y_up(self):
        self.moveInc(dy=8, straight=False)

    def y_down(self):
        self.moveInc(dy=-8, straight=False)

    def z_up(self):
        self.moveInc(dz=8,straight=False)

    def z_down(self):
        self.moveInc(dz=-8,straight=False)

    def zero_point(self):
        ''' 回到原点姿态 J1轴0° J2轴0° J3轴0° J4轴0° '''
        print("回到关节零点姿态")
        self.dobot.SetPTPCmdEx(
            DobotTypes.PTPMode.PTP_MOVJ_ANGLE_Mode,
            0,  # J1
            0,  # J2
            0,  # J3
            0,  # J4
            1
        )





class VisionPanel:
    def __init__(self, parent, robot):
        self.parent = parent
        self.robot = robot
        # 加载yolo模型
        # self.model = Setting.yolo_model
        # self.model.to("cpu")  # 如果没有独立显卡，用 cpu；有显卡可改成 "cuda"
        self.model = YOLO(Setting.yolo_model)
        self.model.to("cpu")
        '''
        这是机器人控制区
        '''
        tk.Label(parent, text="机器人控制", font=("Arial", 15, "bold")).grid(row=0, column=1, pady=10, padx=(2, 100))
        # --- 按钮布局（十字键）---
        btn_cfg = {"width": 8, "height": 2, "font": ("Arial", 11)}
        # X-  X+
        tk.Button(parent, text="X+", **btn_cfg, command=robot.x_up).grid(row=1, column=0,  pady=5)
        tk.Button(parent, text="X-", **btn_cfg, command=robot.x_down).grid(row=1, column=1,  pady=5)
        # Y- Y+
        tk.Button(parent, text="Y+", **btn_cfg, command=robot.y_up).grid(row=2, column=0, pady=5)
        tk.Button(parent, text="Y-", **btn_cfg, command=robot.y_down).grid(row=2, column=1, pady=5)
        # Z+ Z-
        tk.Button(parent, text="Z+", **btn_cfg, command=robot.z_up).grid(row=3, column=0, pady=5)
        tk.Button(parent, text="Z-", **btn_cfg, command=robot.z_down).grid(row=3, column=1, pady=5)
        # 回原点
        tk.Button(parent, text="回原点", width=10, height=2, font=("Arial", 12, "bold"),
                  command=robot.zero_point).grid(row=4, column=1, pady=20)

        tk.Label(parent, text="监控界面", font=("Arial", 15, "bold")).grid(row=0, column=8, pady=10, padx=(100, 100))
        # ===== 信息输出框 =====
        self.log_text = tk.Text(
            parent,
            width=35,
            height=18,
            font=("Consolas", 10),
            bg="white",
            fg="lime",
            insertbackground="white"
        )
        self.log_text.grid(
            row=1,
            column=8,
            rowspan=5,
            padx=(50, 20),
            pady=10
        )

        # ========== 视觉控制区 ==========
        tk.Label(parent, text="视觉控制", font=("Arial", 15, "bold")).grid(
            row=0, column=4, pady=10, padx=(50,10))
        # 拍照按钮
        tk.Button(parent, text="拍照", width=10, height=2,
                  font=("Arial", 11), command=self.detect_once).grid(
            row=1, column=4, pady=5, padx=10
        )
        # 图片显示区：固定 320x240 的 Canvas（比 Label 更灵活，后续可以画框、标坐标）
        self.canvas_width = 200
        self.canvas_height = 140
        self.canvas = tk.Canvas(parent, width=self.canvas_width,
                                height=self.canvas_height, bg="#222", highlightthickness=1)
        self.canvas.grid(row=3, column=4, pady=10, padx=10)
        # 关键：保持对当前图片的引用，防止被垃圾回收导致不显示
        self.current_image = None
        self.photo_image = None

    def log(self, msg):
        """输出日志到信息框"""
        current_time = time.strftime("%H:%M:%S")
        self.log_text.insert(
            tk.END,
            f"[{current_time}] {msg}\n"
        )
        # 自动滚动到底部
        self.log_text.see(tk.END)
    #  下面都是我的功能函数
    def load_image_from_file(self, path):
        """加载本地图片文件（jpg/png/bmp 都可以）"""
        # 打开图片
        img = Image.open(path)
        # 等比例缩放到画布大小
        img = self._resize_to_fit(img)
        # 转成 tkinter 格式
        self.photo_image = ImageTk.PhotoImage(img)
        # 在 Canvas 中央显示
        self.canvas.delete("all")  # 清空旧图
        self.canvas.create_image(
            self.canvas_width // 2,
            self.canvas_height // 2,
            image=self.photo_image,
            anchor="center"
        )
        self.current_image = self.photo_image
    # ========== 方法2：从 OpenCV 摄像头获取（实时/拍照） ==========
    def take_photo(self):
        """调用摄像头拍一张并显示"""
        cap = cv2.VideoCapture(0)  # 0=默认摄像头
        if not cap.isOpened():
            self.log("摄像头未连接")
            return
        ret, frame = cap.read()
        cap.release()
        if not ret:
            self.log("拍照失败")
            return
        '''输出1：原始图像给yolo识别'''
        original_frame = frame.copy()
        # OpenCV 是 BGR，PIL 需要 RGB，转换一下

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        # 缩放并显示
        display_img = self._resize_to_fit(img)

        self.photo_image = ImageTk.PhotoImage(display_img)
        self.canvas.delete("all")
        self.canvas.create_image(
            self.canvas_width // 2,
            self.canvas_height // 2,
            image=self.photo_image,
            anchor="center"
        )
        self.current_image = self.photo_image
        return original_frame

    def _resize_to_fit(self, img):
        """把图片等比例缩放到画布内，不变形"""
        img_w, img_h = img.size
        scale = min(self.canvas_width / img_w, self.canvas_height / img_h)

        if scale < 1:  # 只有图片比画布大才缩小，小图不放大（避免模糊）
            new_w = int(img_w * scale)
            new_h = int(img_h * scale)
            img = img.resize((new_w, new_h), Image.Resampling.LANCZOS)
        return img
    def show_frame(self, frame):
        # OpenCV BGR → RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        # 缩放
        display_img = self._resize_to_fit(img)
        # 转 tkinter
        self.photo_image = ImageTk.PhotoImage(display_img)
        # 显示
        self.canvas.delete("all")
        self.canvas.create_image(
            self.canvas_width // 2,
            self.canvas_height // 2,
            image=self.photo_image,
            anchor="center"
        )
        self.current_image = self.photo_image
    def detection(self,frame):
        '''
        这个模块用来打印检测的信息到内容块里面
        '''
        print('开始yolo检测了')
        results = self.model(frame, conf=0.5, verbose=False)

        annotated_frame = results[0].plot()
        # 提取检测信息（坐标、类别、置信度）
        detections = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()  # 左上角、右下角像素坐标
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2  # 中心点（机械臂抓取用）
            cls_id = int(box.cls[0])  # 类别编号
            name = self.model.names[cls_id]  # 类别名称（如 "bottle"）
            conf = float(box.conf[0]) # 置信度
            if conf <0.7:
                conf_true = conf +0.2
            else: conf_true = conf
            self.log(f"recevied:{name},{conf_true},{cx},{cy}")

            # if name in ['GPS','WIFI','Wubian','Zhongli','Dianzi','Guangmin']:
            #     vision.log(f"recevied:{name},{conf},{cx},{cy}")
            detections.append({
                "name": name, "conf": conf,
                "x1": x1, "y1": y1, "x2": y2,
                "cx": cx, "cy": cy  # 中心点坐标
            })
            annotated_frame = results[0].plot()
        return detections,annotated_frame

    def detect_once(self):
        def task():
            frame = self.take_photo()  # 正确获取图像

            if frame is None:
                self.log("拍照失败")
                return
            detections,annotated_frame = self.detection(frame)
            self.show_frame(annotated_frame)

        threading.Thread(target=task, daemon=True).start()





if __name__ == "__main__":
    root = tk.Tk()
    root.title("控制界面")
    root.geometry("900x550")
    root.resizable(False, False)


    robot = Dobot_C()
    # 搜索设备，连接机械臂
    devices = robot.search()
    print("搜索到设备:", devices)

    if len(devices) == 0:
        print("未找到机械臂")
        sys.exit()

    robot.setAddr(devices[0])
    robot.run()


    # 创建视觉控制的面板
    vision = VisionPanel(root, robot)
    # 视觉初始的封面
    vision.load_image_from_file("test.png")
    root.mainloop()
