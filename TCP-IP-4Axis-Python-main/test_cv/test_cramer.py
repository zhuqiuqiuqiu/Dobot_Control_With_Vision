# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np

from ctypes import *

from Python.MvImport.MvCameraControl_class import *
from Python.MvImport.CameraParams_header import *
from Python.MvImport.MvErrorDefine_const import *


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


if __name__ == "__main__":

    camera = HikCamera()

    if camera.open_camera():

        frame = camera.take_photo()

        if frame is not None:

            # 显示图像
            cv2.imshow("Photo", frame)

            # 保存图像
            cv2.imwrite("test.jpg", frame)

            print("Image Saved -> test.jpg")

            cv2.waitKey(0)

        camera.close_camera()

    cv2.destroyAllWindows()

