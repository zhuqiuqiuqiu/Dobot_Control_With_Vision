import os
from ctypes import *

# 添加 DLL 搜索路径
os.add_dll_directory(
    r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"
)

# 加载 DLL
MvCamCtrldll = WinDLL(
    r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64\MvCameraControl.dll"
)

print("DLL加载成功")