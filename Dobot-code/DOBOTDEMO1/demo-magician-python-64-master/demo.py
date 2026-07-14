import DobotDllType as dType
import time

# ======================
# 连接状态文字
# ======================
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError: "连接成功",
    dType.DobotConnect.DobotConnect_NotFound: "未找到机械臂",
    dType.DobotConnect.DobotConnect_Occupied: "串口被占用"
}

# ======================
# 加载 DLL
# ======================
api = dType.load()

# ======================
# 连接机械臂
# ======================
state = dType.ConnectDobot(api, "", 115200)[0]

print("连接状态:", CON_STR[state])

# ======================
# 连接成功
# ======================
if state == dType.DobotConnect.DobotConnect_NoError:

    # 清空队列
    dType.SetQueuedCmdClear(api)

    # 设置运动参数
    dType.SetPTPJointParams(
        api,
        100, 100, 100, 100,
        100, 100, 100, 100,
        isQueued=1
    )

    dType.SetPTPCommonParams(
        api,
        50,   # velocity
        50,   # acceleration
        isQueued=1
    )

    print("开始运动...")

    # ======================
    # 点1
    # ======================
    lastIndex = dType.SetPTPCmd(
        api,
        dType.PTPMode.PTPMOVJXYZMode,
        200,
        0,
        50,
        0,
        isQueued=1
    )[0]

    # ======================
    # 点2
    # ======================
    lastIndex = dType.SetPTPCmd(
        api,
        dType.PTPMode.PTPMOVJXYZMode,
        250,
        50,
        30,
        0,
        isQueued=1
    )[0]

    # ======================
    # 点3
    # ======================
    lastIndex = dType.SetPTPCmd(
        api,
        dType.PTPMode.PTPMOVJXYZMode,
        200,
        -50,
        60,
        0,
        isQueued=1
    )[0]

    # ======================
    # 回到中间
    # ======================
    lastIndex = dType.SetPTPCmd(
        api,
        dType.PTPMode.PTPMOVJXYZMode,
        220,
        0,
        40,
        0,
        isQueued=1
    )[0]

    # ======================
    # 开始执行
    # ======================
    dType.SetQueuedCmdStartExec(api)

    # 等待执行完成
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

    # 停止队列
    dType.SetQueuedCmdStopExec(api)

    print("运动完成")

# ======================
# 断开连接
# ======================
dType.DisconnectDobot(api)

print("程序结束")