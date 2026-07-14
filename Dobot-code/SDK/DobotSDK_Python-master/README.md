# DobotSDK_Python

这是一个基于 Dobot 官方动态库的 Python SDK/示例工程。项目通过 `ctypes` 加载 Dobot 的 `DobotDll` 动态库，向上封装出机械臂连接、运动控制、末端执行器、I/O、颜色传感器、传送带电机、Wi-Fi 配置等接口，并包含若干用于单机分拣、双机协同搬运、调试和速度测试的脚本。

> 注意：本项目里的不少脚本会直接控制真实机械臂、传送带、电磁泵/夹爪等硬件。运行前请确认端口、坐标、速度、急停和工作空间安全。

## 一、项目整体功能

项目大致分为 4 层：

1. 动态库层：`Windows/x64/`、`Darwin/x64/` 中的 Dobot 动态库和头文件。
2. API 封装层：`DobotAPI.py` 使用 `ctypes` 调用动态库函数；`DobotTypes.py` 定义结构体、枚举和常量。
3. 会话/控制层：`DobotSession.py` 把底层 API 包成对象方法；`DobotControl.py` 再封装连接、初始化、移动、吸取、颜色识别、电机控制等常用动作。
4. 业务脚本层：`SinglePlayer.py`、`DualCarriers.py`、`KeyBoardControl.py` 等脚本基于控制层实现具体流程。

## 二、目录结构

```text
DobotSDK_Python-master/
├── DobotAPI.py              # 底层 ctypes API 封装
├── DobotTypes.py            # Dobot 结构体、枚举、常量定义
├── DobotSession.py          # 面向单个机械臂的会话对象
├── DobotControl.py          # 线程化机械臂控制基类和通用动作
├── SinglePlayer.py          # 单机械臂颜色分拣流程
├── DualCarriers.py          # 双机械臂 + 传送带协同分拣流程
├── KeyBoardControl.py       # 命令行交互调试多个机械臂
├── Musician.py              # 泵/颜色传感器测试示例
├── MutiInOneTime.py         # 多机械臂同步/并发实验脚本
├── getDisMap.py             # 传送带距离标定数据拟合工具
├── speed_test.py            # 传送带定距运行测试
├── TestingSpeed.py          # 旧版/实验性质的速度测试脚本
├── tst2.py                  # Python 表达式临时测试脚本
├── data/
│   └── distance_map.txt     # 传送带距离标定原始数据
├── mycode/
│   └── test/test1.py        # 手动加载 Windows DLL 的最小测试
├── Windows/x64/             # Windows 64 位 Dobot 动态库和依赖 DLL
├── Darwin/x64/              # macOS 64 位 Dobot 动态库和 Qt 依赖
└── .idea/                   # PyCharm/IntelliJ 项目配置，可不参与运行
```

## 三、核心代码说明

### `DobotAPI.py`

底层 API 封装文件，负责把 Python 调用转换为 Dobot 动态库调用。

主要功能：

- 自动判断系统和位数，拼出动态库路径：`./Windows/x64/`、`./Darwin/x64/` 等。
- `load(dobotId=0, split="_")` 加载动态库；为了支持多个机械臂，会把原始 DLL/dylib 复制成带编号的文件再加载，例如 `DobotDll_0.dll`、`DobotDll_1.dll`。
- `SearchDobot()` 搜索可连接设备。
- `ConnectDobot()` / `DisconnectDobot()` 连接和断开机械臂。
- 队列控制：开始、停止、强停、清空、查询队列命令索引。
- 设备信息：设备 SN、名称、版本、姿态、报警状态。
- 运动控制：HOME、JOG、PTP、CP、ARC、WAIT、TRIG 等。
- 末端执行器：激光、吸盘、夹爪。
- I/O：复用、DO、DI、PWM、ADC。
- 扩展模块：传送带电机、颜色传感器、红外传感器、Wi-Fi、固件模式、丢步检测、PID 等。
- `xxxEx` 方法通常会在发送队列命令后等待该命令执行到对应队列索引，适合需要同步等待动作完成的场景。

### `DobotTypes.py`

Dobot API 的类型定义文件，基本不直接运行，而是供 `DobotAPI.py` 和上层代码引用。

主要内容：

- `ctypes.Structure` 结构体：`Pose`、`HOMEParams`、`PTPCmd`、`CPCmd`、`ARCCmd`、`IOMultiplexing`、`IODO`、`EMotor`、`WIFIIPAddress`、`PID` 等。
- 枚举/常量类：`PTPMode`、`ContinuousPathMode`、`IOFunction`、`DobotConnect`、`DobotCommunicate`、`ColorPort`、`EMotorPort` 等。
- 结果码映射：`CONNECT_RESULT`、`Communicate_RESULT`，用于把连接/通信结果转成可读字符串。

### `DobotSession.py`

面向单个机械臂的对象封装层。它内部持有一个 `self.api` 动态库实例，然后把 `DobotAPI.py` 中的函数转成对象方法。

典型用法：

```python
from DobotSession import DobotSession

dobot = DobotSession()
ports = dobot.SearchDobot()
dobot.ConnectDobot(ports[0])
dobot.GetPose()
dobot.SetPTPCmdEx(1, 200, 0, 50, 0, 1)
dobot.DisconnectDobot()
```

主要价值：

- 多机械臂时，每个 `DobotSession` 可以加载独立 DLL 实例。
- 调用方式更接近面向对象：`dobot.GetPose()`、`dobot.SetPTPCmdEx(...)`。
- 覆盖了大部分底层 API，包括普通方法和 `Ex` 同步等待方法。

### `DobotControl.py`

线程化控制基类，业务脚本一般继承它来写自己的机械臂流程。

核心设计：

- `DobotControl` 继承 `threading.Thread`，可以用 `start()` 启动线程执行。
- `search()` 搜索设备并缓存结果。
- `setAddr(addr)` 设置串口/IP 地址。
- `connect()` 创建 `DobotSession` 并连接设备。
- `init()` 做机械臂初始化：清报警、停止/清空/启动队列、设置 PTP 参数、复位姿态、关闭吸取。
- `run()` 的流程是：连接 -> 初始化 -> 执行 `work()` -> 清理。
- 子类通常重写 `user_init()` 和 `work()`。

常用动作封装：

- `moveTo(x, y, z, r, straight=False)` 移动到指定坐标。
- `moveInc(dx, dy, dz, dr, straight=False)` 按增量移动。
- `suck()`、`blow()`、`unsuck()` 控制吸取/吹气/关闭。
- `setPump(power_port, control_port)` 使用外接泵的两个 I/O 口。
- `setColotSensor(...)` 配置外接颜色传感器 I/O。函数名里 `Colot` 应为原代码拼写。
- `getColorSensor()` 读取颜色传感器，返回红/绿/蓝状态。
- `startMoto()`、`startMotoS()`、`stopMoto()` 控制传送带电机。
- `reset_zero(home_pose)` 设置 HOME 点并执行回零。
- `reset_pose()` 尝试通过丢步检测/姿态重置恢复机械臂姿态。

辅助函数：

- `color_exists(n)` 判断颜色读数里是否存在有效颜色。
- `ensure_color_index(tpl, default=-1)` 把颜色元组转换成颜色下标。
- `ensure_color_tuple(index, default=(0, 0, 0))` 把颜色下标转换成颜色元组。

## 四、业务脚本说明

### `SinglePlayer.py`

单机械臂颜色分拣流程。脚本定义了一个 `Robot(DobotControl)`，用于从固定取料区抓取方块，移动到颜色传感器位置识别颜色，再按红/绿/蓝分类放置，无法识别的方块放到废料位。

主要内容：

- `Settings`：保存单机分拣用的坐标、间距、块大小、颜色端口、速度参数、调试开关等。
- `Robot.user_init()`：启用颜色传感器，可选执行 HOME 初始化和调试动作。
- `Robot.work()`：循环处理 12 个方块。
- `moveToGetPlace()`：根据方块编号计算并移动到取料点。
- `gotoColor()`：移动到颜色识别点。
- `readColor()`：多次读取颜色，必要时用蜂鸣声提示识别结果。
- `moveToPutPlace()`：根据颜色和已有堆叠数量计算放置点。
- `capture()` / `release()`：吸取和释放方块。

运行入口：

```bash
python SinglePlayer.py
```

脚本默认使用搜索到的第一个设备：`Robot.search()[0]`。

### `DualCarriers.py`

双机械臂 + 传送带协同分拣流程。一个左侧机械臂负责把物块放上传送带，右侧机械臂负责从传送带/临时区取料、识别颜色并分类放置。

主要内容：

- `Settings`：双机流程的端口、坐标、传送带距离、速度、颜色序列、启停开关等。
- `Interal`：延迟执行某个函数的小线程，主要用于延迟启动传送带。
- `ColorInteral`：右侧颜色检测线程，用来监控物块是否到达。
- `Right(DobotControl)`：右侧机械臂，负责临时块处理、颜色识别、取料、分类放置、废料处理、右侧传送带控制。
- `Left(DobotControl)`：左侧机械臂，负责从左侧取料区取方块，放到传送带入口，并控制传送带定距运行。
- `Global`：保存左右机械臂共享状态，例如是否运行、是否已取走、第一块是否到达、是否由右侧接管传送带等。

运行入口：

```bash
python DualCarriers.py
```

默认端口在 `Settings.COM_LEFT` 和 `Settings.COM_RIGHT` 中配置，例如 `COM6`、`COM5`。运行前需要按实际设备修改。

### `KeyBoardControl.py`

命令行交互式调试工具。它会搜索所有 Dobot 设备，为每个设备创建一个控制线程，然后通过命令行发送动作命令。

主要命令：

- `q`：退出。
- `0s`：打印 0 号机械臂当前姿态。
- `0c`：打印 0 号机械臂颜色传感器读数。
- `0hm`：0 号机械臂回到脚本记录的 home_pose。
- `0us`：关闭吸取。
- `0sk`：开始吸取。
- `0cl`：以当前 home_pose 执行回零。
- `0sr`：把当前姿态保存为 home_pose。
- `0mu 10`：0 号机械臂 Z 轴上移 10。
- `0md 10`：Z 轴下移 10。
- `0ml 10` / `0mr 10`：Y 方向移动。
- `0mf 10` / `0mb 10`：X 方向移动。
- `0mrh 10`：R 轴增量旋转。
- `0to 200 0 50 0`：移动到指定 `x y z r`。

运行入口：

```bash
python KeyBoardControl.py
```

### `Musician.py`

简单的泵/夹爪或颜色传感器测试脚本。目前 `work()` 中实际执行的是循环吸取 1 秒、关闭 1 秒。

主要用途：

- 测试 `setPump(17, 11)` 配置的外接泵是否工作。
- 注释代码里包含颜色传感器读取和蜂鸣提示逻辑，可用于临时调试。

运行入口：

```bash
python Musician.py
```

### `MutiInOneTime.py`

多机械臂并发/同步实验脚本。脚本里固定了 `sr = ["10.3.21.125"]`，并尝试通过 Wi-Fi/IP 方式连接设备，让机械臂按全局计数同步上下移动。

主要内容：

- `UPPER`、`LOWER`：全局同步计数。
- `mutex`：保护全局计数的线程锁。
- `Dobot.user_init()`：读取/配置 Wi-Fi 信息，启用颜色传感器，停止电机。
- `Dobot.work()`：根据全局计数协调多台机械臂上下移动。
- `Command`：启动和停止多个 `Dobot` 线程。

注意：该脚本实验性质较强，里面的 IP、Wi-Fi 名称、密码等需要按现场环境修改。

### `getDisMap.py`

传送带距离标定工具。它读取 `data/distance_map.txt` 中的标定数据，用 `numpy` 和 `scipy.optimize.curve_fit` 拟合电机 tick 与实际距离之间的线性关系。

主要功能：

- `get_dis_map()`：读取标定数据并计算相邻标定点差值。
- `get_dis_tick(mm)`：把期望传送距离换算为电机需要运行的 tick。
- 直接运行时会打印标定映射和示例距离，并用 `matplotlib` 画出拟合曲线。

该文件被 `DobotControl.startMotoS()` 调用，用于传送带定距运行。

### `speed_test.py`

传送带定距运行测试脚本。

主要内容：

- 定义 `Dbt(DobotControl)`。
- 在 `work()` 中依次测试 520、525、530、535 等距离参数。
- 每次运行传送带后等待用户输入，再进入下一组距离。

运行入口：

```bash
python speed_test.py
```

### `TestingSpeed.py`

旧版/实验性质的左侧机械臂速度测试脚本，逻辑上是用不同速度执行取放块流程。

需要注意：

- 代码中的 `Left.__init__()` 调用了 `super().__init__(index, addr)`，但当前 `DobotControl.__init__()` 不接收这两个参数，因此该脚本可能是旧版本遗留，直接运行大概率需要先修正构造函数。
- 可作为历史测试思路参考，不建议直接作为正式入口。

### `tst2.py`

非常小的 Python 表达式测试文件：

```python
a = (0 is not None) or 233
print(a)
```

它不参与 Dobot 控制逻辑，只是临时验证 Python 表达式行为。

### `mycode/test/test1.py`

手动加载 Windows DLL 的最小测试脚本。

主要功能：

- 使用 `ctypes.CDLL()` 加载 `Windows/x64/DobotDll.dll`。
- 加载成功后打印提示。

用途：

- 排查 DLL 路径是否正确。
- 排查 Windows 依赖 DLL 是否齐全。
- 在不运行完整 SDK 的情况下验证动态库能否被 Python 加载。

## 五、数据和动态库说明

### `data/distance_map.txt`

传送带标定数据文件。每行大致是：

```text
电机tick 实测距离
```

其中 `init` 行用于分段初始化。`getDisMap.py` 会读取该文件并拟合出距离换算关系。

### `Windows/x64/`

Windows 64 位运行所需文件：

- `DobotDll.dll`：Dobot 主动态库。
- `DobotDll_0.dll`、`DobotDll_1.dll`：多机械臂加载时生成/使用的编号副本。
- `Qt5Core.dll`、`Qt5Network.dll`、`Qt5SerialPort.dll`：Qt 运行依赖。
- `msvcp120.dll`、`msvcr120.dll`：Visual C++ 运行库依赖。
- `DobotDll.h`、`DobotType.h`、`dobotdll_global.h`：C/C++ 头文件。
- `DobotDll.lib`、`DobotDll.exp`：Windows 链接相关文件。

### `Darwin/x64/`

macOS 64 位运行所需文件：

- `libDobotDll.dylib` 及其版本化链接/副本。
- `QtCore.framework`、`QtNetwork.framework`、`QtSerialPort.framework`：Qt framework 依赖。
- `DobotDll.h`、`DobotType.h`、`dobotdll_global.h`：C/C++ 头文件。

### `.idea/`

PyCharm/IntelliJ 项目配置文件，不影响 SDK 运行。共享项目时可以保留，也可以按团队规范忽略。

## 六、运行环境和依赖

基础依赖：

- Python 3.x
- Dobot 机械臂及对应驱动/串口权限
- 本项目附带的 Dobot 动态库

部分脚本额外依赖：

- `numpy`：`getDisMap.py` 使用。
- `scipy`：`getDisMap.py` 拟合数据使用。
- `matplotlib`：`getDisMap.py` 画图使用。
- `winsound`：`SinglePlayer.py`、`Musician.py` 中的蜂鸣提示使用，只适用于 Windows。

可用以下命令安装数据拟合相关依赖：

```bash
pip install numpy scipy matplotlib
```

## 七、基本使用流程

### 1. 搜索并连接机械臂

```python
from DobotControl import DobotControl

ports = DobotControl.search()
print(ports)
```

### 2. 继承 `DobotControl` 编写自己的流程

```python
from DobotControl import DobotControl

class MyRobot(DobotControl):
    def user_init(self):
        self.speed = 500
        self.acc = 300

    def work(self):
        self.moveTo(200, 0, 50, 0)
        self.suck()
        self.moveInc(dz=30)
        self.unsuck()

robot = MyRobot()
robot.setAddr(DobotControl.search()[0])
robot.start()
robot.join()
```

### 3. 运行现有脚本

单机分拣：

```bash
python SinglePlayer.py
```

双机协同：

```bash
python DualCarriers.py
```

命令行调试：

```bash
python KeyBoardControl.py
```

传送带距离测试：

```bash
python speed_test.py
```

## 八、使用前需要重点修改的配置

运行真实设备前，通常需要先改这些参数：

- 串口/IP：`DualCarriers.Settings.COM_LEFT`、`DualCarriers.Settings.COM_RIGHT`、`MutiInOneTime.sr`。
- 坐标点：各脚本 `Settings` 中的 `HOME_BASE`、`GET_BASE`、`PUT_BASE`、`COLOR_BASE`、`WASTE_POSE` 等。
- 速度/加速度：`speed`、`acc`、`DEFAULT_MOTO_SPEED`。
- 颜色传感器端口：`COLOR_PORT`。
- 电机端口：`MOTOR_PORT`。
- 泵/夹爪 I/O：`setPump(...)`、`setColotSensor(...)` 相关端口。
- 是否启用调试/HOME 初始化：`RIGHT_DEBUG`、`LEFT_DEBUG`、`HOME_INIT`。

## 九、注意事项

- 坐标值和端口值高度依赖现场硬件布局，不能直接照搬到另一套机械臂。
- `DobotAPI.OutPutFlag = False` 会关闭底层 API 的部分打印；调试底层通信时可以改成 `True`。
- `SetPTPCmdEx`、`SetIODOEx` 等 `Ex` 方法会等待队列执行到对应命令，适合按步骤执行动作；普通方法只负责下发命令。
- `DobotControl.run()` 会在 `finally` 中调用 `clean()`，异常退出时也会尝试关闭吸取并断开连接。
- 多机械臂运行时，动态库会被复制成带编号的文件分别加载，这是为了避免多个机械臂共享同一个 DLL 实例造成状态冲突。
- 部分注释和原 README 曾出现乱码，说明历史文件可能存在编码不一致问题。当前 README 使用 UTF-8 编写。

