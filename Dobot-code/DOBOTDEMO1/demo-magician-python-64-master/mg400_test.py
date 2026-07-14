import argparse
import socket
import sys
import time


CON_STR = {
    0: "NoError",
    -1: "ConnectionFailed",
    -2: "CommandFailed",
}


class DobotTcpClient:
    def __init__(self, ip, port, timeout=5):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.socket_dobot = None

    def connect(self):
        self.socket_dobot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_dobot.settimeout(self.timeout)
        self.socket_dobot.connect((self.ip, self.port))

    def close(self):
        if self.socket_dobot is not None:
            self.socket_dobot.close()
            self.socket_dobot = None

    def send_command(self, command, wait_response=True):
        if self.socket_dobot is None:
            raise RuntimeError("TCP socket is not connected")

        # MG400 commands are plain ASCII strings such as EnableRobot().
        self.socket_dobot.send(command.encode("utf-8"))
        if not wait_response:
            return ""
        return self.socket_dobot.recv(1024).decode("utf-8", errors="replace").strip()


class MG400Dashboard(DobotTcpClient):
    def clear_error(self):
        return self.send_command("ClearError()")

    def enable_robot(self):
        return self.send_command("EnableRobot()")

    def disable_robot(self):
        return self.send_command("DisableRobot()")

    def robot_mode(self):
        return self.send_command("RobotMode()")

    def get_pose(self):
        return self.send_command("GetPose()")


class MG400Move(DobotTcpClient):
    def movj(self, x, y, z, r):
        return self.send_command("MovJ({},{},{},{})".format(x, y, z, r))

    def movl(self, x, y, z, r):
        return self.send_command("MovL({},{},{},{})".format(x, y, z, r))

    def sync(self):
        return self.send_command("Sync()")


def connect_port(ip, port, name, timeout):
    client = DobotTcpClient(ip, port, timeout)
    print("正在连接 {} 端口 {} ...".format(name, port))
    client.connect()
    print("{} 连接成功".format(name))
    return client


def run_connection_test(ip, timeout, enable, move_test):
    dashboard = MG400Dashboard(ip, 29999, timeout)
    move = MG400Move(ip, 30003, timeout)

    try:
        print("正在建立 MG400 TCP 连接...")
        dashboard.connect()
        print("控制端口 29999 连接成功")

        move.connect()
        print("运动端口 30003 连接成功")

        print("RobotMode:", dashboard.robot_mode())
        print("GetPose:", dashboard.get_pose())

        if enable:
            print("ClearError:", dashboard.clear_error())
            time.sleep(0.2)
            print("EnableRobot:", dashboard.enable_robot())
            time.sleep(0.5)
            print("RobotMode:", dashboard.robot_mode())

        if move_test:
            if not enable:
                print("已跳过运动测试：执行运动前请同时加 --enable。")
            else:
                print("开始小范围运动测试，请确认机械臂周围安全。")
                print("MovJ:", move.movj(200, 0, 50, 0))
                print("Sync:", move.sync())
                print("MovJ:", move.movj(220, 0, 50, 0))
                print("Sync:", move.sync())
                print("MovJ:", move.movj(200, 0, 50, 0))
                print("Sync:", move.sync())

        print("MG400 连接测试完成:", CON_STR[0])
        return 0
    except OSError as exc:
        print("MG400 连接失败: {} ({})".format(CON_STR[-1], exc))
        return 1
    except Exception as exc:
        print("MG400 测试失败: {} ({})".format(CON_STR[-2], exc))
        return 1
    finally:
        move.close()
        dashboard.close()
        print("TCP 连接已关闭")


def parse_args():
    parser = argparse.ArgumentParser(description="MG400 TCP/IP connection test")
    parser.add_argument(
        "--ip",
        default="192.168.1.6",
        help="MG400 IP address, LAN1 default is 192.168.1.6",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5,
        help="socket timeout seconds",
    )
    parser.add_argument(
        "--enable",
        action="store_true",
        help="clear error and enable robot after connection",
    )
    parser.add_argument(
        "--move-test",
        action="store_true",
        help="run a small MovJ test; requires --enable",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    sys.exit(run_connection_test(args.ip, args.timeout, args.enable, args.move_test))
