import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from autodrive_msgs.msg import DriveCmd, SteerCmd

SPEED_STEP = 20
SPEED_LIMIT = 255

HELP = """
수동 조작 모드 (모터/조향 동작 확인용) — 키를 누르면 즉시 반영됩니다 (Enter 불필요).
  ※ w/x로 속도를 줘도 먼저 g(주행 허용)를 안 보내면 차가 안 움직입니다
    (펌웨어 워치독 게이트 — s를 누르면 다시 닫히므로 그 다음엔 g부터).
  g : go (주행 허용, 반드시 먼저)
  w : 속도 +20 (전진 방향, 음수면 후진)
  x : 속도 -20
  space : 속도 0
  a : 좌 조향 펄스 (L)
  d : 우 조향 펄스 (R)
  f : 조향 중립 (F)
  s : stop (즉시 정지, 게이트도 닫힘)
  h : 이 도움말 다시 보기
  q : 종료 (Ctrl+C도 동작)
"""


def read_key():
    """터미널을 raw 모드로 바꿔 Enter 없이 키 하나를 읽고 원래대로 복원한다."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TeleopNode(Node):
    """run_mission:=false로 띄운 상태에서 모터/조향을 수동으로 확인하는 키보드 조작
    도구(기존 tools/hw_test.py의 ROS 버전). 발행만 하므로 rclpy.spin() 없이 블로킹
    키 입력 루프로 동작한다. mission_node와 마찬가지로 실제 stdin이 필요해
    'ros2 run'으로 직접 실행해야 한다 — ros2 launch는 자식 프로세스의 stdin을
    연결하지 않는다(ros2/launch#735)."""

    def __init__(self):
        super().__init__("teleop_node")
        self._go_pub = self.create_publisher(Empty, "/car/cmd/go", 10)
        self._stop_pub = self.create_publisher(Empty, "/car/cmd/stop", 10)
        self._drive_pub = self.create_publisher(DriveCmd, "/car/cmd/drive", 10)
        self._steer_pub = self.create_publisher(SteerCmd, "/car/cmd/steer", 10)
        self._speed = 0
        self._went_go = False  # 펌웨어 canGo 게이트 미러 — g 전송 전엔 속도를 줘도 안 움직임

    def _set_speed(self, speed):
        self._speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, speed))
        self._drive_pub.publish(DriveCmd(speed=self._speed))
        note = "" if self._went_go else " (아직 g 안 보냄 — 실제로는 안 움직입니다)"
        print(f"speed={self._speed}{note}")

    def run(self):
        print(HELP)
        while rclpy.ok():
            key = read_key()
            if key in ("q", "\x03"):
                break
            elif key == "g":
                self._go_pub.publish(Empty())
                self._went_go = True
                print("go")
            elif key == "w":
                self._set_speed(self._speed + SPEED_STEP)
            elif key == "x":
                self._set_speed(self._speed - SPEED_STEP)
            elif key == " ":
                self._set_speed(0)
            elif key == "a":
                self._steer_pub.publish(SteerCmd(direction="L", pulse=True))
                print("steer L")
            elif key == "d":
                self._steer_pub.publish(SteerCmd(direction="R", pulse=True))
                print("steer R")
            elif key == "f":
                self._steer_pub.publish(SteerCmd(direction="F", pulse=False))
                print("steer F")
            elif key == "s":
                self._speed = 0
                self._went_go = False
                self._stop_pub.publish(Empty())
                print("stop")
            elif key == "h":
                print(HELP)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
