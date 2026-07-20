import datetime
import os
import sys
import termios
import threading
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty, Int16, String

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from .. import config

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

이 모드 동안 /camera/front, /camera/back을 자동으로 mp4 녹화합니다
(camera_node가 같이 떠 있어야 함 — bringup.launch.py run_mission:=false 등으로
먼저 기동해둘 것). 저장 위치는 config.TELEOP_RECORD_DIR.
"""


class _CameraRecorder:
    """CompressedImage(jpeg) 구독 콜백에서 받은 프레임을 mp4로 순차 기록.

    첫 프레임이 와야 실제 해상도를 알 수 있으므로 VideoWriter는 그때 연다.
    실제 프레임 수신 간격과 무관하게 config.TELEOP_RECORD_FPS로 인코딩하는
    근사치 녹화다(teleop 디버그용 — 프레임 드롭/지연이 있어도 재생 속도만
    달라질 뿐 동작에는 문제 없음).
    """

    def __init__(self, name, out_path):
        self._name = name
        self._path = out_path
        self._writer = None
        self._lock = threading.Lock()

    def on_frame(self, msg):
        if cv2 is None:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._lock:
            if self._writer is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self._writer = cv2.VideoWriter(self._path, fourcc, config.TELEOP_RECORD_FPS, (w, h))
                print(f"[teleop] {self._name} 녹화 시작 -> {self._path}")
            self._writer.write(frame)

    def close(self):
        with self._lock:
            if self._writer is not None:
                self._writer.release()
                print(f"[teleop] {self._name} 녹화 종료 -> {self._path}")
                self._writer = None


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
    도구(기존 tools/hw_test.py의 ROS 버전). 모터 명령 발행 자체는 run()의 블로킹
    키 입력 루프가 담당하고, /camera/front, /camera/back 구독(자동 녹화용)은
    main()이 별도 스레드에서 돌리는 executor.spin()이 처리한다. mission_node와
    마찬가지로 실제 stdin이 필요해 'ros2 run'으로 직접 실행해야 한다 — ros2
    launch는 자식 프로세스의 stdin을 연결하지 않는다(ros2/launch#735)."""

    def __init__(self):
        super().__init__("teleop_node")
        self._go_pub = self.create_publisher(Empty, "/car/cmd/go", 10)
        self._stop_pub = self.create_publisher(Empty, "/car/cmd/stop", 10)
        self._drive_pub = self.create_publisher(Int16, "/car/cmd/drive", 10)
        self._steer_pub = self.create_publisher(String, "/car/cmd/steer", 10)
        self._steer_pulse_pub = self.create_publisher(String, "/car/cmd/steer_pulse", 10)
        self._speed = 0
        self._went_go = False  # 펌웨어 canGo 게이트 미러 — g 전송 전엔 속도를 줘도 안 움직임

        self._recorders = []
        if cv2 is None:
            print("[teleop] opencv 미설치 — 카메라 녹화 생략(수동 조작은 그대로 동작)")
        else:
            os.makedirs(config.TELEOP_RECORD_DIR, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            front_rec = _CameraRecorder(
                "front", os.path.join(config.TELEOP_RECORD_DIR, f"{stamp}_front.mp4"))
            back_rec = _CameraRecorder(
                "back", os.path.join(config.TELEOP_RECORD_DIR, f"{stamp}_back.mp4"))
            self._recorders = [front_rec, back_rec]
            self.create_subscription(CompressedImage, "/camera/front", front_rec.on_frame, 10)
            self.create_subscription(CompressedImage, "/camera/back", back_rec.on_frame, 10)

    def _set_speed(self, speed):
        self._speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, speed))
        self._drive_pub.publish(Int16(data=self._speed))
        note = "" if self._went_go else " (아직 g 안 보냄 — 실제로는 안 움직입니다)"
        print(f"speed={self._speed}{note}")

    def run(self):
        print(HELP)
        try:
            while rclpy.ok():
                key = read_key()
                if key in ("q", "\x03"):
                    # raw 터미널 모드라 Ctrl+C(\x03)는 SIGINT가 아니라 그냥 문자로
                    # 들어온다 — KeyboardInterrupt 예외가 안 나므로 여기서 직접
                    # break해야 하고, 정지 발행은 finally가 담당한다.
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
                    self._steer_pulse_pub.publish(String(data="L"))
                    print("steer L")
                elif key == "d":
                    self._steer_pulse_pub.publish(String(data="R"))
                    print("steer R")
                elif key == "f":
                    self._steer_pub.publish(String(data="F"))
                    print("steer F")
                elif key == "s":
                    self._speed = 0
                    self._went_go = False
                    self._stop_pub.publish(Empty())
                    print("stop")
                elif key == "h":
                    print(HELP)
        finally:
            # 종료 경로(q, Ctrl+C, SIGTERM)와 무관하게 모터에 정지 신호를 남긴다 —
            # 안 그러면 마지막으로 준 속도로 차가 계속 움직인 채 프로그램만 끝난다.
            self._stop_pub.publish(Empty())
            print("[teleop] 종료 — 정지 명령 발행")
            for rec in self._recorders:
                rec.close()


def _on_sigterm(_signum, _frame):
    raise SystemExit(0)


def main(args=None):
    import signal
    signal.signal(signal.SIGTERM, _on_sigterm)

    rclpy.init(args=args)
    node = TeleopNode()
    # run()은 read_key()로 stdin을 블로킹 읽으므로, 카메라 구독 콜백(녹화)이
    # 동작하려면 별도 스레드에서 spin해야 한다 — 메인 스레드는 키 입력에 전념.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
