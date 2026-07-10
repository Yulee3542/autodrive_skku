import os
import sys

try:
    import fcntl
    import termios
    import tty
except ImportError:  # Windows 등 — 실제 실행 환경(Linux/WSL2)에서는 항상 존재
    fcntl = termios = tty = None

from .base import Mission

SPEED_STEP = 20
SPEED_LIMIT = 255

HELP = """
[test 미션] 키보드 텔레옵 — 키를 누르면 즉시 반영됩니다 (Enter 불필요).
  ※ w/x로 속도를 줘도 먼저 g(주행 허용)를 안 보내면 차가 안 움직입니다
    (펌웨어 워치독 게이트 — s를 누르면 다시 닫히므로 그 다음엔 g부터).
  g : go (주행 허용, 반드시 먼저)   w/x : 속도 +20/-20   space : 속도 0
  a : 좌 조향 펄스 (L)              d : 우 조향 펄스 (R)  f : 조향 중립 (F)
  s : stop (즉시 정지, 게이트도 닫힘)   h : 도움말 다시 보기
"""


class TestMission(Mission):
    """자동주행 로직 없음 — 대신 stdin을 논블로킹으로 읽어 이 미션 자체가 키보드
    텔레옵 조종 인터페이스를 겸한다(별도 teleop_node 없이 mission:=test 선택 자체가
    곧 수동 조작 모드).

    mission_node를 'ros2 run'으로 직접 실행했을 때만 stdin이 실제 터미널에 연결된다
    — 'ros2 launch'는 자식 프로세스의 stdin을 연결하지 않는 launch 시스템 자체의
    알려진 제약(ros2/launch#735)이라, ros2 launch로 mission:=test를 띄우면 이
    미션은 그냥 아무 것도 안 하는 미션이 된다(자동주행이 없다는 점만 보장). 실제
    키보드 조종은 다음처럼 두 터미널로 나눠서 써야 한다:
      터미널1: ros2 launch autodrive_skku_ros bringup.launch.py run_mission:=false
      터미널2: ros2 run autodrive_skku_ros mission_node --ros-args -p mission:=test
    """

    name = "test"

    def on_start(self, car, config):
        self._speed = 0
        self._went_go = False  # 펌웨어 canGo 게이트 미러 — g 전송 전엔 속도를 줘도 안 움직임
        self._stdin_ready = False
        if termios is None:
            print("[test] termios/tty/fcntl 미지원 플랫폼 — 키보드 조종 비활성화 "
                  "(자동주행은 하지 않으니 ros2 topic pub/Foxglove Publish 패널/"
                  "teleop_node로 계속 조종 가능합니다).")
            return
        if not sys.stdin.isatty():
            print("[test] stdin이 tty가 아닙니다 — 키보드 조종은 "
                  "'ros2 run autodrive_skku_ros mission_node'로 직접 실행했을 "
                  "때만 동작합니다 (ros2 launch는 자식 프로세스 stdin을 안 붙여줌, "
                  "ros2/launch#735). 자동주행은 하지 않으니 다른 방법(ros2 topic "
                  "pub, Foxglove Publish 패널, teleop_node)으로 계속 조종 가능합니다.")
            return
        self._fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)  # 캐노니컬 모드 해제 — Enter 없이 한 글자씩 즉시 read
        flags = fcntl.fcntl(self._fd, fcntl.F_GETFL)
        fcntl.fcntl(self._fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self._stdin_ready = True
        print(HELP)

    def step(self, sensors, car):
        if not self._stdin_ready:
            return
        try:
            key = sys.stdin.read(1)
        except (IOError, OSError):
            return  # 논블로킹 read인데 입력이 없으면 여기로 빠짐 — 정상
        if key:
            self._handle_key(key, car)

    def _handle_key(self, key, car):
        if key == "g":
            car.go()
            self._went_go = True
            print("go")
        elif key == "w":
            self._set_speed(self._speed + SPEED_STEP, car)
        elif key == "x":
            self._set_speed(self._speed - SPEED_STEP, car)
        elif key == " ":
            self._set_speed(0, car)
        elif key == "a":
            car.steer_pulse("L")
            print("steer L")
        elif key == "d":
            car.steer_pulse("R")
            print("steer R")
        elif key == "f":
            car.steer("F")
            print("steer F")
        elif key == "s":
            self._speed = 0
            self._went_go = False
            car.stop()
            print("stop")
        elif key == "h":
            print(HELP)

    def _set_speed(self, speed, car):
        self._speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, speed))
        car.drive(self._speed)
        note = "" if self._went_go else " (아직 g 안 보냄 — 실제로는 안 움직입니다)"
        print(f"speed={self._speed}{note}")

    def on_stop(self, car):
        if self._stdin_ready:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)
        car.stop()
