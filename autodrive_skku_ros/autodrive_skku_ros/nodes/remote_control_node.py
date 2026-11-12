#!/usr/bin/env python3
"""원격 조종(remote control) 노드 — 키보드로 모터/조향을 직접 몰면서 그 주행을 학습용
데이터셋으로 남긴다.

카메라 프레임은 mp4로, 프레임별 라벨은 같은 이름의 .jsonl 사이드카로 나간다:

    20260724_101530_front.mp4
    20260724_101530_front.jsonl

사이드카는 mp4에 실제로 기록된 프레임 한 장당 정확히 한 줄이고, "frame"이 mp4의
0-based 프레임 인덱스다. 즉 학습 코드는 i번째 프레임과 i번째 줄을 그대로 짝지으면
되고 인코딩 fps에 전혀 의존하지 않는다 — mp4는 config.REMOTE_CONTROL_RECORD_FPS라는 고정
명목 레이트로 인코딩되지만 "t"는 camera_node가 찍어준 진짜 촬영 시각이다.
세션 끝에는 {"summary": true, ...} 한 줄이 붙어 실측 fps를 남긴다(재생 속도를
맞추고 싶으면 ffmpeg -r <fps_measured>로 remux).

    {"frame": 0, "t": 1753280000.123, "cam": "front",
     "commands": {"drive": 80, "steer": "L", "go": true},
     "steering_pot": 412, "steering_angle_deg": null}

steering_angle_deg는 arduino_node가 고정 캘리브레이션(steering_adc_left/right)을
받았을 때만 발행되므로 없으면 null이다 — 그 경우에도 원시 ADC(steering_pot)는
남으므로 캘리브레이션이 끝난 뒤 후처리로 각도를 붙일 수 있다.

camera_node.py와 같은 관례로 ROS/유닉스 전용 임포트는 전부 함수 안에서 한다 —
그래야 ROS 없는 개발 PC(윈도우 포함)에서도 셀프테스트가 돈다.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.remote_control_node --selftest
"""
import datetime
import os
import sys
import threading

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from .. import config, drive_logger

SPEED_STEP = 20
SPEED_LIMIT = 255

HELP = """
원격 조종 모드 (모터/조향 동작 확인용) — 키를 누르면 즉시 반영됩니다 (Enter 불필요).
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

이 모드 동안 /camera/front, /camera/back을 mp4로 녹화하고, 프레임마다 그 시점의
조향/속도/게이트 + 조향 POT을 같은 이름의 .jsonl로 남깁니다 (학습용 데이터셋).
camera_node가 같이 떠 있어야 합니다 — bringup.launch.py run_mission:=false 등으로
먼저 기동해둘 것. 저장 위치는 config.REMOTE_CONTROL_RECORD_DIR.
"""


def stamp_to_sec(stamp):
    """builtin_interfaces/Time -> float 초. camera_node가 _publish_frame()에서
    찍어준 실제 촬영 시각이라, 녹화 스레드가 프레임을 언제 처리했는지와 무관하다."""
    return stamp.sec + stamp.nanosec * 1e-9


def measured_fps(frames, first_t, last_t):
    """실제로 받은 프레임 수/시각으로 계산한 평균 fps. 간격이 n-1개뿐이므로
    분자는 frames-1이다. 계산 불가(프레임 1장 이하, 시각 역행/동일)면 None."""
    if frames < 2 or first_t is None or last_t is None or last_t <= first_t:
        return None
    return (frames - 1) / (last_t - first_t)


def _open_video_writer(path, fps, width, height):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    return cv2.VideoWriter(path, fourcc, fps, (width, height))


class _CameraRecorder:
    """CompressedImage(jpeg) 구독 콜백에서 받은 프레임을 mp4 + 라벨 jsonl로 기록.

    첫 프레임이 와야 해상도를 알 수 있으므로 VideoWriter(와 사이드카 로거)는 그때
    연다. mp4 자체는 config.REMOTE_CONTROL_RECORD_FPS 고정 레이트 인코딩이지만, 프레임과
    라벨의 대응은 인덱스로 잡으므로 그 명목 fps가 틀려도 데이터셋은 정확하다.

    ROS 비의존 — msg는 .data와 .header.stamp만 읽는 덕타이핑이라 셀프테스트에서
    가짜 메시지를 그대로 넣을 수 있다.
    """

    def __init__(self, name, out_path, label_fn=None, log_path=None, writer_factory=None):
        self._name = name
        self._path = out_path
        self._log_path = log_path
        self._label_fn = label_fn
        self._writer_factory = writer_factory or _open_video_writer
        self._writer = None
        self._logger = None
        self._frames = 0
        self._first_t = None
        self._last_t = None
        self._closed = False
        self._lock = threading.Lock()

    @property
    def frames(self):
        return self._frames

    def on_frame(self, msg):
        if cv2 is None:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        t = stamp_to_sec(msg.header.stamp)
        with self._lock:
            if self._closed:
                # close() 이후 executor 스레드가 늦게 배달한 프레임. 여기서 막지
                # 않으면 _writer가 None이라 아래에서 같은 경로에 VideoWriter를 새로
                # 열어 방금 끝낸 녹화를 통째로 덮어쓴다(main()이 node.run()의
                # finally에서 close()를 부른 뒤에야 executor.shutdown()을 하므로
                # 이 창은 실제로 열려 있다).
                return
            if self._writer is None:
                h, w = frame.shape[:2]
                self._writer = self._writer_factory(
                    self._path, config.REMOTE_CONTROL_RECORD_FPS, w, h)
                if self._log_path is not None:
                    self._logger = drive_logger.DriveLogger(self._log_path)
                print(f"[remote_control] {self._name} 녹화 시작 -> {self._path}")
            self._writer.write(frame)
            if self._logger is not None:
                record = {"frame": self._frames, "t": t, "cam": self._name}
                if self._label_fn is not None:
                    record.update(self._label_fn())
                self._logger.log_raw(record)
            self._frames += 1
            if self._first_t is None:
                self._first_t = t
            self._last_t = t

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if self._writer is None:
                # 구독은 걸렸는데 프레임이 0장 — 예전엔 조용히 아무 파일도 안 남아
                # 나중에야 알아챘다. camera_node 미기동이 거의 항상 원인.
                print(f"[remote_control] {self._name} 프레임을 한 장도 받지 못했습니다 "
                      f"— camera_node가 떠 있는지 확인하세요 (녹화 파일 없음)")
                return
            self._writer.release()
            self._writer = None
            fps = measured_fps(self._frames, self._first_t, self._last_t)
            if self._logger is not None:
                self._logger.log_raw({
                    "summary": True,
                    "cam": self._name,
                    "frames": self._frames,
                    "fps_measured": fps,
                    "fps_encoded": config.REMOTE_CONTROL_RECORD_FPS,
                    "t_first": self._first_t,
                    "t_last": self._last_t,
                })
                self._logger.close()
                self._logger = None
            shown = "측정불가" if fps is None else f"{fps:.1f}"
            print(f"[remote_control] {self._name} 녹화 종료 -> {self._path} "
                  f"({self._frames}프레임, 실측 {shown}fps / 인코딩 "
                  f"{config.REMOTE_CONTROL_RECORD_FPS:.1f}fps)")


def read_key():
    """터미널을 raw 모드로 바꿔 Enter 없이 키 하나를 읽고 원래대로 복원한다."""
    import termios  # 유닉스 전용 — 모듈 임포트 시점에 끌어오면 윈도우에서 셀프테스트 불가
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def make_session_paths(record_dir, stamp=None):
    """한 세션의 mp4/jsonl 경로를 카메라별로 만든다. mp4와 사이드카가 반드시 같은
    stamp를 쓰도록 여기서 한 번에 만든다 — 짝이 어긋나면 데이터셋이 못 쓰게 된다."""
    os.makedirs(record_dir, exist_ok=True)
    if stamp is None:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return {
        name: (os.path.join(record_dir, f"{stamp}_{name}.mp4"),
               os.path.join(record_dir, f"{stamp}_{name}.jsonl"))
        for name in ("front", "back")
    }


# ============================ ROS2 래퍼 ============================

def main(args=None):
    import signal

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Empty, Float32, Int16, Int32, String

    class RemoteControlNode(Node):
        """run_mission:=false로 띄운 상태에서 모터/조향을 수동으로 몰면서 그 주행을
        데이터셋으로 남기는 키보드 조작 도구(기존 tools/hw_test.py의 ROS 버전).
        모터 명령 발행은 run()의 블로킹 키 입력 루프가 담당하고, /camera/* 구독과
        조향 POT 구독(녹화/라벨용)은 main()이 별도 스레드에서 돌리는
        executor.spin()이 처리한다. mission_node와 마찬가지로 실제 stdin이 필요해
        'ros2 run'으로 직접 실행해야 한다 — ros2 launch는 자식 프로세스의 stdin을
        연결하지 않는다(ros2/launch#735)."""

        def __init__(self):
            super().__init__("remote_control_node")
            self._go_pub = self.create_publisher(Empty, "/car/cmd/go", 10)
            self._stop_pub = self.create_publisher(Empty, "/car/cmd/stop", 10)
            self._drive_pub = self.create_publisher(Int16, "/car/cmd/drive", 10)
            self._steer_pub = self.create_publisher(String, "/car/cmd/steer", 10)
            self._steer_pulse_pub = self.create_publisher(String, "/car/cmd/steer_pulse", 10)
            self._speed = 0
            self._went_go = False  # 펌웨어 canGo 게이트 미러 — g 전송 전엔 속도를 줘도 안 움직임
            # 프레임 라벨에 "그 순간의 마지막 조향 명령"을 남기기 위한 추적
            # (mission_node._last_steer와 같은 목적/같은 이름).
            self._last_steer = "F"

            # 조향 실측값. steering_angle은 arduino_node가 고정 캘리브레이션을 받은
            # 경우에만 발행하므로 안 올 수 있고, 그때는 라벨에 null로 남는다.
            self._pot_adc = None
            self._steer_deg = None
            self.create_subscription(Int32, "/car/steering_pot", self._on_pot, 10)
            self.create_subscription(Float32, "/car/steering_angle", self._on_angle, 10)

            self._recorders = []
            if cv2 is None:
                print("[remote_control] opencv 미설치 — 카메라 녹화 생략(수동 조작은 그대로 동작)")
            else:
                paths = make_session_paths(config.REMOTE_CONTROL_RECORD_DIR)
                for name, topic in (("front", "/camera/front"), ("back", "/camera/back")):
                    mp4_path, log_path = paths[name]
                    rec = _CameraRecorder(name, mp4_path, label_fn=self._labels,
                                          log_path=log_path)
                    self._recorders.append(rec)
                    self.create_subscription(CompressedImage, topic, rec.on_frame, 10)

        def _on_pot(self, msg):
            self._pot_adc = msg.data

        def _on_angle(self, msg):
            self._steer_deg = msg.data

        def _labels(self):
            """녹화 콜백이 프레임마다 호출 — 그 프레임 시점의 제어 상태 스냅샷."""
            return {
                "commands": {
                    "drive": self._speed,
                    "steer": self._last_steer,
                    "go": self._went_go,
                },
                "steering_pot": self._pot_adc,
                "steering_angle_deg": self._steer_deg,
            }

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
                        self._last_steer = "L"
                        print("steer L")
                    elif key == "d":
                        self._steer_pulse_pub.publish(String(data="R"))
                        self._last_steer = "R"
                        print("steer R")
                    elif key == "f":
                        self._steer_pub.publish(String(data="F"))
                        self._last_steer = "F"
                        print("steer F")
                    elif key == "s":
                        self._speed = 0
                        self._went_go = False
                        self._last_steer = "F"
                        self._stop_pub.publish(Empty())
                        print("stop")
                    elif key == "h":
                        print(HELP)
            finally:
                # 종료 경로(q, Ctrl+C, SIGTERM)와 무관하게 모터에 정지 신호를 남긴다 —
                # 안 그러면 마지막으로 준 속도로 차가 계속 움직인 채 프로그램만 끝난다.
                self._stop_pub.publish(Empty())
                print("[remote_control] 종료 — 정지 명령 발행")
                for rec in self._recorders:
                    rec.close()

    def _on_sigterm(_signum, _frame):
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _on_sigterm)

    rclpy.init(args=args)
    node = RemoteControlNode()
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


# ========================= 오프라인 테스트 / 셀프테스트 =========================

class _FakeStamp:
    def __init__(self, t):
        self.sec = int(t)
        self.nanosec = int(round((t - int(t)) * 1e9))


class _FakeHeader:
    def __init__(self, t):
        self.stamp = _FakeStamp(t)


class _FakeMsg:
    """CompressedImage 흉내 — _CameraRecorder는 .data와 .header.stamp만 본다."""

    def __init__(self, jpeg_bytes, t):
        self.data = jpeg_bytes
        self.header = _FakeHeader(t)


class _StubWriter:
    """cv2.VideoWriter 대역 — 실제 mp4를 만들지 않고 write/release 횟수만 센다."""

    def __init__(self):
        self.written = 0
        self.released = 0

    def write(self, _frame):
        self.written += 1

    def release(self):
        self.released += 1


def selftest():
    """실제 카메라/ROS/mp4 인코더 없이 _CameraRecorder의 프레임↔라벨 정합,
    타임스탬프 출처, close() 이후 재오픈 방지를 검증한다."""
    if cv2 is None:
        print("[X ] opencv 미설치 — remote_control 셀프테스트 불가")
        return 1
    import json
    import shutil
    import tempfile

    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    tmp_dir = tempfile.mkdtemp(prefix="remote_control_selftest_")
    try:
        # ---- make_session_paths: mp4와 사이드카가 같은 stamp를 공유 ----
        rec_dir = os.path.join(tmp_dir, "recordings")
        paths = make_session_paths(rec_dir, stamp="20260724_101530")
        front_mp4, front_log = paths["front"]
        check("make_session_paths: 디렉토리를 미리 생성함", os.path.isdir(rec_dir))
        check("make_session_paths: mp4와 jsonl이 같은 stamp/이름을 공유",
              os.path.basename(front_mp4) == "20260724_101530_front.mp4"
              and os.path.basename(front_log) == "20260724_101530_front.jsonl")

        # ---- measured_fps 순수 함수 ----
        check("measured_fps: 간격은 n-1개 (11프레임/1초 -> 10fps)",
              abs(measured_fps(11, 100.0, 101.0) - 10.0) < 1e-9)
        check("measured_fps: 프레임 1장 이하/시각 역행이면 None",
              measured_fps(1, 100.0, 100.0) is None
              and measured_fps(5, 100.0, 99.0) is None
              and measured_fps(5, None, None) is None)

        # ---- 라벨이 바뀌는 시퀀스를 흘려보내고 사이드카를 검증 ----
        ok, buf = cv2.imencode(".jpg", np.zeros((8, 12, 3), dtype=np.uint8))
        jpeg = buf.tobytes()
        check("셀프테스트용 jpeg 인코딩 성공", ok)

        labels = {"commands": {"drive": 0, "steer": "F", "go": False},
                  "steering_pot": None, "steering_angle_deg": None}
        stub = _StubWriter()
        rec = _CameraRecorder("front", front_mp4, label_fn=lambda: dict(labels),
                              log_path=front_log, writer_factory=lambda *a: stub)

        rec.on_frame(_FakeMsg(jpeg, 1000.0))
        labels = {"commands": {"drive": 80, "steer": "L", "go": True},
                  "steering_pot": 412, "steering_angle_deg": None}
        rec.on_frame(_FakeMsg(jpeg, 1000.5))
        labels = {"commands": {"drive": 80, "steer": "R", "go": True},
                  "steering_pot": 430, "steering_angle_deg": -3.5}
        rec.on_frame(_FakeMsg(jpeg, 1001.0))
        rec.close()

        with open(front_log, "r", encoding="utf-8") as f:
            lines = [json.loads(line) for line in f if line.strip()]
        data = [ln for ln in lines if not ln.get("summary")]
        summary = [ln for ln in lines if ln.get("summary")]

        check("프레임 3장 -> mp4 write 3회", stub.written == 3)
        check("프레임 수와 데이터 줄 수가 1:1", len(data) == stub.written == rec.frames)
        check("frame 인덱스가 0부터 빠짐없이 증가(mp4 인덱스와 동일)",
              [ln["frame"] for ln in data] == [0, 1, 2])
        check("t는 헤더 스탬프에서 옴(벽시계가 아님)",
              [ln["t"] for ln in data] == [1000.0, 1000.5, 1001.0])
        check("라벨이 프레임 시점의 값으로 각각 캡처됨",
              [ln["commands"]["steer"] for ln in data] == ["F", "L", "R"]
              and [ln["commands"]["drive"] for ln in data] == [0, 80, 80]
              and [ln["commands"]["go"] for ln in data] == [False, True, True])
        check("steering_pot이 그대로 남음(미수신 구간은 null)",
              [ln["steering_pot"] for ln in data] == [None, 412, 430])
        check("steering_angle_deg는 캘리브 전이면 null, 오면 값",
              data[0]["steering_angle_deg"] is None
              and data[2]["steering_angle_deg"] == -3.5)
        check("cam 필드로 전/후방 구분 가능",
              all(ln["cam"] == "front" for ln in data))
        check("close()가 VideoWriter를 release함", stub.released == 1)
        check("종료 요약 줄에 프레임 수와 실측 fps가 남음",
              len(summary) == 1 and summary[0]["frames"] == 3
              and abs(summary[0]["fps_measured"] - 2.0) < 1e-9
              and summary[0]["fps_encoded"] == config.REMOTE_CONTROL_RECORD_FPS)

        # ---- 회귀: close() 이후 늦게 도착한 프레임이 녹화를 덮어쓰지 않아야 함 ----
        # (executor는 daemon 스레드라 node.run()의 finally가 close()를 부른 뒤에도
        #  프레임을 배달할 수 있다.)
        reopened = []

        def _tripwire_factory(*_a):
            reopened.append(True)
            return _StubWriter()

        rec._writer_factory = _tripwire_factory
        rec.on_frame(_FakeMsg(jpeg, 1001.5))
        with open(front_log, "r", encoding="utf-8") as f:
            after = [json.loads(line) for line in f if line.strip()]
        check("close() 후 늦은 프레임이 VideoWriter를 재오픈하지 않음(파일 덮어쓰기 방지)",
              not reopened)
        check("close() 후 늦은 프레임이 사이드카에 줄을 추가하지 않음",
              len(after) == len(lines))
        check("close() 재호출이 안전함(release 중복 없음)",
              (rec.close() is None) and stub.released == 1)

        # ---- 프레임 0장으로 끝난 세션: 파일 없이 경고만 ----
        empty_mp4, empty_log = paths["back"]
        empty = _CameraRecorder("back", empty_mp4, log_path=empty_log,
                                writer_factory=lambda *a: _StubWriter())
        empty.close()
        check("프레임 0장이면 mp4/jsonl을 만들지 않고 경고만 (camera_node 미기동)",
              not os.path.exists(empty_mp4) and not os.path.exists(empty_log))
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
