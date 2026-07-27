#!/usr/bin/env python3
"""수동 조작(teleop) 노드 — 키보드로 모터/조향을 직접 몰면서 그 주행을 학습용
데이터셋으로 남긴다.

카메라 프레임은 mp4로, 프레임별 라벨은 같은 이름의 .jsonl 사이드카로 나간다:

    20260724_101530_front.mp4
    20260724_101530_front.jsonl

사이드카는 mp4에 실제로 기록된 프레임 한 장당 정확히 한 줄이고, "frame"이 mp4의
0-based 프레임 인덱스다. 즉 학습 코드는 i번째 프레임과 i번째 줄을 그대로 짝지으면
되고 인코딩 fps에 전혀 의존하지 않는다 — mp4는 config.TELEOP_RECORD_FPS라는 고정
명목 레이트로 인코딩되지만 "t"는 camera_node가 찍어준 진짜 촬영 시각이다.
세션 끝에는 {"summary": true, ...} 한 줄이 붙어 실측 fps를 남긴다(재생 속도를
맞추고 싶으면 ffmpeg -r <fps_measured>로 remux).

    {"frame": 0, "t": 1753280000.123, "cam": "front",
     "commands": {"drive": 80, "steer": "L", "go": true},
     "steering_pot": 412, "steering_angle_deg": null}

steering_angle_deg는 arduino_node가 고정 캘리브레이션(steering_adc_left/right)을
받았을 때만 발행되므로 없으면 null이다 — 그 경우에도 원시 ADC(steering_pot)는
남으므로 캘리브레이션이 끝난 뒤 후처리로 각도를 붙일 수 있다.

't' 키는 별도 기능이다 — odometry_node가 쓰는 config.ODOMETRY["pwm_to_mps"]를
실측하기 위해, 지정한 PWM으로 정확히 CALIB_DURATION_S초 주행하고 완전히 멈춘 뒤
사람이 줄자로 잰 거리를 받아 선형 적합까지 해준다 (2026-07-27). 이 차량에는 휠
인코더가 없어 코드가 스스로 잴 수 없는 값이라 사람 손이 반드시 한 번 필요하다.

camera_node.py와 같은 관례로 ROS/유닉스 전용 임포트는 전부 함수 안에서 한다 —
그래야 ROS 없는 개발 PC(윈도우 포함)에서도 셀프테스트가 돈다.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.teleop_node --selftest
"""
import datetime
import json
import os
import sys
import threading
import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from .. import config, drive_logger

SPEED_STEP = 20
SPEED_LIMIT = 255

# ---- 't' PWM 캘리브레이션 ----
# 측정 구간은 "정지 상태 → 완전 정지"다. arduino_node가 출발/정지 양쪽에 같은
# Cubic 램프(SPEED_RAMP_S)를 걸기 때문에 램프업에서 못 간 거리를 램프다운에서
# 정확히 되찾아, 명령 프로파일의 적분이 hold_s >= ramp_s인 한 항상 PWM*hold_s가
# 된다 (command_integral() 셀프테스트로 검증). 그래서 보정계수 없이
# v = 총이동거리 / CALIB_DURATION_S 를 평균속도로 쓸 수 있다.
CALIB_DURATION_S = 5.0
# stop 발행 후 차가 실제로 멈출 때까지의 대기. arduino_node의 램프 0.5초에
# keepalive 주기(200ms) 지연과 여유를 더한 값 — 이만큼 기다린 뒤에야 "재세요"라고
# 알려야 측정 구간이 위 적분과 일치한다. teleop이 arduino_node를 임포트하지
# 않도록(순환/시리얼 의존 회피) 여기 상수로 둔다.
CALIB_SETTLE_S = 0.8
CALIB_KIND = "pwm_calib"       # 누적 로그에서 이 기능의 줄만 골라내는 태그
CALIB_DEADZONE_WARN_PWM = 20   # 데드존이 이보다 크면 원점 통과 모델을 경고

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
  t : PWM 캘리브레이션 — 지정 PWM으로 5초 주행 후 실측거리를 넣어 pwm_to_mps 산출
  h : 이 도움말 다시 보기
  q : 종료 (Ctrl+C도 동작)

이 모드 동안 /camera/front, /camera/back을 mp4로 녹화하고, 프레임마다 그 시점의
조향/속도/게이트 + 조향 POT을 같은 이름의 .jsonl로 남깁니다 (학습용 데이터셋).
camera_node가 같이 떠 있어야 합니다 — bringup.launch.py run_mission:=false 등으로
먼저 기동해둘 것. 저장 위치는 config.TELEOP_RECORD_DIR.
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
    연다. mp4 자체는 config.TELEOP_RECORD_FPS 고정 레이트 인코딩이지만, 프레임과
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
                    self._path, config.TELEOP_RECORD_FPS, w, h)
                if self._log_path is not None:
                    self._logger = drive_logger.DriveLogger(self._log_path)
                print(f"[teleop] {self._name} 녹화 시작 -> {self._path}")
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
                print(f"[teleop] {self._name} 프레임을 한 장도 받지 못했습니다 "
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
                    "fps_encoded": config.TELEOP_RECORD_FPS,
                    "t_first": self._first_t,
                    "t_last": self._last_t,
                })
                self._logger.close()
                self._logger = None
            shown = "측정불가" if fps is None else f"{fps:.1f}"
            print(f"[teleop] {self._name} 녹화 종료 -> {self._path} "
                  f"({self._frames}프레임, 실측 {shown}fps / 인코딩 "
                  f"{config.TELEOP_RECORD_FPS:.1f}fps)")


CALIB_HELP = f"""
--- PWM 캘리브레이션 ({CALIB_DURATION_S:.0f}초 정속 주행) ---
odometry_node의 pwm_to_mps(= PWM 1당 m/s)를 실측합니다. 이 차량엔 휠 인코더가
없어서 거리는 사람이 줄자로 재야 합니다.

  1) 직선 구간에 차를 세우고 앞바퀴 위치를 바닥에 표시(테이프 등)
  2) 아래에서 PWM 값을 입력 -> 3초 카운트다운 후 정확히 {CALIB_DURATION_S:.0f}초 주행
  3) 차가 완전히 멈추면 표시한 지점부터 같은 기준점까지의 거리를 cm로 입력
  4) PWM을 바꿔 3점 이상(예: 60 / 100 / 150) 모으면 데드존까지 나옵니다

거리는 "출발 표시 -> 완전 정지" 전체를 재세요. 출발/정지 램프가 서로 정확히
상쇄되므로 그 전체 거리를 {CALIB_DURATION_S:.0f}초로 나눈 값이 정속 속도가 됩니다.
샘플은 세션이 바뀌어도 누적되니 며칠에 나눠 재도 됩니다.
"""


def load_calib_samples(path):
    """누적 캘리브 로그에서 이 기능이 남긴 줄만 읽어온다.

    파일이 없으면 [] — 첫 측정 세션도 그냥 빈 리스트로 시작하면 된다. 깨진 줄
    (중간에 죽어 잘린 마지막 줄 등)과 다른 스키마의 줄은 조용히 건너뛴다.
    """
    samples = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except OSError:
        return samples
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if isinstance(rec, dict) and rec.get("kind") == CALIB_KIND:
            samples.append(rec)
    return samples


def calib_speed_mps(sample):
    """샘플 한 개의 부호 있는 평균 속도(m/s). 값이 없거나 시간이 0 이하면 None."""
    dur = sample.get("duration_s")
    dist = sample.get("distance_m")
    if dist is None or not dur or dur <= 0:
        return None
    return dist / dur


def fit_pwm_to_mps(samples):
    """(PWM, 실측거리) 샘플들로 PWM->속도 선형 모델을 적합한다. 순수 함수.

    pwm_to_mps는 **원점을 지나는** 최소제곱 기울기 sum(p*v)/sum(p*p)다 —
    odometry_node.integrate_command_motion()이 v = speed_pwm * pwm_to_mps로 절편
    없는 스칼라 곱만 하므로, 파라미터에 넣을 값은 반드시 이 형태여야 한다.
    절편을 허용한 아핀 적합(slope/intercept/deadzone_pwm)은 진단용으로만 같이
    돌린다 — 데드존이 크면 저PWM 구간에서 원점 통과 모델이 실제보다 빠르게
    예측한다는 뜻이라 경고 근거가 된다.

    적합 불가(샘플 0개, 전부 PWM=0/무효)면 None.
    """
    pts = []
    for s in samples:
        v = calib_speed_mps(s)
        p = s.get("pwm")
        if v is None or not p:
            continue
        pts.append((float(p), v))
    if not pts:
        return None

    sum_pp = sum(p * p for p, _ in pts)
    if sum_pp <= 0:
        return None
    k = sum(p * v for p, v in pts) / sum_pp
    out = {
        "n": len(pts),
        "pwm_to_mps": k,
        "max_resid_mps": max(abs(v - k * p) for p, v in pts),
        "slope": None,
        "intercept": None,
        "deadzone_pwm": None,
        "r2": None,
    }

    n = len(pts)
    mean_p = sum(p for p, _ in pts) / n
    mean_v = sum(v for _, v in pts) / n
    var_p = sum((p - mean_p) ** 2 for p, _ in pts)
    if n < 2 or var_p <= 0:
        # 점이 하나뿐이거나 전부 같은 PWM — 기울기와 절편을 분리할 수 없다.
        return out
    slope = sum((p - mean_p) * (v - mean_v) for p, v in pts) / var_p
    intercept = mean_v - slope * mean_p
    out["slope"] = slope
    out["intercept"] = intercept
    if slope:
        out["deadzone_pwm"] = -intercept / slope
    ss_tot = sum((v - mean_v) ** 2 for _, v in pts)
    if ss_tot > 0:
        ss_res = sum((v - (slope * p + intercept)) ** 2 for p, v in pts)
        out["r2"] = 1.0 - ss_res / ss_tot
    return out


def format_calib_report(fit):
    """적합 결과를 사람이 읽는 여러 줄 문자열로. 순수 함수(셀프테스트 가능)."""
    if fit is None:
        return "[teleop] 아직 적합할 샘플이 없습니다 (거리를 입력한 주행이 1회 이상 필요)."
    lines = [
        f"[teleop] 샘플 {fit['n']}점 적합 결과",
        f"  pwm_to_mps = {fit['pwm_to_mps']:.6f}"
        f"  (원점 통과, 최대잔차 {fit['max_resid_mps']:.3f} m/s)",
    ]
    if fit["slope"] is None:
        lines.append("  점이 1개이거나 전부 같은 PWM — 데드존 분리 불가."
                     " PWM을 바꿔 한 점 더 재세요.")
    else:
        r2 = "" if fit["r2"] is None else f", R2={fit['r2']:.3f}"
        lines.append(f"  진단용 아핀: v = {fit['slope']:.6f}*PWM"
                     f" + {fit['intercept']:.4f}{r2}")
        dz = fit["deadzone_pwm"]
        if dz is not None:
            lines.append(f"  데드존(속도가 0이 되는 PWM) = {dz:.1f}")
            if abs(dz) >= CALIB_DEADZONE_WARN_PWM:
                lines.append(
                    f"  * 데드존이 큽니다 — |PWM| {abs(dz):.0f} 부근 저속에서는 원점 통과"
                    " 모델이 실제보다 빠르게 예측합니다. 실제 주행에 쓰는 PWM 대역"
                    " (config.DRIVE_SPEED 근처) 위주로 점을 모으세요.")
    lines.append("  ros2 param set /odometry_node odometry.pwm_to_mps "
                 f"{fit['pwm_to_mps']:.6f}")
    return "\n".join(lines)


def parse_calib_pwm(text, default):
    """PWM 입력 파싱. 빈칸이면 default. 반환 (pwm, 오류메시지) — 한쪽만 not None."""
    if text == "":
        pwm = default
    else:
        try:
            pwm = int(float(text))
        except ValueError:
            return None, "숫자를 입력하세요."
    if pwm == 0:
        return None, "PWM 0으로는 움직이지 않습니다 — 0이 아닌 값을 주세요."
    if abs(pwm) > SPEED_LIMIT:
        return None, f"PWM은 -{SPEED_LIMIT}..{SPEED_LIMIT} 범위여야 합니다."
    return pwm, None


def parse_calib_distance(text, pwm):
    """실측 거리 입력(cm) 파싱. 빈칸이면 (None, None) = 기록 건너뛰기.

    저장은 부호 있는 미터다 — 후진 주행(음수 PWM)은 음수 거리로 남겨야 전진/후진
    점이 원점 통과 직선 하나에 같이 올라간다. 사용자는 항상 이동한 길이를
    양수로 입력하고 부호는 여기서 붙인다.
    """
    if text == "":
        return None, None
    try:
        cm = float(text)
    except ValueError:
        return None, "숫자를 입력하세요 (cm)."
    if cm <= 0:
        return None, "이동 거리는 0보다 커야 합니다."
    return (cm / 100.0) * (1 if pwm > 0 else -1), None


def command_integral(pwm, hold_s, ramp_s, dt=0.001):
    """arduino_node가 실제로 내보내는 속도 명령 프로파일(램프업->정속->램프다운)의
    수치적분(PWM*초). 중점법.

    이 기능이 "총 이동거리 / hold_s"를 그대로 평균속도로 쓰는 근거를 셀프테스트에서
    검증하기 위한 것 — 램프가 양끝 대칭이라 hold_s >= ramp_s면 결과는 항상
    pwm*hold_s가 된다. 실제 램프 함수(arduino_node.cubic_trajectory)를 그대로
    가져다 쓰므로, 나중에 램프 방식이 바뀌면 셀프테스트가 먼저 깨진다.
    """
    from .arduino_node import cubic_trajectory

    total = 0.0
    steps = int(round((hold_s + ramp_s) / dt))
    for i in range(steps):
        t = (i + 0.5) * dt
        if t < ramp_s:
            v = cubic_trajectory(t, ramp_s, 0.0, pwm)
        elif t < hold_s:
            v = pwm
        else:
            v = cubic_trajectory(t - hold_s, ramp_s, pwm, 0.0)
        total += v * dt
    return total


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


def prompt_line(text):
    """한 줄 입력(Enter 필요). 취소/입력종료(Ctrl+C, Ctrl+D)면 None.

    read_key()는 읽는 그 순간에만 터미널을 raw로 바꿨다가 되돌리므로, 키 루프
    사이에서 이렇게 평범한 줄 입력을 받아도 된다.
    """
    try:
        return input(text).strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return None


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

    class TeleopNode(Node):
        """run_mission:=false로 띄운 상태에서 모터/조향을 수동으로 몰면서 그 주행을
        데이터셋으로 남기는 키보드 조작 도구(기존 tools/hw_test.py의 ROS 버전).
        모터 명령 발행은 run()의 블로킹 키 입력 루프가 담당하고, /camera/* 구독과
        조향 POT 구독(녹화/라벨용)은 main()이 별도 스레드에서 돌리는
        executor.spin()이 처리한다. mission_node와 마찬가지로 실제 stdin이 필요해
        'ros2 run'으로 직접 실행해야 한다 — ros2 launch는 자식 프로세스의 stdin을
        연결하지 않는다(ros2/launch#735)."""

        def __init__(self):
            super().__init__("teleop_node")
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

            # 't' PWM 캘리브레이션 누적 샘플. 고정 경로라 지난 세션 점들이 그대로
            # 살아있고, 새 점을 넣을 때마다 전부 합쳐 다시 적합한다.
            self._calib_path = config.PWM_CALIB_LOG
            self._calib_samples = load_calib_samples(self._calib_path)
            if self._calib_samples:
                print(f"[teleop] 기존 PWM 캘리브 샘플 {len(self._calib_samples)}점 로드"
                      f" ({self._calib_path})")

            self._recorders = []
            if cv2 is None:
                print("[teleop] opencv 미설치 — 카메라 녹화 생략(수동 조작은 그대로 동작)")
            else:
                paths = make_session_paths(config.TELEOP_RECORD_DIR)
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

        def _run_calibration(self):
            """지정 PWM으로 정확히 CALIB_DURATION_S초 주행하고 완전히 멈춘 뒤,
            사람이 잰 거리를 받아 pwm_to_mps를 적합한다.

            측정 구간이 "정지 -> 완전 정지"인 이유는 CALIB_DURATION_S 주석 참고
            (양끝 램프가 상쇄되어 보정계수가 필요 없다). 주행 중 카메라 녹화는
            평소처럼 계속 돌아가고, _labels()가 이 구간의 PWM을 그대로 라벨에
            남긴다.
            """
            print(CALIB_HELP)
            default_pwm = self._speed or config.DRIVE_SPEED
            while True:
                text = prompt_line(f"PWM 값 (c=취소) [{default_pwm}]: ")
                if text is None or text.lower() == "c":
                    print("[teleop] 캘리브레이션 취소")
                    return
                pwm, err = parse_calib_pwm(text, default_pwm)
                if err is None:
                    break
                print(f"  {err}")

            # tools/hw_test.py와 같은 확인 관문 — 실제로 모터가 도는 기능이다.
            confirm = prompt_line(
                f"PWM {pwm}으로 {CALIB_DURATION_S:.0f}초 주행합니다."
                " 직선 경로가 비어 있고 출발 위치를 표시했나요? (y/N): ")
            if confirm is None or confirm.lower() != "y":
                print("[teleop] 캘리브레이션 취소")
                return
            for i in (3, 2, 1):
                print(f"  {i}...")
                time.sleep(1.0)

            aborted = False
            t0 = time.monotonic()
            try:
                # go가 drive보다 먼저여야 한다 — 펌웨어 canGo 게이트가 닫혀 있으면
                # 속도값을 받아도 구동은 0으로 처리된다.
                self._go_pub.publish(Empty())
                self._went_go = True
                self._speed = pwm
                t0 = time.monotonic()
                self._drive_pub.publish(Int16(data=pwm))
                print(f"[teleop] 주행 시작 (PWM {pwm})")
                time.sleep(CALIB_DURATION_S)
            except KeyboardInterrupt:
                # raw 모드가 아닌 구간이라 Ctrl+C가 실제 예외로 들어온다. 여기서
                # 삼켜 키 루프로 되돌아간다 — 정지는 아래 finally가 이미 보낸다.
                aborted = True
            finally:
                held_s = time.monotonic() - t0
                self._stop_pub.publish(Empty())
                self._speed = 0
                self._went_go = False
                self._last_steer = "F"
                print(f"[teleop] 정지 명령 발행 — 정지 대기 {CALIB_SETTLE_S:.1f}초")
                time.sleep(CALIB_SETTLE_S)

            if aborted:
                print(f"[teleop] 중단됨 ({held_s:.2f}초 주행) — 기록하지 않습니다")
                return

            print(f"[teleop] 주행 완료 (실제 홀드 {held_s:.2f}초)."
                  " 출발 표시부터 지금 위치까지 같은 기준점으로 재세요.")
            while True:
                text = prompt_line("실측 이동거리 cm (빈칸=기록 안 함): ")
                dist_m, err = parse_calib_distance("" if text is None else text, pwm)
                if err is None:
                    break
                print(f"  {err}")
            if dist_m is None:
                print("[teleop] 거리 미입력 — 이번 주행은 기록하지 않습니다")
                return

            record = {
                "t": time.time(),
                "kind": CALIB_KIND,
                "pwm": pwm,
                "distance_m": dist_m,
                "duration_s": held_s,
                "speed_mps": dist_m / held_s,
            }
            try:
                os.makedirs(os.path.dirname(self._calib_path), exist_ok=True)
                logger = drive_logger.DriveLogger(self._calib_path)
                logger.log_raw(record)
                logger.close()
                print(f"[teleop] 기록 -> {self._calib_path}")
            except OSError as e:
                print(f"[teleop] 캘리브 로그 기록 실패({e})"
                      " — 이번 점은 이 세션에서만 유효합니다")
            self._calib_samples.append(record)
            print(format_calib_report(fit_pwm_to_mps(self._calib_samples)))

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
                    elif key == "t":
                        self._run_calibration()
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
        print("[X ] opencv 미설치 — teleop 셀프테스트 불가")
        return 1
    import shutil
    import tempfile

    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    tmp_dir = tempfile.mkdtemp(prefix="teleop_selftest_")
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
              and summary[0]["fps_encoded"] == config.TELEOP_RECORD_FPS)

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

        # ================= 't' PWM 캘리브레이션 =================

        # ---- 이 기능의 전제: 양끝 램프가 상쇄되어 유효 주행시간 = 홀드 시간 ----
        # 실제 램프 함수(arduino_node.cubic_trajectory)를 그대로 적분하므로,
        # 램프 방식이 바뀌면 여기서 먼저 깨진다.
        from .arduino_node import SPEED_RAMP_S
        area = command_integral(100, CALIB_DURATION_S, SPEED_RAMP_S)
        check("램프업/램프다운이 정확히 상쇄 — 명령 적분 = PWM*홀드시간",
              abs(area - 100 * CALIB_DURATION_S) < 0.01)
        check("램프 시간이 달라져도 상쇄는 유지 (홀드 >= 램프인 한)",
              abs(command_integral(80, 5.0, 1.5) - 80 * 5.0) < 0.01
              and abs(command_integral(80, 5.0, 0.0) - 80 * 5.0) < 0.01)

        # ---- fit_pwm_to_mps ----
        def sample(pwm, dist_m, dur=CALIB_DURATION_S):
            return {"kind": CALIB_KIND, "pwm": pwm,
                    "distance_m": dist_m, "duration_s": dur}

        check("샘플 0개면 적합 결과 None", fit_pwm_to_mps([]) is None)
        check("PWM=0 샘플만 있으면 적합 불가(None)",
              fit_pwm_to_mps([sample(0, 0.0)]) is None)

        # v = 0.01*PWM 인 완벽한 원점 통과 데이터 (5초 * 0.01 * PWM 미터)
        exact = [sample(p, 0.01 * p * CALIB_DURATION_S) for p in (60, 100, 150)]
        fit = fit_pwm_to_mps(exact)
        check("원점 통과 데이터에서 pwm_to_mps를 정확히 복원",
              abs(fit["pwm_to_mps"] - 0.01) < 1e-9 and fit["n"] == 3)
        check("잔차가 0이고 데드존도 0으로 나옴",
              fit["max_resid_mps"] < 1e-9 and abs(fit["deadzone_pwm"]) < 1e-6
              and abs(fit["r2"] - 1.0) < 1e-9)

        one = fit_pwm_to_mps([sample(100, 5.0)])
        check("점이 1개면 원점 통과 값만 나오고 데드존은 None",
              abs(one["pwm_to_mps"] - 0.01) < 1e-9 and one["slope"] is None
              and one["deadzone_pwm"] is None)
        check("같은 PWM만 여러 점이어도 데드존 분리 불가(None)",
              fit_pwm_to_mps([sample(100, 5.0), sample(100, 5.2)])["slope"] is None)

        # 데드존이 있는 데이터: v = 0.01*(PWM - 30) -> 데드존 30
        dz = [sample(p, 0.01 * (p - 30) * CALIB_DURATION_S) for p in (60, 100, 150)]
        fit_dz = fit_pwm_to_mps(dz)
        check("절편 있는 데이터에서 데드존 PWM을 복원",
              abs(fit_dz["deadzone_pwm"] - 30.0) < 1e-6)
        check("데드존이 있으면 원점 통과 기울기는 실제 기울기보다 작다",
              fit_dz["pwm_to_mps"] < fit_dz["slope"])

        # 후진(음수 PWM) 샘플은 음수 거리로 저장되어 같은 직선에 올라간다
        both = [sample(100, 5.0), sample(-100, -5.0)]
        check("전진/후진 샘플이 부호까지 맞게 같은 기울기로 적합",
              abs(fit_pwm_to_mps(both)["pwm_to_mps"] - 0.01) < 1e-9)

        # 시간이 0/음수인 깨진 샘플은 무시
        check("duration_s가 0 이하인 샘플은 무시",
              calib_speed_mps(sample(100, 5.0, 0)) is None
              and fit_pwm_to_mps([sample(100, 5.0, 0)] + exact)["n"] == 3)

        # ---- 입력 파싱 ----
        check("PWM 입력: 빈칸이면 기본값, 숫자면 그 값",
              parse_calib_pwm("", 100) == (100, None)
              and parse_calib_pwm("150", 100) == (150, None)
              and parse_calib_pwm("-80", 100) == (-80, None))
        check("PWM 입력: 0/범위초과/비숫자는 거부",
              parse_calib_pwm("0", 100)[0] is None
              and parse_calib_pwm("300", 100)[0] is None
              and parse_calib_pwm("abc", 100)[0] is None)
        check("거리 입력: cm -> m 변환, 부호는 PWM을 따라감",
              parse_calib_distance("250", 100) == (2.5, None)
              and parse_calib_distance("250", -100) == (-2.5, None))
        check("거리 입력: 빈칸은 기록 건너뛰기, 0/음수/비숫자는 거부",
              parse_calib_distance("", 100) == (None, None)
              and parse_calib_distance("0", 100)[1] is not None
              and parse_calib_distance("x", 100)[1] is not None)

        # ---- 누적 로그 로드 ----
        calib_path = os.path.join(tmp_dir, "pwm_calib.jsonl")
        check("로그 파일이 없으면 빈 리스트", load_calib_samples(calib_path) == [])
        logger = drive_logger.DriveLogger(calib_path)
        for s in exact:
            logger.log_raw(dict(s, t=1753280000.0))
        # 다른 스키마의 줄(teleop 프레임 라벨 등)이 섞여도 골라내야 한다
        logger.log_raw({"frame": 0, "t": 1.0, "cam": "front"})
        logger.close()
        with open(calib_path, "a", encoding="utf-8") as f:
            f.write('{"kind": "pwm_calib", "pwm": 1')  # 중간에 죽어 잘린 줄
        loaded = load_calib_samples(calib_path)
        check("누적 로그에서 캘리브 줄만 골라 읽음(다른 스키마/깨진 줄 무시)",
              len(loaded) == 3 and all(r["kind"] == CALIB_KIND for r in loaded))
        check("로드한 샘플로 다시 적합해도 같은 결과(세션 간 누적)",
              abs(fit_pwm_to_mps(loaded)["pwm_to_mps"] - 0.01) < 1e-9)

        # ---- 리포트 문자열 ----
        report = format_calib_report(fit_dz)
        check("리포트에 ros2 param set 명령과 데드존 경고가 포함됨",
              "ros2 param set /odometry_node odometry.pwm_to_mps" in report
              and "데드존" in report and "데드존이 큽니다" in report)
        check("샘플이 없으면 리포트가 안내 문구를 반환",
              "샘플이 없습니다" in format_calib_report(None))
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
