#!/usr/bin/env python3
"""실행 중인 시스템을 단계별로 진단한다 (nodes doctor).

check_env.py가 "설치돼 있고 장치가 보이는가"(정적)를 본다면, 이 스크립트는
"지금 실제로 살아서 돌고 있는가"(런타임)를 본다 — README의 '디버깅 사다리'를
자동화한 것이다. 주행이 안 될 때 어느 단계에서 끊겼는지 위에서부터 좁혀준다.

원칙:
  * **읽기 전용/비파괴** — 모터를 절대 움직이지 않는다(그건 tools/hw_test.py 담당).
  * **돌고 있는 시스템과 싸우지 않는다** — 아두이노/카메라를 노드가 이미 점유
    중이면 뺏어오지 않고 "노드가 사용 중"으로 보고한다. 오히려 점유돼 있다는
    사실 자체가 노드가 살아 있다는 증거다.
  * ROS가 없는 환경(개발용 Windows 등)에서도 죽지 않는다 — 해당 검사는 SKIP.

사용법:
    python3 tools/doctor.py              # 전체
    python3 tools/doctor.py --timeout 5  # 토픽 대기 시간(기본 3초)
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _TOOLS)
sys.path.insert(0, os.path.join(os.path.dirname(_TOOLS), "autodrive_skku_ros"))

PASS, FAIL, WARN, SKIP = "OK", "X ", "! ", "- "

# launch의 executable 이름과 실제 ROS 노드 이름이 다르다 — 헷갈리기 쉬운 지점이라
# 양쪽을 같이 들고 다니며 보고한다 (예: arduino_node 실행 → /arduino_bridge_node).
EXPECTED_NODES = [
    ("/arduino_bridge_node", "arduino_node", "구동/조향 시리얼 브릿지"),
    ("/camera_publisher_node", "camera_node", "카메라 캡처/발행"),
    ("/mission_node", "mission_node", "미션 오케스트레이터"),
    ("/lidar_geometry_node", "lidar_node", "라이다 지오메트리(선택: run_lidar)"),
    ("/odometry_node", "odometry_node", "오도메트리(선택: run_odometry)"),
]

# (토픽, 필수 여부, 설명) — 필수가 아니면 없어도 WARN까지만
TOPICS = [
    ("/camera/front", True, "전방 카메라 — 차선/신호등 전부 이거에 의존"),
    ("/car/state", True, "아두이노 상태 (0정지/1전진/2후진)"),
    ("/car/cmd/drive", False, "미션이 내리는 속도 명령 (run_mission=false면 없음)"),
    ("/scan", False, "라이다 원시 스캔 (run_lidar=false면 없음)"),
    ("/camera/back", False, "후방 카메라 (REAR_CAMERA 설정 시에만)"),
]

_results = []


def report(status, name, detail=""):
    print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))
    _results.append((status, name))
    return status == PASS


def section(title):
    print(f"\n== {title} ==")


def _run(cmd, timeout=10):
    """(rc, stdout). 실행 자체가 불가하면 (None, '')."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout
    except Exception:
        return None, ""


def _have_ros():
    return shutil.which("ros2") is not None


# ---------------- 1. 정적 환경 ----------------

def check_static():
    section("1. 정적 환경 (check_env.py 위임)")
    try:
        import check_env
    except Exception as e:
        return report(FAIL, "check_env 임포트", str(e))
    ok = check_env.check_imports()
    return report(PASS if ok else WARN, "python 패키지",
                  "" if ok else "위 [X] 항목 참고 — rclpy 없으면 아래 ROS 검사는 전부 SKIP")


# ---------------- 2. 디스크 ----------------

def check_disk(min_free_gb=2.0):
    section("2. 디스크 여유")
    ok = True
    try:
        usage = shutil.disk_usage(os.path.expanduser("~"))
        free_gb = usage.free / 1e9
        ok = free_gb >= min_free_gb
        report(PASS if ok else FAIL, f"여유 공간 {free_gb:.1f} GB",
               "" if ok else f"{min_free_gb} GB 미만 — 로그/빌드가 실패할 수 있음")
    except Exception as e:
        report(WARN, "디스크 조회 실패", str(e))

    # YOLO 설치 시 흔한 함정: CUDA 휠 4~6GB (이 차량엔 NVIDIA GPU가 없어 전부 낭비).
    # 2026-07 팀 실사례 — 원인을 ROS2 재설치로 오해해 워크스페이스를 지웠지만
    # 용량은 ~/.local에 있어서 해결되지 않았다. README 'YOLO 설치 시 저장공간' 참고.
    nvidia_dirs = glob.glob(os.path.expanduser(
        "~/.local/lib/python3*/site-packages/nvidia"))
    if not nvidia_dirs:
        report(PASS, "NVIDIA CUDA 휠 없음", "CPU 전용 torch거나 YOLO 미설치 — 정상")
    else:
        total = 0
        for d in nvidia_dirs:
            for root, _dirs, files in os.walk(d):
                for f in files:
                    try:
                        total += os.path.getsize(os.path.join(root, f))
                    except OSError:
                        pass
        report(WARN, f"NVIDIA CUDA 휠 {total / 1e9:.1f} GB 발견",
               "GPU가 없는 차량이면 낭비 — README 'YOLO 설치 시 저장공간'대로 "
               "CPU 전용 torch로 교체하면 회수 가능")
    return ok


# ---------------- 3. ROS 그래프 ----------------

def check_ros_graph():
    section("3. ROS 노드 생존")
    if not _have_ros():
        return report(SKIP, "ros2 명령 없음",
                      "source /opt/ros/<distro>/setup.bash 후 재실행")
    rc, out = _run(["ros2", "node", "list"])
    if rc is None:
        return report(FAIL, "ros2 node list 실행 실패")
    alive = {ln.strip() for ln in out.splitlines() if ln.strip()}
    if not alive:
        return report(FAIL, "실행 중인 노드 없음",
                      "bringup.launch.py가 떠 있는지 확인")
    ok = True
    for node, exe, desc in EXPECTED_NODES:
        if node in alive:
            report(PASS, node, desc)
        else:
            optional = "선택" in desc
            report(WARN if optional else FAIL, f"{node} 없음",
                   f"{desc} — launch의 executable 이름은 '{exe}' (노드 이름과 다름)")
            ok = ok and optional
    extra = alive - {n for n, _e, _d in EXPECTED_NODES}
    if extra:
        report(SKIP, "기타 노드", ", ".join(sorted(extra)))
    return ok


# ---------------- 4. 토픽 생존 ----------------

def check_topics(timeout):
    section(f"4. 토픽 생존 (각 {timeout}초 대기)")
    if not _have_ros():
        return report(SKIP, "ros2 명령 없음")
    rc, out = _run(["ros2", "topic", "list"])
    if rc is None:
        return report(FAIL, "ros2 topic list 실행 실패")
    listed = {ln.strip() for ln in out.splitlines() if ln.strip()}

    ok = True
    for topic, required, desc in TOPICS:
        if topic not in listed:
            report(FAIL if required else SKIP, f"{topic} 없음", desc)
            ok = ok and not required
            continue
        # 존재해도 발행이 멈췄을 수 있다 — 실제로 한 장 받아본다.
        t0 = time.time()
        rc2, _o = _run(["ros2", "topic", "echo", topic, "--once"], timeout=timeout)
        dt = time.time() - t0
        if rc2 == 0:
            report(PASS, f"{topic} 수신", f"{dt:.2f}s — {desc}")
        else:
            report(FAIL if required else WARN, f"{topic} 무발행",
                   f"{timeout}s 내 메시지 없음 (토픽은 존재) — 발행 노드가 멈췄거나 "
                   f"센서가 끊김. {desc}")
            ok = ok and not required
    return ok


# ---------------- 5. 아두이노 ----------------

def check_arduino():
    section("5. 아두이노 시리얼")
    try:
        from serial.tools import list_ports
        import serial
    except ImportError:
        return report(SKIP, "pyserial 미설치")
    ports = [p for p in list_ports.comports()
             if "ACM" in p.device or "USB" in p.device or "COM" in p.device]
    if not ports:
        return report(FAIL, "시리얼 포트 없음", "아두이노 미연결 / WSL2면 usbipd attach")
    ok = False
    for p in ports:
        try:
            with serial.Serial(p.device, 9600, timeout=1.5) as ser:
                line = ser.readline().decode(errors="replace").strip()
            # 펌웨어는 상태(0/1/2)와 'P <adc>'를 계속 흘린다
            if line and (line[:1] in "012" or line.startswith("P ")):
                ok = report(PASS, f"{p.device} 응답", f"수신: {line!r}")
            else:
                report(WARN, f"{p.device} 무응답/불명",
                       f"수신: {line!r} — 펌웨어 미업로드 또는 다른 장치일 수 있음")
        except Exception as e:
            msg = str(e)
            if "busy" in msg.lower() or "Access is denied" in msg or "PermissionError" in msg:
                # 점유 자체가 arduino_node가 살아 있다는 증거 — 뺏지 않는다.
                ok = report(PASS, f"{p.device} 사용 중",
                            "arduino_node가 점유 중으로 보임 (정상). 뺏지 않고 넘어감")
            else:
                report(WARN, f"{p.device} 열기 실패", msg)
    return ok


# ---------------- 6. 카메라 ----------------

def check_camera():
    section("6. 카메라 스트림")
    try:
        import cv2
        from autodrive_skku_ros import config
    except Exception as e:
        return report(SKIP, "cv2/config 임포트 실패", str(e))

    if not sys.platform.startswith("linux"):
        report(SKIP, "장치 열기", "Linux가 아니므로 /dev/video 검사 생략")
        return True

    devs = sorted(glob.glob("/dev/video*"))
    if not devs:
        return report(FAIL, "/dev/video* 없음", "카메라 미연결 / WSL2면 usbipd attach")

    # 이름을 먼저 보여준다 — 어느 인덱스가 C920인지가 흔한 혼동 지점
    for d in devs:
        n = d.rsplit("video", 1)[-1]
        try:
            with open(f"/sys/class/video4linux/video{n}/name") as f:
                report(SKIP, d, f.read().strip())
        except OSError:
            pass

    idx = config.FRONT_CAMERA if config.FRONT_CAMERA is not None else 0
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        cap.release()
        return report(PASS, f"index {idx} 열기 실패",
                      "camera_node가 점유 중이면 정상 (뺏지 않음). "
                      "노드가 안 떠 있는데도 실패면 tools/pick_camera.py로 인덱스 확인")
    got, frame = cap.read()
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    cap.release()
    if not got or frame is None:
        return report(FAIL, f"index {idx} 프레임 없음", "장치는 열리는데 캡처 실패")
    match = (int(w), int(h)) == (config.FRAME_WIDTH, config.FRAME_HEIGHT)
    return report(PASS if match else WARN, f"index {idx} 캡처 {int(w)}x{int(h)}",
                  "" if match else
                  f"config는 {config.FRAME_WIDTH}x{config.FRAME_HEIGHT} — 드라이버가 "
                  "요청 해상도를 못 맞춤(성능/화각 영향)")


# ---------------- 7. 라이다 ----------------

def check_lidar(timeout):
    section("7. 라이다 스캔 내용")
    if not _have_ros():
        return report(SKIP, "ros2 명령 없음")
    rc, out = _run(["ros2", "topic", "echo", "/scan", "--once",
                    "--field", "ranges"], timeout=timeout)
    if rc != 0 or not out.strip():
        return report(WARN, "/scan 수신 실패",
                      "run_lidar:=false면 정상. 아니면 rplidar 전원/포트 확인")
    # 전부 inf/0이면 스핀은 하는데 아무것도 못 보는 상태
    finite = sum(1 for tok in out.replace("[", " ").replace("]", " ").split(",")
                 if tok.strip().replace(".", "").replace("-", "").isdigit())
    return report(PASS if finite else WARN, f"유효 거리값 {finite}개",
                  "" if finite else "전부 inf/0 — 라이다가 돌지만 반사를 못 받음")


# ---------------- 8. 미션/오버레이 ----------------

def check_mission(timeout):
    section("8. 미션 상태 / 디버그 오버레이")
    if not _have_ros():
        return report(SKIP, "ros2 명령 없음")
    rc, out = _run(["ros2", "param", "get", "/mission_node", "mission"], timeout=timeout)
    if rc == 0 and out.strip():
        report(PASS, "활성 미션", out.strip())
    else:
        report(WARN, "미션 파라미터 조회 실패", "mission_node 미실행(run_mission:=false)일 수 있음")
    rc2, out2 = _run(["ros2", "topic", "list"])
    overlays = [t for t in out2.splitlines() if t.strip().startswith("/debug/")]
    return report(PASS if overlays else WARN,
                  f"/debug/* 오버레이 {len(overlays)}개",
                  ", ".join(overlays) if overlays else
                  "미션이 아직 프레임을 처리하지 않았거나 debug.overlay=false")


def main():
    ap = argparse.ArgumentParser(description="실행 중인 시스템 단계별 진단 (비파괴)")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="토픽 한 건 수신 대기 시간(초, 기본 3)")
    args = ap.parse_args()

    print("=== autodrive_skku nodes doctor ===")
    print("읽기 전용 — 모터를 움직이지 않습니다 (구동 테스트는 tools/hw_test.py)")

    check_static()
    check_disk()
    check_ros_graph()
    check_topics(args.timeout)
    check_arduino()
    check_camera()
    check_lidar(args.timeout)
    check_mission(args.timeout)

    fails = [n for s, n in _results if s == FAIL]
    warns = [n for s, n in _results if s == WARN]
    print("\n==== 요약 ====")
    print(f"  통과 {sum(1 for s, _ in _results if s == PASS)} / "
          f"경고 {len(warns)} / 실패 {len(fails)}")
    if fails:
        print("  먼저 볼 것(위에서부터): " + ", ".join(fails[:4]))
    print("\n결과:", "이상 없음" if not fails else "위 [X ] 항목부터 해결")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
