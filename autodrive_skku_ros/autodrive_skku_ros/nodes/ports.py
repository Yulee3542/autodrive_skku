#!/usr/bin/env python3
"""시리얼 포트 자동 감지 + launch 시작 전 좀비/스테일 상태 정리.

ROS 노드가 아니라 순수 헬퍼 함수뿐이라 ros_main()이 없다. bringup.launch.py가
launch-description 생성 시점에 직접 import해서 쓰고, arduino_node.py/hw_test.py도
그대로 재사용한다.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.ports --selftest
"""
import glob
import os
import subprocess

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None


def autodetect_ports():
    """연결된 시리얼 포트에서 (아두이노, 라이다) 추정. 못 찾으면 None.

    아두이노 메가는 보통 Arduino/CH340/ttyACM, RPLidar는 CP210x(Silicon Labs)로 잡힌다.
    둘 다 CP210x로 잡히는 보드 조합이면 구분이 불가능하므로 config.py나
    --arduino/--lidar 인자로 직접 지정할 것.
    """
    if list_ports is None:
        return None, None

    ports = list(list_ports.comports())
    if ports:
        print("[ports] 감지된 시리얼 포트:")
        for p in ports:
            print(f"  {p.device}: {p.description}")

    def desc(p):
        return f"{p.description or ''} {p.manufacturer or ''}".lower()

    arduino = None
    for p in ports:
        if "arduino" in desc(p) or "ch340" in desc(p) or "acm" in p.device.lower():
            arduino = p.device
            break

    lidar = None
    for p in ports:
        if p.device == arduino:
            continue
        if "cp210" in desc(p) or "silicon labs" in desc(p):
            lidar = p.device
            break

    return arduino, lidar


# 우리 launch가 띄우는 노드 실행파일 이름 — 비정상 종료(Qt abort, rplidar
# buffer overflow 등 SIGABRT류) 후 좀비로 남을 수 있는 것들. rplidar_composition/
# foxglove_bridge는 서드파티 실행파일이라 이름으로만 매칭.
STALE_PROCESS_NAMES = (
    "arduino_node", "camera_node", "mission_node", "odometry_node", "lidar_node",
    "rplidar_composition", "foxglove_bridge",
)
# FastDDS 공유메모리 락 파일 — 비정상 종료로 못 지워지면 다음 실행에서
# "Failed init_port fastrtps_port<N>" 에러 + 노드 디스커버리 실패로 이어진다
# (2026-07-17 실차에서 반복 확인).
STALE_SHM_GLOB = "/dev/shm/*fastrtps*"


def cleanup_stale_ros_state(names=STALE_PROCESS_NAMES, shm_glob=STALE_SHM_GLOB,
                             run=subprocess.run, glob_fn=glob.glob, remove=os.remove):
    """launch 시작 전 이전 실행의 잔재를 정리한다.

    이전 실행이 크래시(SIGABRT 등)로 죽으면 자식 프로세스가 좀비로 남거나
    FastDDS 공유메모리 락 파일을 못 지운다 — 다음 launch가 그 상태를 물려받아
    노드 디스커버리가 절반만 되는 등 원인 파악이 어려운 증상으로 이어진다.
    bringup.launch.py가 generate_launch_description() 시작 시 매번 호출한다.

    best-effort: run()/remove()가 예외를 던져도(권한 없음, pkill 미설치,
    비-Linux 플랫폼 등) 조용히 무시하고 launch 자체를 막지 않는다.

    반환: (killed_names, removed_paths) — 로그/테스트용.
    """
    killed = []
    for name in names:
        try:
            result = run(["pkill", "-9", "-f", name],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if getattr(result, "returncode", 1) == 0:
                killed.append(name)
        except Exception:
            pass

    removed = []
    try:
        paths = glob_fn(shm_glob)
    except Exception:
        paths = []
    for path in paths:
        try:
            remove(path)
            removed.append(path)
        except Exception:
            pass

    return killed, removed


# ========================= 오프라인 테스트 / 셀프테스트 =========================

class _FakePort:
    """serial.tools.list_ports.comports()가 반환하는 ListPortInfo 흉내."""

    def __init__(self, device, description="", manufacturer=""):
        self.device = device
        self.description = description
        self.manufacturer = manufacturer


def selftest():
    if list_ports is None:
        print("[X ] pyserial 미설치 — 포트 셀프테스트 불가")
        return 1

    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    def with_ports(fake_ports, fn):
        original = list_ports.comports
        list_ports.comports = lambda: fake_ports
        try:
            return fn()
        finally:
            list_ports.comports = original

    arduino, lidar = with_ports(
        [_FakePort("/dev/ttyACM0", "Arduino Mega", "Arduino"),
         _FakePort("/dev/ttyUSB0", "CP2102 USB to UART", "Silicon Labs")],
        autodetect_ports)
    check("Arduino(ACM) + CP210x(라이다) 조합 정상 인식",
          arduino == "/dev/ttyACM0" and lidar == "/dev/ttyUSB0")

    arduino, lidar = with_ports(
        [_FakePort("/dev/ttyUSB0", "CH340 serial converter")],
        autodetect_ports)
    check("CH340(설명 문자열)만 있으면 아두이노로 인식, 라이다는 None",
          arduino == "/dev/ttyUSB0" and lidar is None)

    arduino, lidar = with_ports([], autodetect_ports)
    check("포트 없으면 (None, None)", arduino is None and lidar is None)

    arduino, lidar = with_ports(
        [_FakePort("/dev/ttyACM0", "Arduino Mega", "Arduino"),
         _FakePort("/dev/ttyACM1", "CP2102 USB to UART", "Silicon Labs")],
        autodetect_ports)
    check("아두이노로 이미 잡힌 포트는 라이다 후보에서 제외",
          arduino == "/dev/ttyACM0" and lidar == "/dev/ttyACM1")

    # ---- cleanup_stale_ros_state (launch 시작 전 좀비/스테일 SHM 정리) ----
    pkill_calls = []

    def fake_run(cmd, **_kwargs):
        pkill_calls.append(cmd)
        class _Result:
            returncode = 0
        return _Result()

    removed_paths = []

    def fake_remove(path):
        removed_paths.append(path)

    killed, removed = cleanup_stale_ros_state(
        names=("arduino_node", "mission_node"),
        shm_glob="/dev/shm/*fastrtps*",
        run=fake_run,
        glob_fn=lambda pattern: ["/dev/shm/fastrtps_port7923", "/dev/shm/sem.fastrtps_x"],
        remove=fake_remove)
    check("등록된 프로세스 이름마다 pkill -9 -f 호출",
          pkill_calls == [["pkill", "-9", "-f", "arduino_node"],
                          ["pkill", "-9", "-f", "mission_node"]])
    check("pkill 성공(returncode=0)한 이름이 killed에 기록됨",
          killed == ["arduino_node", "mission_node"])
    check("SHM 글롭 매치 파일 전부 제거 시도 + 반환값 일치",
          removed == ["/dev/shm/fastrtps_port7923", "/dev/shm/sem.fastrtps_x"]
          and removed_paths == removed)

    def raising_run(_cmd, **_kwargs):
        raise OSError("pkill 없음(플랫폼 미지원 등)")

    killed2, removed2 = cleanup_stale_ros_state(
        names=("arduino_node",), run=raising_run,
        glob_fn=lambda _p: (_ for _ in ()).throw(OSError("no /dev/shm")),
        remove=fake_remove)
    check("run()/glob_fn()이 예외를 던져도 크래시 없이 빈 결과로 계속 진행",
          killed2 == [] and removed2 == [])

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    sys.exit(selftest())
