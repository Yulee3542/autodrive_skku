#!/usr/bin/env python3
"""시리얼 포트 자동 감지 — 아두이노/라이다를 장치 설명 문자열로 구분한다.

ROS 노드가 아니라 순수 헬퍼 함수 하나뿐이라 ros_main()이 없다. bringup.launch.py가
launch-description 생성 시점에 직접 import해서 쓰고, arduino_node.py/hw_test.py도
그대로 재사용한다.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.ports --selftest
"""
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

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    sys.exit(selftest())
