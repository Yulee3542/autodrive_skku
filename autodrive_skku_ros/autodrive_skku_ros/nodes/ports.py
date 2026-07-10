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
