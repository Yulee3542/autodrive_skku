import threading
import time

try:
    import serial
except ImportError:
    serial = None


class ArduinoNode:
    """아두이노 시리얼 링크. 프로토콜은 README '시리얼 프로토콜' 절 참고.

    조향은 전용 스티어링 모터의 120ms 펄스 방식이다: L/R 한 번 = 한 펄스만큼
    바퀴가 돌아가고 그 각도가 유지된다. steer()는 같은 값 연속 호출을 무시하고
    (검증된 main3 방식), 펄스를 반복해야 하는 기동은 steer_pulse()를 쓴다.

    펌웨어 워치독(500ms)이 통신 두절 시 차를 세우도록 수신 스레드가 200ms마다
    현재 속도를 keepalive로 재전송한다.
    """

    def __init__(self, port, baud=9600):
        self.state = None  # 0 정지 / 1 전진 / 2 후진
        self._ser = None
        self._speed = 0
        self._last = {}
        self._lock = threading.Lock()
        self._running = False

        if serial is None:
            print("[arduino] pyserial 미설치 — 차량 제어 없이 실행")
            return
        if port is None:
            print("[arduino] 포트를 찾지 못함 (--arduino 로 지정) — 차량 제어 없이 실행")
            return
        try:
            self._ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:
            print(f"[arduino] {port} 열기 실패: {e} — 차량 제어 없이 실행")
            self._ser = None
            return

        time.sleep(2)  # 보드가 시리얼 연결 시 리셋되므로 대기
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"[arduino] {port} 연결됨")

    def _loop(self):
        last_keepalive = 0.0
        while self._running:
            now = time.time()
            if now - last_keepalive >= 0.2:
                self._write(f"V{self._speed}\n")
                last_keepalive = now
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                time.sleep(0.05)
                continue
            if line in ("0", "1", "2"):
                self.state = int(line)

    def _write(self, text):
        if self._ser is None:
            return
        with self._lock:
            try:
                self._ser.write(text.encode("ascii"))
            except Exception:
                pass

    def _send_once(self, key, value):
        if self._last.get(key) == value:
            return
        self._write(value)
        self._last[key] = value

    def go(self):
        self._send_once("gate", "G")

    def drive(self, speed):
        """부호 있는 속도 지정. 음수 = 후진. 실제 전송은 keepalive가 담당."""
        self._speed = max(-255, min(255, int(speed)))

    def steer(self, direction):
        """조향 (같은 방향 연속 호출은 무시). F=조향 모터 정지, L/R=한 펄스."""
        self._send_once("steer", direction if direction in ("F", "L", "R") else "F")

    def steer_pulse(self, direction):
        """조향 펄스를 강제로 한 번 더 보낸다 (주차 등 반복 조향 기동용)."""
        if direction in ("F", "L", "R"):
            self._write(direction)
            self._last["steer"] = direction

    def stop(self):
        self._speed = 0
        self._write("V0\n")
        self._send_once("gate", "S")

    def close(self):
        self.stop()
        self._running = False
        time.sleep(0.1)
        if self._ser is not None:
            self._ser.close()
