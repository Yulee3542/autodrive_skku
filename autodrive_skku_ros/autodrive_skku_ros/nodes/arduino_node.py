#!/usr/bin/env python3
"""아두이노 시리얼 브릿지 — 차량 구동/조향 명령을 시리얼로 중계한다.

ArduinoNode: 시리얼 포트를 직접 소유하는 순수 파이썬 클래스 (ROS 비의존, 워치독
keepalive 스레드 포함). ros_main()의 ArduinoBridgeNode가 이 클래스를 얇게
감싸 /car/cmd/go, /car/cmd/stop, /car/cmd/drive(Int16), /car/cmd/steer(String,
dedup), /car/cmd/steer_pulse(String, 강제) 토픽을 구독해 대응 메서드를 호출하고
/car/state(Int8)를 발행한다. 시리얼 프로토콜은 README '시리얼 프로토콜' 절 참고.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.arduino_node --selftest
"""
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


# 아두이노 state(0 정지/1 전진/2 후진)가 None(미연결)일 때 Int8로 실어보낼 센티널.
# 구독 쪽(mission_node)에서 다시 None으로 복원한다.
STATE_UNKNOWN = -1


# ============================ ROS2 래퍼 ============================

def ros_main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, Int8, Int16, String

    from .. import config
    from .ports import autodetect_ports

    class ArduinoBridgeNode(Node):
        """ArduinoNode(시리얼 프로토콜)를 그대로 소유하고 ROS 토픽만 얹는 얇은 래퍼.

        /car/cmd/go, /car/cmd/stop, /car/cmd/drive(Int16), /car/cmd/steer(String,
        dedup) /car/cmd/steer_pulse(String, 강제) 구독 → ArduinoNode의 대응 메서드
        (go/stop/drive/steer/steer_pulse)를 그대로 호출한다. 시리얼 프로토콜·워치독·
        dedupe 로직은 ArduinoNode에 손대지 않고 그대로 재사용한다.
        """

        def __init__(self):
            super().__init__("arduino_bridge_node")

            self.declare_parameter("port", "")
            self.declare_parameter("baud", config.ARDUINO_BAUD)

            port = self.get_parameter("port").value or None
            if port is None:
                port, _lidar = autodetect_ports()
            baud = self.get_parameter("baud").value

            self._car = ArduinoNode(port, baud)

            self.create_subscription(Empty, "/car/cmd/go", self._on_go, 10)
            self.create_subscription(Empty, "/car/cmd/stop", self._on_stop, 10)
            self.create_subscription(Int16, "/car/cmd/drive", self._on_drive, 10)
            self.create_subscription(String, "/car/cmd/steer", self._on_steer, 10)
            self.create_subscription(String, "/car/cmd/steer_pulse", self._on_steer_pulse, 10)

            self._state_pub = self.create_publisher(Int8, "/car/state", 10)
            self.create_timer(1.0 / config.LOOP_HZ, self._publish_state)

        def _on_go(self, _msg):
            self._car.go()

        def _on_stop(self, _msg):
            self._car.stop()

        def _on_drive(self, msg):
            self._car.drive(msg.data)

        def _on_steer(self, msg):
            self._car.steer(msg.data)

        def _on_steer_pulse(self, msg):
            self._car.steer_pulse(msg.data)

        def _publish_state(self):
            state = self._car.state
            self._state_pub.publish(Int8(data=STATE_UNKNOWN if state is None else state))

        def destroy_node(self):
            self._car.close()
            super().destroy_node()

    import signal

    def _on_sigterm(_signum, _frame):
        # ros2 launch 종료/kill 등 SIGTERM도 SIGINT와 동일하게 finally에서
        # node.destroy_node() → ArduinoNode.close() → stop()이 돌게 만든다
        # (모터가 마지막 속도로 계속 도는 것을 방지 — 워치독 500ms보다 즉시 정지가 안전).
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _on_sigterm)

    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ========================= 오프라인 테스트 / 셀프테스트 =========================

def selftest():
    """시리얼 없이 ArduinoNode의 dedup/펄스/정지 로직만 검증한다."""
    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    car = ArduinoNode(port=None)  # 포트 없음 → 실제 시리얼 없이 순수 로직만 테스트
    writes = []
    car._write = lambda text: writes.append(text)

    car.go()
    car.go()
    check("go() 중복 호출은 1회만 전송(dedup)", writes == ["G"])

    writes.clear()
    car.steer("L")
    car.steer("L")
    check("steer() 같은 방향 연속 호출은 1회만 전송(dedup)", writes == ["L"])

    writes.clear()
    car.steer_pulse("L")
    car.steer_pulse("L")
    check("steer_pulse()는 매번 강제 전송", writes == ["L", "L"])

    writes.clear()
    car._speed = 80
    car.stop()
    check("stop()은 속도를 0으로 리셋하고 V0/S를 전송",
          car._speed == 0 and writes == ["V0\n", "S"])

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    ros_main()
