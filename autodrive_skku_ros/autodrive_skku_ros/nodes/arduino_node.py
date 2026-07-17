#!/usr/bin/env python3
"""아두이노 시리얼 브릿지 — 차량 구동/조향 명령을 시리얼로 중계한다.

ArduinoNode: 시리얼 포트를 직접 소유하는 순수 파이썬 클래스 (ROS 비의존, 워치독
keepalive 스레드 포함). ros_main()의 ArduinoBridgeNode가 이 클래스를 얇게
감싸 /car/cmd/go, /car/cmd/stop, /car/cmd/drive(Int16), /car/cmd/steer(String,
dedup), /car/cmd/steer_pulse(String, 강제) 토픽을 구독해 대응 메서드를 호출하고
/car/state(Int8)를 발행한다. 시리얼 프로토콜은 README '시리얼 프로토콜' 절 참고.

조향 POT(가변저항, A6)이 장착돼 있으면 기동 시 1회 좌/우 풀락 ADC를 자동으로
찾아(calibrate_steering) /car/steering_pot(Int32, raw ADC), /car/steering_angle
(Float32, deg)로 발행한다. POT 미장착이면 자동으로 스킵되고 기존 펄스 방식
그대로 동작 — 이 하드웨어는 선택사항이다.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.arduino_node --selftest
"""
import threading
import time

try:
    import serial
except ImportError:
    serial = None

# 출발/정지 속도 램프 시간(초) — 선배 조언(2026-07-17): control단 마지막
# 단계(실제 전송 직전)에 Cubic Polynomial Trajectory를 적용해 급가속/급정지로
# 인한 휠슬립·드리프트를 완화한다. 📏 실차 튜닝 대상.
SPEED_RAMP_S = 0.5


def cubic_trajectory(t, duration, v0, v1):
    """Cubic Polynomial Trajectory — 경계조건 x(0)=v0, x(T)=v1, ẋ(0)=ẋ(T)=0
    (양끝 가속도 0)을 만족하는 3차 다항식 속도 보간. t<=0이면 v0, t>=duration이면
    v1로 클램프. duration<=0이면 즉시 v1(램프 없음)."""
    if duration <= 0:
        return v1
    s = max(0.0, min(1.0, t / duration))
    return v0 + (v1 - v0) * (3 * s * s - 2 * s * s * s)


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
        self.pot_adc = None  # 조향 POT 원시값(A6, 0~1023) — 펌웨어가 항상 보냄
        self._ser = None
        self._speed = 0            # 마지막으로 실제 전송한 값(램프 진행 중 포함)
        self._speed_target = 0     # drive()가 지정한 목표 속도
        self._speed_ramp_from = 0  # 현재 램프 시작점(목표 변경 시점의 _speed)
        self._speed_ramp_t0 = 0.0  # 램프 시작 시각(time.time())
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
                self._speed = self._ramped_speed(now)
                self._write(f"V{self._speed}\n")
                last_keepalive = now
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                time.sleep(0.05)
                continue
            if line in ("0", "1", "2"):
                self.state = int(line)
            elif line.startswith("P "):
                try:
                    self.pot_adc = int(line[2:])
                except ValueError:
                    pass

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
        """부호 있는 속도 목표 지정. 음수 = 후진. 실제 전송(keepalive)은 이
        목표까지 Cubic Polynomial Trajectory(SPEED_RAMP_S초)로 부드럽게
        램프한다 — 급가속/급정지로 인한 휠슬립·드리프트 완화. 같은 목표값
        연속 호출은 램프를 재시작하지 않는다(미션이 매 틱 같은 속도로
        drive()를 부르므로). 즉시 정지가 필요하면 stop()을 쓸 것 — 램프를
        타지 않는다."""
        target = max(-255, min(255, int(speed)))
        if target != self._speed_target:
            self._speed_ramp_from = self._speed
            self._speed_ramp_t0 = time.time()
            self._speed_target = target

    def _ramped_speed(self, now):
        if self._speed_target == self._speed_ramp_from:
            return self._speed_target
        value = cubic_trajectory(now - self._speed_ramp_t0, SPEED_RAMP_S,
                                 self._speed_ramp_from, self._speed_target)
        return int(round(value))

    def steer(self, direction):
        """조향 (같은 방향 연속 호출은 무시). F=조향 모터 정지, L/R=한 펄스."""
        self._send_once("steer", direction if direction in ("F", "L", "R") else "F")

    def steer_pulse(self, direction):
        """조향 펄스를 강제로 한 번 더 보낸다 (주차 등 반복 조향 기동용)."""
        if direction in ("F", "L", "R"):
            self._write(direction)
            self._last["steer"] = direction

    def stop(self):
        """즉시 정지 — 램프를 타지 않는다(안전 우선: 미션 종료/비상정지에서
        Cubic Trajectory로 서서히 멈추면 위험). 램프 상태도 0으로 리셋해
        다음 keepalive tick이 남은 램프를 이어서 재출발시키지 않게 한다."""
        self._speed = 0
        self._speed_target = 0
        self._speed_ramp_from = 0
        self._speed_ramp_t0 = time.time()
        self._write("V0\n")
        self._send_once("gate", "S")

    def close(self):
        self.stop()
        self._running = False
        time.sleep(0.1)
        if self._ser is not None:
            self._ser.close()

    def _read_pot_median(self, duration, samples=5):
        """duration 동안 samples번 나눠 읽어 중앙값을 반환.

        조향 모터 펄스 직후 순간적으로 튀는 값(예: 349→356→349처럼 한 틱만
        스파이크)이 스티어링 모터 노이즈로 ADC에 실릴 수 있어, 한 번만 읽는
        대신 여러 번 읽어 중앙값을 쓰는 게 이상치에 더 강하다.
        """
        readings = []
        step = duration / samples if samples > 0 else duration
        for _ in range(max(samples, 1)):
            time.sleep(step)
            adc = self.pot_adc
            if adc is not None:
                readings.append(adc)
        if not readings:
            return None
        readings.sort()
        return readings[len(readings) // 2]

    def calibrate_steering(self, max_pulses=40, min_pulses=8, settle_s=0.18,
                            stable_count=3, stable_tol=1, min_span=3,
                            recenter_tol=1, pot_timeout_s=2.0):
        """조향 POT 기준 좌/우 풀락 ADC를 실측하고, 중앙으로 복귀시킨 뒤 반환한다.

        방향별로 steer_pulse()를 반복하면서, 최소 min_pulses는 무조건 채운
        뒤부터 ADC가 stable_count회 연속 stable_tol 이내로 안 바뀌면 기계적
        풀락(스토퍼)에 닿았다고 판단한다. 각 방향 max_pulses가 최종 상한 —
        스토퍼를 못 찾아도 기어박스에 무리가 가지 않도록 여기서 반드시 멈춘다.
        풀락 값은 마지막 stable_count개 읽음값의 평균으로 잡아 ADC 노이즈를
        줄인다.

        📏 min_pulses가 필요한 이유(2026-07 실측): 조향 링키지-POT 스윙이
        ADC 기준 4카운트 정도로 좁아 ADC 노이즈(±1~2)와 신호가 비슷한 크기다
        — min_pulses 없이 stable_count(3)만으로 판정하면 실제 풀락에 도달하기
        전에 노이즈가 우연히 "안정된 것처럼" 보여 너무 일찍 멈추는 문제가
        있었음(예: 5펄스 만에 [348,347,349,348,348]로 멈춰 좌/우가 348/347로
        거의 같아짐 — 실제 풀락은 346/350).

        POT이 실제로 없어도 A6가 플로팅이라 펌웨어는 계속 "P <adc>" 라인을
        보낸다 — 그래서 "라인이 오는지"가 아니라 "스윕해봤더니 ADC가 실제로
        min_span 이상 움직였는지"로 진짜 POT 장착 여부를 판단한다.

        반환: (adc_left, adc_right) — POT 미장착/응답없음이면 (None, None).
        """
        t0 = time.time()
        while self.pot_adc is None and time.time() - t0 < pot_timeout_s:
            time.sleep(0.05)
        if self.pot_adc is None:
            print("[arduino] POT 라인 응답 없음 — 캘리브레이션 스킵")
            return None, None

        def sweep(direction):
            history = []
            for i in range(max_pulses):
                self.steer_pulse(direction)
                adc = self._read_pot_median(settle_s)
                if adc is None:
                    continue
                history.append(adc)
                if i + 1 >= min_pulses and len(history) >= stable_count and \
                        max(history[-stable_count:]) - min(history[-stable_count:]) <= stable_tol:
                    break
            print(f"[arduino] {direction} 스윕: {len(history)}펄스, 기록={history}")
            if not history:
                return None
            tail = history[-stable_count:]
            return sum(tail) / len(tail)

        adc_left = sweep("L")
        adc_right = sweep("R")

        span = None if (adc_left is None or adc_right is None) else abs(adc_left - adc_right)
        print(f"[arduino] 스윕 결과: adc_left={adc_left}, adc_right={adc_right}, "
              f"span={span}, min_span={min_span}")

        if adc_left is None or adc_right is None or span < min_span:
            print("[arduino] POT 값이 조향에 반응하지 않음(미장착 추정) — 캘리브레이션 스킵")
            self.steer("F")
            return None, None

        # 방금 R로 풀락(adc_right)까지 왔으므로, 중앙(mid)까지는 부호를 몰라도
        # "L 방향으로 가면 adc_left에 가까워진다"만 알면 매 틱 방향을 다시 정해
        # 재수렴시킬 수 있다 — 오버슈트해도 스스로 반대로 튼다.
        mid = (adc_left + adc_right) / 2.0
        increasing_dir = "L" if adc_left > adc_right else "R"
        decreasing_dir = "R" if adc_left > adc_right else "L"
        for _ in range(max_pulses):
            adc = self.pot_adc
            if adc is None or abs(adc - mid) <= recenter_tol:
                break
            self.steer_pulse(decreasing_dir if adc > mid else increasing_dir)
            time.sleep(settle_s)

        self.steer("F")
        print(f"[arduino] 조향 캘리브레이션 완료: adc_left={adc_left}, adc_right={adc_right}")
        return adc_left, adc_right


# 아두이노 state(0 정지/1 전진/2 후진)가 None(미연결)일 때 Int8로 실어보낼 센티널.
# 구독 쪽(mission_node)에서 다시 None으로 복원한다.
STATE_UNKNOWN = -1


def adc_to_deg(adc, adc_left, adc_right, angle_left_deg, angle_right_deg):
    """조향 POT ADC → 각도[deg] 선형 매핑 + 클램프 (calibrate_steering 결과 사용).

    반환은 항상 float로 고정한다 — 클램프가 걸리면 min()/max()가 lo/hi를
    그대로 반환하는데, 호출부가 config.STEERING_LIMIT_DEG(int) 같은 정수
    한계값을 넘기면 클램프된 결과가 int가 돼 Float32 발행 시 타입 assert로
    죽는다(2026-07-17 실차: ADC 노이즈로 실제 클램프가 걸려서야 처음 발견됨).
    """
    if adc_left == adc_right:
        return 0.0
    t = (adc - adc_right) / (adc_left - adc_right)
    deg = angle_right_deg + t * (angle_left_deg - angle_right_deg)
    lo, hi = min(angle_left_deg, angle_right_deg), max(angle_left_deg, angle_right_deg)
    return float(max(lo, min(hi, deg)))


# ============================ ROS2 래퍼 ============================

def ros_main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, Int8, Int16, Int32, Float32, String

    from .. import config
    from .ports import autodetect_ports

    class ArduinoBridgeNode(Node):
        """ArduinoNode(시리얼 프로토콜)를 그대로 소유하고 ROS 토픽만 얹는 얇은 래퍼.

        /car/cmd/go, /car/cmd/stop, /car/cmd/drive(Int16), /car/cmd/steer(String,
        dedup) /car/cmd/steer_pulse(String, 강제) 구독 → ArduinoNode의 대응 메서드
        (go/stop/drive/steer/steer_pulse)를 그대로 호출한다. 시리얼 프로토콜·워치독·
        dedupe 로직은 ArduinoNode에 손대지 않고 그대로 재사용한다.

        calibrate_steering 파라미터(기본 true)가 켜져 있으면 시작 시 1회
        ArduinoNode.calibrate_steering()을 돌려 좌/우 풀락 ADC를 찾고,
        그 결과로 /car/steering_pot(Int32)·/car/steering_angle(Float32)를
        계속 발행한다. POT 미장착이면 자동으로 조용히 스킵된다.
        """

        def __init__(self):
            super().__init__("arduino_bridge_node")

            self.declare_parameter("port", "")
            self.declare_parameter("baud", config.ARDUINO_BAUD)
            self.declare_parameter("calibrate_steering", True)

            port = self.get_parameter("port").value or None
            if port is None:
                port, _lidar = autodetect_ports()
            baud = self.get_parameter("baud").value

            self._car = ArduinoNode(port, baud)

            # 구독/발행/타이머를 먼저 등록한다 — calibrate_steering()이 최대
            # 수 초간 블로킹하는 동안(2026-07-17 실차 로그: ~4초), mission_node는
            # 별도 프로세스라 이미 on_start()에서 car.go()/car.drive()를 (한 번만)
            # 발행해버릴 수 있다. 구독이 나중에 생기면 기본 QoS(VOLATILE)라
            # 그 사이 도착한 메시지는 유실되고 — go/drive를 반복 호출하지 않는
            # 미션(road 등)에서는 차가 조향만 되고 영원히 안 움직이는 버그가
            # 됐었다. DDS 레벨에서는 구독 등록만 되면(콜백은 spin() 시작 후 처리)
            # 큐잉되므로, 블로킹 작업 전에 구독부터 만들면 메시지가 안 유실된다.
            self.create_subscription(Empty, "/car/cmd/go", self._on_go, 10)
            self.create_subscription(Empty, "/car/cmd/stop", self._on_stop, 10)
            self.create_subscription(Int16, "/car/cmd/drive", self._on_drive, 10)
            self.create_subscription(String, "/car/cmd/steer", self._on_steer, 10)
            self.create_subscription(String, "/car/cmd/steer_pulse", self._on_steer_pulse, 10)

            self._state_pub = self.create_publisher(Int8, "/car/state", 10)
            self._pot_pub = self.create_publisher(Int32, "/car/steering_pot", 10)
            self._angle_pub = self.create_publisher(Float32, "/car/steering_angle", 10)
            # 캘리브레이션 성공 시의 좌/우 풀락 ADC 차이 — odometry_node가 POT 각도를
            # 신뢰할지 판단하는 데 쓴다(config.ODOMETRY.min_pot_span_counts). 이
            # min_span(장착 여부 판정, 기본 3)보다 엄격한 별도 기준이 필요한 이유는
            # calibrate_steering()의 실측 로그 참고 — 2026-07 실측 링크 커플링은
            # 풀락 스윙이 ADC 4카운트뿐이라 min_span=3은 통과해도 각도 분해능은
            # 너무 거칠다.
            self._span_pub = self.create_publisher(Int32, "/car/steering_pot_span", 10)
            self.create_timer(1.0 / config.LOOP_HZ, self._publish_state)

            self._adc_left = None
            self._adc_right = None
            self._adc_span = None
            if self.get_parameter("calibrate_steering").value:
                self.get_logger().info("조향 캘리브레이션 시작 (좌/우 풀락 탐색)...")
                self._adc_left, self._adc_right = self._car.calibrate_steering()
                if self._adc_left is None:
                    self.get_logger().warn("조향 POT 미검출 — 캘리브레이션 없이 펄스 방식으로 동작")
                else:
                    # calibrate_steering()은 여러 샘플 평균(float)을 반환 —
                    # Int32는 진짜 int만 받으므로(std_msgs 타입 assert) 반올림.
                    self._adc_span = int(round(abs(self._adc_left - self._adc_right)))

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

            adc = self._car.pot_adc
            if adc is None:
                return
            self._pot_pub.publish(Int32(data=adc))
            if self._adc_left is not None:
                deg = adc_to_deg(adc, self._adc_left, self._adc_right,
                                  config.STEERING_LIMIT_DEG, -config.STEERING_LIMIT_DEG)
                self._angle_pub.publish(Float32(data=deg))
                self._span_pub.publish(Int32(data=self._adc_span))

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
    """시리얼 없이 ArduinoNode의 dedup/펄스/정지/조향 캘리브레이션 로직만 검증한다."""
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

    # ---- Cubic Polynomial Trajectory (속도 램프) ----
    check("cubic_trajectory: t<=0 -> v0", cubic_trajectory(-1, 1.0, 0, 100) == 0)
    check("cubic_trajectory: t>=duration -> v1", cubic_trajectory(2, 1.0, 0, 100) == 100)
    check("cubic_trajectory: 중간(t=T/2) -> 정확히 중점(대칭 smoothstep)",
          abs(cubic_trajectory(0.5, 1.0, 0, 100) - 50.0) < 1e-9)
    check("cubic_trajectory: duration<=0 -> 즉시 v1(램프 없음)",
          cubic_trajectory(0.0, 0.0, 0, 100) == 100)

    ramp_car = ArduinoNode(port=None)
    ramp_car._write = lambda text: None
    t0 = time.time()
    ramp_car.drive(200)
    check("drive() 직후에도 _speed는 아직 안 바뀜(램프는 keepalive가 진행)",
          ramp_car._speed == 0 and ramp_car._speed_target == 200)
    check("램프 시작 직후(t~=0) 램프값은 시작점 근처",
          ramp_car._ramped_speed(t0) == 0)
    check("램프 절반 지점 -> 목표의 절반 근처",
          abs(ramp_car._ramped_speed(t0 + SPEED_RAMP_S / 2) - 100) <= 1)
    check("램프 완료 후 -> 목표값",
          ramp_car._ramped_speed(t0 + SPEED_RAMP_S + 1) == 200)

    same_t0 = ramp_car._speed_ramp_t0
    ramp_car.drive(200)  # 같은 목표값 연속 호출 -> 램프 재시작 안 함(미션이 매 틱 호출)
    check("같은 목표값 연속 drive() 호출은 램프를 재시작하지 않음",
          ramp_car._speed_ramp_t0 == same_t0)

    ramp_car._speed = 150  # keepalive가 램프 중간에 전송했다고 가정
    ramp_car.stop()
    check("stop()은 램프를 타지 않고 즉시 0 + 램프 상태도 리셋",
          ramp_car._speed == 0 and ramp_car._speed_target == 0
          and ramp_car._ramped_speed(time.time() + 10) == 0)

    check("adc_to_deg: 좌 최대", abs(adc_to_deg(460, 460, 352, 20, -20) - 20.0) < 1e-9)
    check("adc_to_deg: 우 최대", abs(adc_to_deg(352, 460, 352, 20, -20) + 20.0) < 1e-9)
    check("adc_to_deg: 중앙 0도", abs(adc_to_deg(406, 460, 352, 20, -20)) < 1e-9)
    check("adc_to_deg: 범위 밖 클램프",
          adc_to_deg(1023, 460, 352, 20, -20) == 20.0 and
          adc_to_deg(0, 460, 352, 20, -20) == -20.0)

    # POT이 있는 것처럼 시뮬레이션: L 펄스마다 -5(하한 300), R 펄스마다 +5(상한 500)
    cal_car = ArduinoNode(port=None)
    cal_car.pot_adc = 400  # 캘리브레이션 시작 전 "POT 응답 있음"으로 간주되는 초기값
    cal_car._write = lambda text: None

    def fake_steer_pulse(direction):
        if direction == "L":
            cal_car.pot_adc = max(300, cal_car.pot_adc - 5)
        elif direction == "R":
            cal_car.pot_adc = min(500, cal_car.pot_adc + 5)
    cal_car.steer_pulse = fake_steer_pulse

    adc_left, adc_right = cal_car.calibrate_steering(
        max_pulses=80, settle_s=0, stable_count=3, stable_tol=0,
        min_span=30, recenter_tol=4, pot_timeout_s=0.2)
    check("캘리브레이션: 좌 풀락(300 근처) 수렴",
          adc_left is not None and abs(adc_left - 300) <= 3)
    check("캘리브레이션: 우 풀락(500 근처) 수렴",
          adc_right is not None and abs(adc_right - 500) <= 3)
    check("캘리브레이션 후 중앙(400 근처)으로 복귀",
          abs(cal_car.pot_adc - 400) <= 4)

    # POT 미장착 시뮬레이션: 펄스를 줘도 ADC가 거의 안 움직임 → 스킵돼야 함
    nopot_car = ArduinoNode(port=None)
    nopot_car.pot_adc = 512  # 플로팅 A6의 노이즈 섞인 고정값 흉내
    nopot_car._write = lambda text: None
    nopot_car.steer_pulse = lambda d: None  # 펄스를 줘도 ADC 불변(POT 미연결)
    adc_left2, adc_right2 = nopot_car.calibrate_steering(
        max_pulses=10, settle_s=0, stable_count=3, stable_tol=0,
        min_span=30, recenter_tol=4, pot_timeout_s=0.2)
    check("POT 미장착(값 불변)이면 캘리브레이션 스킵 → (None, None)",
          adc_left2 is None and adc_right2 is None)

    # 2026-07-16 실측 회귀 테스트: 조향 링키지-POT 커플링이 1:1이 아니라 풀락
    # 스윙이 ADC 4카운트 정도(346~350)밖에 안 되는 실제 하드웨어 — 기본
    # 파라미터(min_span=3 등)로도 "미장착"으로 스킵되지 않고 잡혀야 한다.
    narrow_car = ArduinoNode(port=None)
    narrow_car.pot_adc = 348
    narrow_car._write = lambda text: None

    def narrow_steer_pulse(direction):
        if direction == "L":
            narrow_car.pot_adc = max(346, narrow_car.pot_adc - 1)
        elif direction == "R":
            narrow_car.pot_adc = min(350, narrow_car.pot_adc + 1)
    narrow_car.steer_pulse = narrow_steer_pulse

    adc_left3, adc_right3 = narrow_car.calibrate_steering(settle_s=0, pot_timeout_s=0.2)
    check("좁은 실측 범위(ADC 4카운트)도 기본 파라미터로 캘리브레이션됨(스킵 안 됨)",
          adc_left3 is not None and adc_right3 is not None)

    # 2026-07-17 실차 회귀: calibrate_steering()은 float(샘플 평균)를 반환하므로
    # abs(adc_left-adc_right)도 float — ArduinoBridgeNode._publish_state()가 이걸
    # 그대로 Int32(data=...)에 넣으면 std_msgs 타입 assert로 죽는다(POT이 실제로
    # 검출된 최초 실행에서만 이 경로를 타서 그동안 안 잡혔음). int(round(...))로
    # 감싸야 한다는 계약을 여기서 고정한다.
    check("calibrate_steering() 반환값은 float(샘플 평균) — Int32 발행 전 캐스팅 필요",
          isinstance(adc_left3, float) and isinstance(adc_right3, float))
    span_for_publish = int(round(abs(adc_left3 - adc_right3)))
    check("int(round(...)) 캐스팅 후에는 진짜 int (Int32에 안전하게 발행 가능)",
          type(span_for_publish) is int)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    ros_main()
