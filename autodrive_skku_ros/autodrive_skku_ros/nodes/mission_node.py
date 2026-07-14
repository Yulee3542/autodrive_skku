"""미션 오케스트레이터 — 카메라/라이다/차량상태 토픽을 구독해 sensors dict를
구성하고 Mission 서브클래스(road/traffic/t_parking/test)를 그대로 구동한다.

ROS 배선(구독/타이머)이 __init__ 안에 인라인되어 있어 다른 노드 파일처럼 순수
core/ros_main() 분리와 --selftest를 두지 않는다. 이 오케스트레이션 레이어가
실제로 호출하는 Mission.step() 로직의 테스트는 tools/smoke_test_missions.py
(FakeCar/FakeClock로 ROS 없이 각 미션 FSM을 직접 구동) 참고.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Empty, Float32, Int8, Int16, String

try:
    import cv2
except ImportError:
    cv2 = None

from .. import config
from ..missions import MISSIONS
from .arduino_node import STATE_UNKNOWN
from .lidar_node import laserscan_msg_to_tuples

MISSION_DESC = {
    "road": "도로 주행 — 차선 인식/차선 변경/장애물 회피",
    "traffic": "신호등 주행 — 정지선/신호등 인식",
    "t_parking": "T 주차 — 라이다 맵/후방캠 주차선",
    "test": "수동 테스트 — 자동주행 없음, teleop_node/ros2 topic pub과 병행",
}


def pick_mission():
    names = list(MISSIONS)
    print("\n미션 선택:")
    for i, name in enumerate(names, 1):
        print(f"  {i}. {name:<10} {MISSION_DESC.get(name, '')}")
    while True:
        choice = input("번호 또는 이름 입력 > ").strip().lower()
        if choice in MISSIONS:
            return choice
        if choice.isdigit() and 1 <= int(choice) <= len(names):
            return names[int(choice) - 1]
        print("잘못된 입력입니다.")


def resolve_mission(node):
    """mission 파라미터가 비었으면: stdin이 진짜 tty일 때만(= 'ros2 run'으로 직접
    실행) 대화형 메뉴로 폴백한다. 'ros2 launch'는 자식 프로세스의 stdin을 연결하지
    않는 launch 시스템 자체의 알려진 제약(ros2/launch#735)이라 input()이 그냥
    영원히 멈춘다 — 조용히 멈추는 대신 바로 에러로 안내한다."""
    name = node.get_parameter("mission").value
    if name:
        return name
    if sys.stdin.isatty():
        return pick_mission()
    node.get_logger().fatal(
        "mission 파라미터가 비어 있고 stdin이 tty가 아닙니다. 'ros2 launch'는 자식 "
        "프로세스의 stdin을 연결하지 않아 대화형 메뉴를 쓸 수 없습니다(ROS 2 launch "
        "자체의 알려진 제약, ros2/launch#735) — "
        "'ros2 launch autodrive_skku_ros bringup.launch.py mission:=road'처럼 launch "
        "인자로 미션을 지정하세요. (대화형 메뉴는 'ros2 run autodrive_skku_ros "
        "mission_node'로 직접 실행할 때만 동작합니다.)")
    raise SystemExit(1)


def show_frames(top, bottom, rear):
    if cv2 is None:
        return True
    if top is not None:
        cv2.imshow("top (traffic light)", top)
    if bottom is not None:
        cv2.imshow("bottom (lane)", bottom)
    if rear is not None:
        cv2.imshow("rear (parking)", rear)
    return (cv2.waitKey(1) & 0xFF) != ord("q")


class RosCarProxy:
    """ArduinoNode와 동일한 인터페이스(go/drive/steer/steer_pulse/stop/.state)를
    ROS 토픽 발행/구독으로 구현한 어댑터. Mission.step(sensors, car)이 그대로
    재사용될 수 있도록 하는 게 유일한 목적 — 미션 코드는 이 객체가 ROS로
    구현됐는지 몰라도 된다."""

    def __init__(self, node):
        self._go_pub = node.create_publisher(Empty, "/car/cmd/go", 10)
        self._stop_pub = node.create_publisher(Empty, "/car/cmd/stop", 10)
        self._drive_pub = node.create_publisher(Int16, "/car/cmd/drive", 10)
        self._steer_pub = node.create_publisher(String, "/car/cmd/steer", 10)
        self._steer_pulse_pub = node.create_publisher(String, "/car/cmd/steer_pulse", 10)
        self._state = None
        node.create_subscription(Int8, "/car/state", self._on_state, 10)

    def _on_state(self, msg):
        self._state = None if msg.data == STATE_UNKNOWN else msg.data

    @property
    def state(self):
        return self._state

    def go(self):
        self._go_pub.publish(Empty())

    def stop(self):
        self._stop_pub.publish(Empty())

    def drive(self, speed):
        self._drive_pub.publish(Int16(data=int(speed)))

    def steer(self, direction):
        self._steer_pub.publish(String(data=direction))

    def steer_pulse(self, direction):
        self._steer_pulse_pub.publish(String(data=direction))


class MissionNode(Node):
    """카메라/라이다/차량상태 토픽을 구독해 sensors dict를 구성하고, 기존
    Mission 서브클래스(road/traffic/t_parking)를 그대로 구동하는 오케스트레이터."""

    def __init__(self):
        super().__init__("mission_node")

        self.declare_parameter("mission", "")
        self.declare_parameter("show", False)

        self._top = None
        self._bottom = None
        self._rear = None
        self._lidar_scan = None
        self._lidar_min_m = None

        self.create_subscription(CompressedImage, "/camera/top",
                                  self._make_image_cb("top"), 10)
        self.create_subscription(CompressedImage, "/camera/bottom",
                                  self._make_image_cb("bottom"), 10)
        self.create_subscription(CompressedImage, "/camera/rear",
                                  self._make_image_cb("rear"), 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Float32, "/lidar/rear_min_m", self._on_rear_min, 10)

        self._car = RosCarProxy(self)
        self._show = self.get_parameter("show").value

        mission_name = resolve_mission(self)
        self.get_logger().info(f"mission={mission_name}")
        self._mission = MISSIONS[mission_name]()
        self._mission.on_start(self._car, config)

        self.create_timer(1.0 / config.LOOP_HZ, self._tick)

    def _make_image_cb(self, slot):
        def _cb(msg):
            if cv2 is None:
                return
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            setattr(self, f"_{slot}", frame)
        return _cb

    def _on_scan(self, msg):
        self._lidar_scan = laserscan_msg_to_tuples(msg)

    def _on_rear_min(self, msg):
        self._lidar_min_m = None if math.isnan(msg.data) else msg.data

    def _tick(self):
        # sensors dict 스키마: missions/base.py의 Mission 클래스 docstring 참고
        sensors = {
            "top": self._top,
            "bottom": self._bottom,
            "rear": self._rear,
            "lidar_min_m": self._lidar_min_m,
            "lidar_scan": self._lidar_scan,
            "state": self._car.state,
        }
        self._mission.step(sensors, self._car)
        if self._show and not show_frames(self._top, self._bottom, self._rear):
            self._show = False

    def destroy_node(self):
        self._mission.on_stop(self._car)
        super().destroy_node()


def _on_sigterm(_signum, _frame):
    # ros2 launch 종료/kill 등 SIGTERM도 SIGINT와 동일하게 finally에서
    # node.destroy_node() → Mission.on_stop(car) → car.stop() 발행이 돌게 만든다
    # (미션 프로세스가 죽어도 마지막 명령대로 차가 계속 움직이지 않도록).
    raise SystemExit(0)


def main(args=None):
    import signal
    signal.signal(signal.SIGTERM, _on_sigterm)

    rclpy.init(args=args)
    try:
        node = MissionNode()
    except SystemExit:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
