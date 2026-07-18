"""미션 오케스트레이터 — 카메라/라이다/차량상태 토픽을 구독해 sensors dict를
구성하고 Mission 서브클래스(road/traffic/t_parking/test)를 그대로 구동한다.

ROS 배선(구독/타이머)이 __init__ 안에 인라인되어 있어 다른 노드 파일처럼 순수
core/ros_main() 분리와 --selftest를 두지 않는다. 이 오케스트레이션 레이어가
실제로 호출하는 Mission.step() 로직의 테스트는 tools/smoke_test_missions.py
(FakeCar/FakeClock로 ROS 없이 각 미션 FSM을 직접 구동) 참고.
"""
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Empty, Float32, Int8, Int16, String

try:
    import cv2
except ImportError:
    cv2 = None

from .. import config, debug_viz, drive_logger, tuning
from ..missions import MISSIONS
from .arduino_node import STATE_UNKNOWN
from .lidar_node import laserscan_msg_to_tuples

# Mission.debug 키 → (원본 프레임 슬롯, 그리기 함수). 오버레이 타이머가 이 표에
# 있는 키만 /debug/<키> CompressedImage로 발행한다 (없는 키는 무시).
OVERLAY_DRAWERS = {
    "lane_poi": ("bottom", debug_viz.draw_lane_poi),
    "obstacle": ("bottom", debug_viz.draw_obstacle),
    "stop_line": ("bottom", debug_viz.draw_stop_line),
    "traffic_light": ("top", debug_viz.draw_traffic_light),
    "parking_line": ("rear", debug_viz.draw_parking_line),
}

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
        # go/stop 게이트 + 마지막 속도 — republish()가 매 틱 재발행한다. 미션은
        # on_start()에서 go()/drive()를 딱 한 번만 부르는 경우가 많은데(steer()는
        # follow_lane_poi 등이 매 틱 부름), mission_node와 arduino_node는 별도
        # 프로세스라 그 한 번이 두 노드의 DDS 디스커버리가 끝나기 전에 나가면
        # 그냥 유실된다(기본 QoS는 late-joiner 재전송 없음) — 2026-07-17 실차에서
        # "조향은 되는데 안 움직임"으로 반복 확인. steer()처럼 계속 재발행해
        # 디스커버리가 끝난 뒤 어느 틱에서든 반드시 한 번은 도착하게 만든다.
        self._gate_open = False
        self._last_speed = 0
        self._last_steer = "F"  # drive_logger가 매 틱 마지막 조향 명령을 남기기 위한 추적

    def _on_state(self, msg):
        self._state = None if msg.data == STATE_UNKNOWN else msg.data

    @property
    def state(self):
        return self._state

    @property
    def gate_open(self):
        return self._gate_open

    @property
    def last_speed(self):
        return self._last_speed

    @property
    def last_steer(self):
        return self._last_steer

    def go(self):
        self._gate_open = True
        self._go_pub.publish(Empty())

    def stop(self):
        self._gate_open = False
        self._last_speed = 0
        self._stop_pub.publish(Empty())

    def drive(self, speed):
        self._last_speed = int(speed)
        self._drive_pub.publish(Int16(data=self._last_speed))

    def republish(self):
        """go/stop 게이트와 마지막 drive 속도를 다시 발행 — mission_node가 매 틱
        호출한다(steer는 이미 미션들이 매 틱 부르므로 별도 처리 불필요)."""
        (self._go_pub if self._gate_open else self._stop_pub).publish(Empty())
        self._drive_pub.publish(Int16(data=self._last_speed))

    def steer(self, direction):
        self._last_steer = direction
        self._steer_pub.publish(String(data=direction))

    def steer_pulse(self, direction):
        self._last_steer = direction
        self._steer_pulse_pub.publish(String(data=direction))


class MissionNode(Node):
    """카메라/라이다/차량상태 토픽을 구독해 sensors dict를 구성하고, 기존
    Mission 서브클래스(road/traffic/t_parking)를 그대로 구동하는 오케스트레이터."""

    def __init__(self):
        super().__init__("mission_node")

        self.declare_parameter("mission", "")
        self.declare_parameter("show", False)

        self._front = None
        self._back = None
        self._lidar_scan = None
        self._lidar_min_m = None
        self._pose = None       # (x, y, theta) — odometry_node 상대 pose
        self._pose_conf = 0.0   # 미수신/미보정 시 0.0 (fail-inert 기본값)
        self._pose_stamp = None  # 마지막 /car/pose 수신 시각 — staleness 감시용

        self.declare_parameter("split", config.CAMERA_SPLIT)
        self._split = self.get_parameter("split").value

        self.create_subscription(CompressedImage, "/camera/front",
                                  self._make_image_cb("front"), 10)
        self.create_subscription(CompressedImage, "/camera/back",
                                  self._make_image_cb("back"), 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Float32, "/lidar/rear_min_m", self._on_rear_min, 10)
        self.create_subscription(PoseStamped, "/car/pose", self._on_pose, 10)
        self.create_subscription(Float32, "/car/pose_confidence", self._on_pose_conf, 10)

        self._car = RosCarProxy(self)
        self._show = self.get_parameter("show").value
        # show:=true인데 DISPLAY가 없으면(SSH/헤드리스) cv2.imshow()가 Qt
        # xcb 플러그인을 못 찾아 프로세스 전체를 abort(exit -6)시킨다 —
        # 2026-07-17 실차 로그로 확인된 크래시. 여기서 미리 걸러 경고만
        # 남기고 끈다(show_frames()의 기존 'q' 입력 시 자동 off 로직은
        # imshow가 이미 죽은 뒤라 발동할 기회가 없었음).
        if self._show and cv2 is not None and not os.environ.get("DISPLAY"):
            self.get_logger().warn(
                "show:=true지만 DISPLAY 환경변수가 없습니다(SSH/헤드리스 추정) — "
                "cv2.imshow()가 프로세스를 죽이므로 카메라 미리보기를 끕니다. "
                "미리보기가 필요하면 'ssh -X'나 VNC 등으로 X 디스플레이를 연결하세요.")
            self._show = False

        # 실차 튜닝 파라미터 — 미션이 매 틱 읽는 튜닝 dict를 ros2 param set으로
        # 라이브 조정할 수 있게 노출한다 (미션 선택과 무관하게 전부 선언 —
        # 안 쓰는 namespace는 그냥 무해하다). on_start() 전에 설치해야
        # tuning_params:= 로 들어온 기동 시점 override가 미션 시작값에 반영된다.
        self._tuning_bindings = tuning.install(
            self, tuning.tunable_dicts(), tuning.tunable_attrs())

        mission_name = resolve_mission(self)
        self._mission_name = mission_name
        self.get_logger().info(f"mission={mission_name}")
        self._mission = MISSIONS[mission_name]()
        self._mission.on_start(self._car, config)

        # 디지털 트윈 재현용 주행 로그 (지도 교수 피드백, 2026-07-18) — 매 틱
        # 튜닝값+명령을 타임스탬프와 함께 JSON Lines로 남긴다. log_dir을 비우면
        # config.DRIVE_LOG_DIR 사용.
        self.declare_parameter("log_drive", True)
        self.declare_parameter("log_dir", "")
        self._drive_logger = None
        if self.get_parameter("log_drive").value:
            log_dir = self.get_parameter("log_dir").value or config.DRIVE_LOG_DIR
            log_path = drive_logger.make_log_path(log_dir, mission=mission_name)
            self._drive_logger = drive_logger.DriveLogger(log_path)
            self.get_logger().info(f"주행 로그: {log_path}")

        self.create_timer(1.0 / config.LOOP_HZ, self._tick)

        # 디버그 오버레이 — 감지기 분석(Mission.debug)을 프레임에 그려
        # /debug/*(CompressedImage)로 발행한다 (Foxglove 실차 튜닝용).
        # debug.overlay는 ros2 param set으로 라이브 on/off 가능. overlay_hz는
        # 타이머 생성 시점 값으로 고정(런타임 변경은 재기동 필요).
        self.declare_parameter("debug.overlay", True)
        self.declare_parameter("debug.overlay_hz", 5.0)
        self._overlay_pubs = {}
        overlay_hz = max(float(self.get_parameter("debug.overlay_hz").value), 0.1)
        self.create_timer(1.0 / overlay_hz, self._publish_overlays)

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

    def _on_pose(self, msg):
        # odometry_node는 z/w만 채운 평면 회전 quaternion을 발행 → theta 복원
        q = msg.pose.orientation
        self._pose = (msg.pose.position.x, msg.pose.position.y,
                      2.0 * math.atan2(q.z, q.w))
        self._pose_stamp = self.get_clock().now()

    def _on_pose_conf(self, msg):
        self._pose_conf = float(msg.data)

    def _split_front(self):
        """/camera/front 한 장을 신호등용(top)/차선용(bottom)으로 나눈다 —
        camera_node는 물리 카메라 2대에 맞춰 front/back만 발행하고, 이 분할은
        detection(mission_node) 책임이다."""
        frame = self._front
        if frame is None:
            return None, None
        if not self._split:
            return frame, frame
        h = frame.shape[0]
        return frame[:h // 2, :], frame[h // 2:, :]

    def _tick(self):
        # sensors dict 스키마: missions/base.py의 Mission 클래스 docstring 참고
        top, bottom = self._split_front()
        # odometry_node가 죽거나 멈춰 /car/pose가 갱신을 멈추면 pose_conf를
        # 마지막 값으로 영원히 유지하지 않도록 강제로 0(미가용)까지 낮춘다 —
        # t_parking 점유 격자 등이 문서화된 fail-inert 폴백 경로로 자연 진입.
        stale = (self._pose_stamp is None or
                 (self.get_clock().now() - self._pose_stamp).nanoseconds
                 > config.POSE_STALE_S * 1e9)
        pose_conf = 0.0 if stale else self._pose_conf
        sensors = {
            "top": top,
            "bottom": bottom,
            "rear": self._back,
            "lidar_min_m": self._lidar_min_m,
            "lidar_scan": self._lidar_scan,
            "state": self._car.state,
            "pose": self._pose,
            "pose_conf": pose_conf,
        }
        self._mission.step(sensors, self._car)
        # go/stop 게이트 + 마지막 drive 속도를 매 틱 재발행 — mission_node/
        # arduino_node가 별도 프로세스라 on_start()의 단발 go()/drive()가 DDS
        # 디스커버리 완료 전에 나가면 유실될 수 있음(RosCarProxy.republish() 참고).
        self._car.republish()
        if self._drive_logger is not None:
            self._drive_logger.log(
                drive_logger.snapshot_bindings(self._tuning_bindings),
                {"steer": self._car.last_steer, "drive": self._car.last_speed,
                 "go": self._car.gate_open},
                mission=self._mission_name, state=self._car.state)
        if self._show and not show_frames(top, bottom, self._back):
            self._show = False

    def _publish_overlays(self):
        """Mission.debug 스크래치를 debug_viz로 그려 /debug/*에 발행한다.
        미션 tick과 다른 프레임이 쓰일 수 있지만(최대 1 오버레이 주기 차이)
        튜닝 확인용으로는 충분하다. 실패해도 미션 루프에는 영향 없음."""
        if cv2 is None or not self.get_parameter("debug.overlay").value:
            return
        occ = self._mission.debug.get("occupancy")
        if occ is not None:
            try:
                self._publish_occupancy(occ)
            except Exception as e:
                self.get_logger().warning(f"점유 격자 발행 실패: {e}",
                                          throttle_duration_sec=5.0)
        top, bottom = self._split_front()
        frames = {"top": top, "bottom": bottom, "rear": self._back}
        for key, dbg in list(self._mission.debug.items()):
            entry = OVERLAY_DRAWERS.get(key)
            if entry is None:
                continue
            frame = frames.get(entry[0])
            if frame is None:
                continue
            try:
                vis = entry[1](frame, dbg)
                ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if not ok:
                    continue
                pub = self._overlay_pubs.get(key)
                if pub is None:  # 데이터가 실제로 생긴 토픽만 lazy 생성
                    pub = self.create_publisher(CompressedImage, f"/debug/{key}", 1)
                    self._overlay_pubs[key] = pub
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = buf.tobytes()
                pub.publish(msg)
            except Exception as e:
                self.get_logger().warning(f"오버레이({key}) 발행 실패: {e}",
                                          throttle_duration_sec=5.0)

    def _publish_occupancy(self, occ):
        """t_parking 점유 격자를 nav_msgs/OccupancyGrid로 발행 (frame_id=odom —
        Foxglove 3D 패널에서 /car/pose, /lidar/scan_corrected와 함께 겹쳐 보임)."""
        x0, y0, res, n, data = occ.to_ros_grid_data()
        pub = self._overlay_pubs.get("occupancy")
        if pub is None:
            pub = self.create_publisher(OccupancyGrid, "/debug/occupancy", 1)
            self._overlay_pubs["occupancy"] = pub
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.info.resolution = float(res)
        msg.info.width = n
        msg.info.height = n
        msg.info.origin.position.x = float(x0)
        msg.info.origin.position.y = float(y0)
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        pub.publish(msg)

    def destroy_node(self):
        self._mission.on_stop(self._car)
        if self._drive_logger is not None:
            self._drive_logger.close()
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
