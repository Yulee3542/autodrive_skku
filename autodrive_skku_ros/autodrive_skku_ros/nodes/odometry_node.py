#!/usr/bin/env python3
"""상대 오도메트리 노드 — 시각 오도메트리(VO)와 커맨드-적분(가짜 데드레커닝)을
융합해 미션 시작 이후 상대 pose(x, y, theta)를 추정한다.

IEEE 5520874("Integrating visual odometry and dead-reckoning for robot
localization and obstacle detection")의 접근(평면 지면 가정 하 모노큘러 VO +
데드레커닝 융합)에서 착안했으나, 이 차량에는 휠 인코더/IMU/GPS가 전혀 없다 —
"데드레커닝" 항은 실제 바퀴 회전이 아니라 커맨드(속도 PWM, 조향각)의 적분값일
뿐이라 슬립/정지/배터리 처짐을 감지하지 못한다("커맨드-적분"/"가짜
데드레커닝"이라 부르고 논문의 진짜 인코더 데드레커닝과 동일시하지 않는다).
조향각은 가능하면 조향 POT 실측값(/car/steering_angle)을, POT 스윙이 너무
좁으면(미보정 링크 커플링) 펄스 카운트 적분으로 폴백한다.

CAMERA_MOUNT(카메라 높이/틸트)와 ODOMETRY.pwm_to_mps가 미측정(None)인 동안은
VO/커맨드-적분 둘 다 비활성(정지 취급, confidence=0)으로 fail-inert 동작한다 —
POT 미장착 시 calibrate_steering()이 조용히 스킵되는 것과 같은 패턴.

상대 오도메트리일 뿐 전역/GPS 기준 위치는 아니다 — 누적 오차는 시간이 지나면
무한정 커진다.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.odometry_node --selftest
"""
import math

from .. import filters

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None


# ============================ 순수 함수 (ROS/하드웨어 불필요, 셀프테스트 대상) ============================

_MAX_GROUND_RANGE_M = 20.0  # 이보다 먼(주로 지평선 근처 투영 폭주) 지면 투영점은 버림


def ground_projection_matrix(cam_mount, hfov_deg, vfov_deg, frame_w, frame_h, crop_top_px=0):
    """카메라 마운트(높이/틸트)와 화각으로 픽셀→지면(X=전방, Y=좌측, m) 호모그래피를
    만든다. height_m/tilt_deg가 미측정(None)이면 None — 호출자는 VO 비활성으로
    처리해야 한다 (estimate_visual_motion은 H=None이면 항상 ok=False).

    frame_w/frame_h: 실제로 VO가 도는 (크롭된) 프레임 크기. hfov_deg/vfov_deg는
    크롭 전 원본 프레임 기준 화각이므로, crop_top_px(원본 상단에서 잘려나간 픽셀
    수 — 우리 경우 상/하 분할의 상단 절반)만큼 주점(cy)을 보정한다. 크롭은
    초점거리를 바꾸지 않고 주점만 옮긴다는 핀홀 카메라 성질을 이용한다.
    """
    if np is None:
        return None
    height_m = cam_mount.get("height_m")
    tilt_deg = cam_mount.get("tilt_deg")
    if height_m is None or tilt_deg is None:
        return None

    full_h = frame_h + crop_top_px
    fx = (frame_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = (full_h / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    cx = frame_w / 2.0
    cy = full_h / 2.0 - crop_top_px

    k_inv = np.array([[1.0 / fx, 0.0, -cx / fx],
                       [0.0, 1.0 / fy, -cy / fy],
                       [0.0, 0.0, 1.0]])
    # 카메라축(우=x_c, 하=y_c, 광축=z_c) → 차량축(전방=X, 좌=Y, 상=Z) 고정 회전.
    cam_to_vehicle = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])
    t = math.radians(tilt_deg)  # 하향 틸트, 차량 Y축 기준 회전(전방축이 -Z 쪽으로 기움)
    tilt_r = np.array([[math.cos(t), 0.0, math.sin(t)],
                        [0.0, 1.0, 0.0],
                        [-math.sin(t), 0.0, math.cos(t)]])

    n = tilt_r @ cam_to_vehicle @ k_inv  # 픽셀 동차좌표 → 차량 프레임 광선 방향
    n1, n2, n3 = n[0], n[1], n[2]
    # 광선과 지면(Z=-height_m, 카메라가 지면 위 height_m) 교점을 동차좌표로 정리한 것.
    return np.vstack([-height_m * n1, -height_m * n2, n3])


def _project_ground(pts_px, H):
    pts = pts_px.reshape(-1, 1, 2).astype(np.float64)
    return cv2.perspectiveTransform(pts, H.astype(np.float64)).reshape(-1, 2)


def _vo_empty(**overrides):
    result = dict(dx_m=0.0, dy_m=0.0, dtheta_rad=0.0, n_tracked=0, n_inliers=0, ok=False)
    result.update(overrides)
    return result


def estimate_visual_motion(prev_gray, curr_gray, H, vo_cfg):
    """평면 지면 가정 하 모노큘러 VO 1스텝 (연속 두 프레임 그레이스케일 비교).

    goodFeaturesToTrack로 코너를 뽑고 calcOpticalFlowPyrLK로 추적, H로 지면
    좌표(m)에 역투영한 뒤 RANSAC 강체변환(estimateAffinePartial2D)을 적합한다.
    실패 조건(H 없음/첫 프레임/특징점·인라이어 부족) 전부 ok=False, 이동량은
    0으로 반환 — 호출자(fuse)가 커맨드-적분 쪽으로만 폴백하도록.
    """
    if cv2 is None or np is None or H is None or prev_gray is None or curr_gray is None:
        return _vo_empty()

    gf = vo_cfg["goodfeatures"]
    prev_pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, **gf)
    if prev_pts is None or len(prev_pts) < vo_cfg["vo_min_features"]:
        return _vo_empty()

    curr_pts, status, _err = cv2.calcOpticalFlowPyrLK(
        prev_gray, curr_gray, prev_pts, None,
        winSize=tuple(vo_cfg["lk_win_size"]), maxLevel=vo_cfg["lk_max_level"])
    if curr_pts is None or status is None:
        return _vo_empty()

    ok_mask = status.reshape(-1).astype(bool)
    prev_ok = prev_pts[ok_mask].reshape(-1, 2)
    curr_ok = curr_pts[ok_mask].reshape(-1, 2)
    n_tracked = len(prev_ok)
    if n_tracked < vo_cfg["vo_min_features"]:
        return _vo_empty(n_tracked=n_tracked)

    prev_ground = _project_ground(prev_ok, H)
    curr_ground = _project_ground(curr_ok, H)
    valid = (np.isfinite(prev_ground).all(axis=1) & np.isfinite(curr_ground).all(axis=1) &
             (np.linalg.norm(prev_ground, axis=1) < _MAX_GROUND_RANGE_M) &
             (np.linalg.norm(curr_ground, axis=1) < _MAX_GROUND_RANGE_M))
    prev_ground, curr_ground = prev_ground[valid], curr_ground[valid]
    if len(prev_ground) < vo_cfg["vo_min_features"]:
        return _vo_empty(n_tracked=n_tracked)

    # curr→prev 방향으로 적합: 정지된 지면점이 차량 로컬 프레임에서 어떻게
    # "되돌아가 보이는지"가 곧 차량 자신의 이동(R(dtheta), (dx,dy))이다
    # (p_prev = R(dtheta)@p_curr + t — compose_pose의 합성 규약과 짝을 이룸).
    M, inliers = cv2.estimateAffinePartial2D(
        curr_ground.astype(np.float32), prev_ground.astype(np.float32), method=cv2.RANSAC)
    if M is None:
        return _vo_empty(n_tracked=n_tracked)
    n_inliers = int(inliers.sum()) if inliers is not None else 0
    if n_inliers < vo_cfg["vo_min_inliers"]:
        return _vo_empty(n_tracked=n_tracked, n_inliers=n_inliers)

    dtheta = math.atan2(M[1, 0], M[0, 0])
    return dict(dx_m=float(M[0, 2]), dy_m=float(M[1, 2]), dtheta_rad=dtheta,
                n_tracked=n_tracked, n_inliers=n_inliers, ok=True)


def steer_dedup_pulse(direction, last_direction):
    """ArduinoNode.steer()의 dedup 규칙(_send_once — 같은 값 연속 호출은 전송 안 함)을
    그대로 흉내낸다. 실제로 "전송"됐고 방향이 L/R이면 물리 펄스 1회로 센다
    (F는 조향 모터 정지일 뿐 펄스가 아니다). last_direction 초기값은 None
    (ArduinoNode._last 딕셔너리가 비어서 시작하는 것과 동일).

    반환: (pulse_occurred: bool, new_last_direction)."""
    if direction not in ("F", "L", "R"):
        direction = "F"
    if direction == last_direction:
        return False, last_direction
    return direction in ("L", "R"), direction


def apply_steer_pulse(direction, pulse_state_deg, deg_per_pulse, steering_limit_deg):
    """L/R 펄스 1회를 조향각 적분 상태(pulse_state_deg)에 반영, ±steering_limit_deg로
    클램프. deg_per_pulse가 0/None(미측정)이면 상태가 안 바뀐다 — 모르는 조향각을
    함부로 추정하지 않고 직진(0도)으로 취급하는 fail-inert 동작.

    반환은 항상 float로 고정한다 — steering_limit_deg가 int로 들어오면(예:
    ros2 param set으로 정수 설정) 클램프된 결과가 int로 오염될 수 있다
    (arduino_node.adc_to_deg에서 실차로 확인된 것과 같은 종류의 버그)."""
    if not deg_per_pulse:
        return pulse_state_deg
    if direction == "L":
        pulse_state_deg += deg_per_pulse
    elif direction == "R":
        pulse_state_deg -= deg_per_pulse
    else:
        return pulse_state_deg
    return float(max(-steering_limit_deg, min(steering_limit_deg, pulse_state_deg)))


def select_steering_angle_deg(pot_angle_deg, pot_span_counts, pulse_state_deg, odo_cfg):
    """가능하면 POT 실측 각도(/car/steering_angle)를, 아니면(미검출 또는
    캘리브레이션 스윙이 config.ODOMETRY.min_pot_span_counts보다 좁음) 펄스 카운트
    적분값으로 폴백한다. arduino_node 자체의 min_span(POT "장착 여부" 판정, 기본
    3)과는 별개의, 더 엄격한 "각도 신뢰" 기준 — 2026-07 실측 링크 커플링은 스윙이
    ADC 4카운트뿐이라 min_span=3은 통과해도 각도 분해능은 못 믿을 수준이다.

    반환: (선택된 각도 deg, 출처 'pot'|'pulse')."""
    if (pot_angle_deg is not None and pot_span_counts is not None
            and pot_span_counts >= odo_cfg["min_pot_span_counts"]):
        return pot_angle_deg, "pot"
    return pulse_state_deg, "pulse"


def integrate_command_motion(speed_pwm, steer_angle_deg, dt_s, wheelbase_m, odo_cfg):
    """자전거 모델(bicycle model) 1스텝 — 논문의 데드레커닝 항을 대신하는
    "커맨드-적분"(실제 바퀴 회전이 아니라 명령값의 적분). pwm_to_mps가
    미측정(None)이면 정확히 (0,0,0) — 잘못된 암묵적 기본값 대신 확실한 비활성."""
    pwm_to_mps = odo_cfg.get("pwm_to_mps")
    if not pwm_to_mps:
        return 0.0, 0.0, 0.0
    v = speed_pwm * pwm_to_mps
    dtheta = (v / wheelbase_m) * math.tan(math.radians(steer_angle_deg)) * dt_s
    dx = v * dt_s * math.cos(dtheta / 2.0)
    dy = v * dt_s * math.sin(dtheta / 2.0)
    return dx, dy, dtheta


def fuse(pose_kf, vo, cmd, dt, odo_cfg):
    """VO(estimate_visual_motion 반환값)와 커맨드-적분((dx,dy,dtheta) 튜플, 현재
    헤딩 기준 차량 프레임 로컬 증분)을 pose_kf(filters.PoseKalmanFilter — 노드가
    프레임 간 지속시키는 실제 x/y/theta 상태)에 칼만필터로 융합한다.

    2026-07-18: 매 프레임 n_inliers만으로 confidence를 처음부터 다시 계산하던
    상보 필터(가중치가 weight_max에서 바로 포화되고, 이전 프레임의 신뢰 이력이
    전혀 반영 안 됐음)에서 전환 — 커맨드-적분을 predict(제어입력, 프로세스
    노이즈는 dt에 비례해 커짐)로, VO를 update(측정, 노이즈는 인라이어 수가
    많을수록 계속 작아짐)로 쓰는 x/y/theta 축별 독립 칼만필터로 바꿨다. P
    (분산)가 프레임 간 지속되므로 VO가 계속 잘 맞으면 점점 확신이 커지고,
    끊기면(vo["ok"]=False, predict만 하고 넘어감) 점점 다시 불확실해진다 —
    기존 방식엔 없던 감쇠 거동.

    커맨드/VO 로컬 증분은 둘 다 같은 이전 헤딩(theta0)으로 전역 프레임에 회전
    시킨 뒤 각 축에 합산한다(compose_pose와 동일한 회전 합성 규약).

    반환: (x, y, theta, confidence) — 이번 틱 융합 pose와 0..kf_conf_max 신뢰도.
    confidence 의미는 기존과 동일(POSE_STALE_S 감시/min_pose_conf 게이팅에
    쓰는 소비자 계약 불변) — CAMERA_MOUNT/pwm_to_mps가 미측정이면 P가 초기값
    (kf_p0_pos/kf_p0_theta, 매우 큼)에서 전혀 안 줄어들어 confidence≈0으로
    fail-inert 동작(기존과 동일한 보장)."""
    x0, y0, theta0 = pose_kf.pose
    cmd_dx, cmd_dy, cmd_dtheta = cmd
    c, s = math.cos(theta0), math.sin(theta0)
    cmd_dx_g = c * cmd_dx - s * cmd_dy
    cmd_dy_g = s * cmd_dx + c * cmd_dy

    pose_kf.x.predict(odo_cfg["kf_q_pos"] * dt, control=cmd_dx_g)
    pose_kf.y.predict(odo_cfg["kf_q_pos"] * dt, control=cmd_dy_g)
    pose_kf.theta.predict(odo_cfg["kf_q_theta"] * dt, control=cmd_dtheta)

    if vo["ok"]:
        vo_dx_g = c * vo["dx_m"] - s * vo["dy_m"]
        vo_dy_g = s * vo["dx_m"] + c * vo["dy_m"]
        min_inliers = odo_cfg["vo_min_inliers"]
        n = max(vo["n_inliers"], min_inliers)  # ok=True는 이미 n_inliers>=min_inliers를 보장(안전망)
        r_pos = max(odo_cfg["kf_r_pos_base"] * (min_inliers / n), odo_cfg["kf_r_pos_floor"])
        r_theta = max(odo_cfg["kf_r_theta_base"] * (min_inliers / n), odo_cfg["kf_r_theta_floor"])
        pose_kf.x.update(x0 + vo_dx_g, r_pos)
        pose_kf.y.update(y0 + vo_dy_g, r_pos)
        pose_kf.theta.update(theta0 + vo["dtheta_rad"], r_theta)

    def _axis_confidence(variance, p_ref):
        return p_ref / (p_ref + variance)

    confidence = odo_cfg["kf_conf_max"] * min(
        _axis_confidence(pose_kf.x.variance(), odo_cfg["kf_conf_p_ref_pos"]),
        _axis_confidence(pose_kf.y.variance(), odo_cfg["kf_conf_p_ref_pos"]),
        _axis_confidence(pose_kf.theta.variance(), odo_cfg["kf_conf_p_ref_theta"]))
    return pose_kf.x.value(), pose_kf.y.value(), pose_kf.theta.value(), confidence


def compose_pose(x, y, theta, dx, dy, dtheta):
    """로컬 증분(dx, dy: 현재 헤딩 기준 차량 프레임, dtheta)을 전역 상대 pose에
    누적한다. 미션 시작(또는 마지막 reset)을 원점으로 하는 상대 좌표일 뿐 —
    전역/GPS 기준이 아니다.

    fuse()는 더 이상 이 함수를 호출하지 않는다(각 축을 독립 칼만필터로 갱신
    하며 같은 회전 합성을 인라인으로 한다) — 하지만 회전 합성 규약 자체가
    동일해야 하므로(그리고 다른 잠재적 호출부를 위해) 순수 함수로 남겨둔다."""
    c, s = math.cos(theta), math.sin(theta)
    x2 = x + c * dx - s * dy
    y2 = y + s * dx + c * dy
    theta2 = theta + dtheta
    return x2, y2, theta2


# ============================ ROS2 래퍼 ============================

def ros_main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Float32, Int8, Int16, Int32, String
    from geometry_msgs.msg import PoseStamped

    from .. import config, tuning
    from .arduino_node import STATE_UNKNOWN

    class OdometryNode(Node):
        """/camera/front + /car/cmd/*, /car/state, /car/steering_angle,
        /car/steering_pot_span을 구독해 전방 프레임이 도착할 때마다 VO+커맨드-적분을
        융합, 상대 pose를 /car/pose(PoseStamped, frame_id="odom"), 신뢰도를
        /car/pose_confidence(Float32)로 발행한다.

        config.CAMERA_MOUNT/ODOMETRY.pwm_to_mps가 미측정인 동안은 confidence=0,
        거의 정지 상태의 pose만 나온다 — 의도된 fail-inert 동작(POT 미장착 시
        calibrate_steering()이 조용히 스킵되는 것과 같은 패턴)."""

        def __init__(self):
            super().__init__("odometry_node")

            self._split = config.CAMERA_SPLIT
            self._prev_gray = None
            self._H = None
            self._H_shape = None
            self._last_time = None
            self._pose_kf = filters.PoseKalmanFilter(
                config.ODOMETRY["kf_p0_pos"], config.ODOMETRY["kf_p0_theta"])
            self.x, self.y, self.theta = self._pose_kf.pose

            self._car_state = None
            self._last_drive_cmd = 0
            self._last_steer_dir = None
            self._pulse_state_deg = 0.0
            self._pot_angle_deg = None
            self._pot_span = None

            self.create_subscription(CompressedImage, "/camera/front", self._on_front, 10)
            self.create_subscription(Int8, "/car/state", self._on_state, 10)
            self.create_subscription(Int16, "/car/cmd/drive", self._on_drive_cmd, 10)
            self.create_subscription(String, "/car/cmd/steer", self._on_steer_cmd, 10)
            self.create_subscription(String, "/car/cmd/steer_pulse", self._on_steer_pulse_cmd, 10)
            self.create_subscription(Float32, "/car/steering_angle", self._on_pot_angle, 10)
            self.create_subscription(Int32, "/car/steering_pot_span", self._on_pot_span, 10)

            self._pose_pub = self.create_publisher(PoseStamped, "/car/pose", 10)
            self._conf_pub = self.create_publisher(Float32, "/car/pose_confidence", 10)

            # 실차 캘리브레이션 파라미터 — CAMERA_MOUNT(높이/틸트)와 ODOMETRY
            # (pwm_to_mps 등)를 rebuild 없이 ros2 param set으로 넣을 수 있게
            # 노출한다 (0.0=미측정 규약). camera_mount가 바뀌면 캐시된
            # 호모그래피(_H)를 무효화해 다음 프레임에 재계산한다.
            tuning.install(self, tuning.odometry_tunable_dicts(),
                           on_change=self._on_tuning_change)

        def _on_tuning_change(self, param_name):
            if param_name.startswith("camera_mount."):
                self._H = None

        def _on_state(self, msg):
            self._car_state = None if msg.data == STATE_UNKNOWN else msg.data

        def _on_drive_cmd(self, msg):
            self._last_drive_cmd = msg.data

        def _on_steer_cmd(self, msg):
            pulsed, self._last_steer_dir = steer_dedup_pulse(msg.data, self._last_steer_dir)
            if pulsed:
                self._pulse_state_deg = apply_steer_pulse(
                    msg.data, self._pulse_state_deg,
                    config.ODOMETRY["deg_per_pulse"], config.STEERING_LIMIT_DEG)

        def _on_steer_pulse_cmd(self, msg):
            self._pulse_state_deg = apply_steer_pulse(
                msg.data, self._pulse_state_deg,
                config.ODOMETRY["deg_per_pulse"], config.STEERING_LIMIT_DEG)

        def _on_pot_angle(self, msg):
            self._pot_angle_deg = msg.data

        def _on_pot_span(self, msg):
            self._pot_span = msg.data

        def _effective_speed_pwm(self):
            """firmware가 실제로 적용한 부호 있는 속도를 재구성한다 — 원시
            /car/cmd/drive 값만으로는 canGo 게이트/워치독으로 무시된 상태(=0)를
            알 수 없어 /car/state(펌웨어가 실제 적용한 속도의 부호로 만든 값)와
            조합한다."""
            if self._car_state in (None, 0):
                return 0
            magnitude = abs(self._last_drive_cmd)
            return magnitude if self._car_state == 1 else -magnitude

        def _on_front(self, msg):
            if cv2 is None or np is None:
                return
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return
            h = frame.shape[0]
            bottom = frame[h // 2:, :] if self._split else frame

            now = Time.from_msg(msg.header.stamp)
            dt = None if self._last_time is None else (now - self._last_time).nanoseconds / 1e9
            self._last_time = now

            gray = cv2.cvtColor(bottom, cv2.COLOR_BGR2GRAY)
            bh, bw = gray.shape[:2]
            if self._H is None or self._H_shape != (bw, bh):
                self._H = ground_projection_matrix(
                    config.CAMERA_MOUNT, config.FRONT_CAMERA_SENSOR_HFOV_DEG,
                    config.FRONT_CAMERA_MOUNT_VFOV_DEG, bw, bh, crop_top_px=h // 2)
                self._H_shape = (bw, bh)

            vo = estimate_visual_motion(self._prev_gray, gray, self._H, config.ODOMETRY)
            self._prev_gray = gray

            confidence = 0.0
            if dt is not None and dt > 0:
                steer_deg, _source = select_steering_angle_deg(
                    self._pot_angle_deg, self._pot_span, self._pulse_state_deg, config.ODOMETRY)
                cmd = integrate_command_motion(
                    self._effective_speed_pwm(), steer_deg, dt, config.WHEELBASE_M, config.ODOMETRY)
                self.x, self.y, self.theta, confidence = fuse(
                    self._pose_kf, vo, cmd, dt, config.ODOMETRY)

            self._publish(msg.header.stamp, confidence)

        def _publish(self, stamp, confidence):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "odom"
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.orientation.z = math.sin(self.theta / 2.0)
            pose.pose.orientation.w = math.cos(self.theta / 2.0)
            self._pose_pub.publish(pose)
            self._conf_pub.publish(Float32(data=confidence))

    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ========================= 오프라인 테스트 / 셀프테스트 =========================

def selftest():
    """ROS/카메라/시리얼 없이 순수 함수(자전거 모델 적분, 펄스 dedup, POT/펄스
    선택, 칼만필터 융합, pose 합성, 지면 투영 호모그래피, VO)만 검증한다."""
    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    inert_cfg = dict(pwm_to_mps=None, deg_per_pulse=None, min_pot_span_counts=8,
                      vo_min_features=15, vo_min_inliers=10,
                      goodfeatures=dict(maxCorners=200, qualityLevel=0.01, minDistance=7, blockSize=7),
                      lk_win_size=(21, 21), lk_max_level=3,
                      kf_p0_pos=1e6, kf_p0_theta=1e6, kf_q_pos=0.01, kf_q_theta=0.02,
                      kf_r_pos_base=0.01, kf_r_pos_floor=0.0009,
                      kf_r_theta_base=0.02, kf_r_theta_floor=0.0009,
                      kf_conf_p_ref_pos=0.01, kf_conf_p_ref_theta=0.01, kf_conf_max=0.8)
    cal_cfg = dict(inert_cfg, pwm_to_mps=0.01, deg_per_pulse=2.0)

    # ---- integrate_command_motion ----
    check("미보정(pwm_to_mps=None)이면 커맨드-적분은 항상 (0,0,0)",
          integrate_command_motion(100, 10.0, 0.1, 0.545, inert_cfg) == (0.0, 0.0, 0.0))

    dx, dy, dtheta = integrate_command_motion(100, 0.0, 0.1, 0.545, cal_cfg)
    v = 100 * cal_cfg["pwm_to_mps"]
    check("직진(조향각 0)이면 dtheta=0, dx≈v*dt",
          abs(dtheta) < 1e-12 and abs(dx - v * 0.1) < 1e-9 and abs(dy) < 1e-12)

    total_dx = 0.0
    for _ in range(10):
        step_dx, _dy, _dtheta = integrate_command_motion(100, 0.0, 0.1, 0.545, cal_cfg)
        total_dx += step_dx
    check("직진 10틱 누적 dx ≈ v*dt*10", abs(total_dx - v * 0.1 * 10) < 1e-9)

    _dx, _dy, dtheta_left = integrate_command_motion(100, 10.0, 0.1, 0.545, cal_cfg)
    check("전진 + 좌조향(+각도)이면 dtheta>0 (좌회전, CCW 양수 규약)", dtheta_left > 0)

    _dx, _dy, dtheta_rev = integrate_command_motion(-100, 10.0, 0.1, 0.545, cal_cfg)
    check("같은 조향각이라도 후진(speed<0)이면 dtheta 부호가 반전",
          dtheta_rev < 0 and abs(dtheta_rev + dtheta_left) < 1e-12)

    # ---- steer_dedup_pulse / apply_steer_pulse ----
    pulsed, last = steer_dedup_pulse("L", None)
    check("첫 L 펄스는 전송+카운트", pulsed and last == "L")
    pulsed2, last2 = steer_dedup_pulse("L", last)
    check("같은 방향 연속 호출은 dedup(전송 안 함)", not pulsed2 and last2 == "L")
    pulsed3, last3 = steer_dedup_pulse("F", last2)
    check("F는 전송은 되지만 펄스로 안 셈", not pulsed3 and last3 == "F")
    pulsed4, _last4 = steer_dedup_pulse("R", last3)
    check("F 이후 R은 다시 펄스로 카운트", pulsed4)

    state = 0.0
    state = apply_steer_pulse("L", state, 2.0, 20)
    state = apply_steer_pulse("L", state, 2.0, 20)
    check("L 펄스 2회 누적 == +4도", abs(state - 4.0) < 1e-9)
    for _ in range(20):
        state = apply_steer_pulse("L", state, 2.0, 20)
    check("조향각 누적이 steering_limit_deg(20)에서 클램프", state == 20.0)
    check("deg_per_pulse 미측정(0/None)이면 상태 불변",
          apply_steer_pulse("L", 5.0, None, 20) == 5.0)

    # ---- select_steering_angle_deg ----
    angle, source = select_steering_angle_deg(15.0, 40, -3.0, inert_cfg)
    check("POT 스윙이 충분히 넓으면 POT 각도 사용", angle == 15.0 and source == "pot")
    angle2, source2 = select_steering_angle_deg(15.0, 4, -3.0, inert_cfg)
    check("2026-07-16 실측(ADC 스윙 4카운트, 임계 8 미달) → 펄스 폴백",
          angle2 == -3.0 and source2 == "pulse")
    angle3, source3 = select_steering_angle_deg(None, None, -3.0, inert_cfg)
    check("POT 미검출(None)이면 펄스 폴백", angle3 == -3.0 and source3 == "pulse")

    # ---- fuse (칼만필터 융합) ----
    cmd = (1.0, 0.0, 0.0)  # 회전 없는 직진 성분만 -> 전역/로컬 좌표가 같아 검증 단순
    pose_kf = filters.PoseKalmanFilter(cal_cfg["kf_p0_pos"], cal_cfg["kf_p0_theta"])
    x, y, theta, conf = fuse(pose_kf, _vo_empty(ok=False), cmd, 0.1, cal_cfg)
    check("VO ok=False(미검출)인 첫 틱 -> confidence는 초기 분산이 커서 거의 0",
          conf < 1e-3)

    for _ in range(20):
        x, y, theta, conf = fuse(pose_kf, _vo_empty(ok=False), cmd, 0.1, cal_cfg)
    check("VO가 계속 안 잡히면(커맨드-적분만) confidence는 계속 ~0 유지 (fail-inert 보존)",
          conf < 1e-3)

    vo_full = dict(dx_m=2.0, dy_m=0.0, dtheta_rad=0.0,
                   n_tracked=50, n_inliers=cal_cfg["vo_min_inliers"] * 5, ok=True)
    for _ in range(15):
        x, y, theta, conf_good = fuse(pose_kf, vo_full, cmd, 0.1, cal_cfg)
    check("인라이어 충분한 VO가 이어지면 confidence가 유의미하게 상승(기존 fail-inert 상태에서 벗어남)",
          conf_good > 0.3)
    check("확신이 오른 뒤 x는 커맨드(1.0/틱)보다 VO(2.0/틱) 쪽에 더 가깝게 수렴",
          abs(x - 15 * vo_full["dx_m"]) < abs(x - 15 * cmd[0]))

    conf_before_dropout = conf_good
    for _ in range(20):
        x, y, theta, conf_decayed = fuse(pose_kf, _vo_empty(ok=False), cmd, 0.1, cal_cfg)
    check("VO가 다시 여러 틱 끊기면 confidence가 이전 확신에서 다시 감소함 "
          "(상보 필터엔 없던 감쇠 거동 — 이번 전환의 핵심)",
          conf_decayed < conf_before_dropout)

    # ---- compose_pose ----
    x, y, theta = 0.0, 0.0, 0.0
    x, y, theta = compose_pose(x, y, theta, 1.0, 0.0, 0.0)  # 1m 직진
    x, y, theta = compose_pose(x, y, theta, 0.0, 0.0, math.pi / 2)  # 제자리 좌회전 90도
    x, y, theta = compose_pose(x, y, theta, 1.0, 0.0, 0.0)  # 새 헤딩(+Y)으로 1m 직진
    check("직진1m→좌회전90도→직진1m 시퀀스 → (1,1,90도)",
          abs(x - 1.0) < 1e-9 and abs(y - 1.0) < 1e-9 and abs(theta - math.pi / 2) < 1e-9)

    # ---- ground_projection_matrix ----
    check("CAMERA_MOUNT 미측정(height/tilt=None)이면 H=None",
          ground_projection_matrix(dict(height_m=None, tilt_deg=None), 78, 78, 320, 240) is None)

    if cv2 is None or np is None:
        print("[!!] opencv/numpy 미설치 — VO 관련 셀프테스트 스킵")
    else:
        H = ground_projection_matrix(dict(height_m=0.5, tilt_deg=15.0), 78, 78, 320, 240,
                                      crop_top_px=240)
        check("실측치가 있으면 H는 3x3 행렬", H is not None and H.shape == (3, 3))

        vo_cfg = dict(cal_cfg, vo_min_features=15, vo_min_inliers=8)
        rng = np.random.default_rng(0)
        speckle = rng.integers(0, 256, size=(240, 320), dtype=np.uint8)

        check("H=None이면 프레임이 있어도 VO는 ok=False",
              estimate_visual_motion(speckle, speckle, None, vo_cfg)["ok"] is False)
        check("첫 프레임(prev_gray=None)이면 ok=False",
              estimate_visual_motion(None, speckle, H, vo_cfg)["ok"] is False)

        blank = np.zeros((240, 320), dtype=np.uint8)
        check("무특징(빈 프레임) 쌍이면 ok=False",
              estimate_visual_motion(blank, blank, H, vo_cfg)["ok"] is False)

        vo_same = estimate_visual_motion(speckle, speckle, H, vo_cfg)
        check("완전히 같은 프레임(무운동)이면 ok=True, 이동량 ≈0",
              vo_same["ok"] and abs(vo_same["dx_m"]) < 1e-6 and
              abs(vo_same["dy_m"]) < 1e-6 and abs(vo_same["dtheta_rad"]) < 1e-6)

        def synth_frame(prev, H, dx, dy, dtheta):
            """H와 알려진 지면상 강체 이동(dx,dy,dtheta)으로부터, 그 이동이 실제로
            일어났다면 다음 프레임이 어떻게 보일지를 픽셀 공간 호모그래피
            (H^-1 @ T @ H)로 정확히 렌더링한다. 단순 픽셀 평행이동으로 만든 프레임은
            원근 투영 하에서 거리별로 다른 픽셀 이동량을 갖는 진짜 강체 이동과
            달라(먼 점일수록 픽셀 이동이 작다) 검증용으로 부적절 — 이 방식으로
            대체(2026-07-16, 균일 픽셀 시프트로 첫 시도했을 때 estimateAffinePartial2D가
            스케일까지 적합해버려 부호를 오판독하는 문제를 실측으로 확인)."""
            c, s = math.cos(dtheta), math.sin(dtheta)
            tx, ty = -(c * dx + s * dy), -(-s * dx + c * dy)
            T = np.array([[c, s, tx], [-s, c, ty], [0.0, 0.0, 1.0]])
            W = np.linalg.inv(H) @ T @ H
            return cv2.warpPerspective(prev, (W / W[2, 2]).astype(np.float64), (320, 240),
                                        borderMode=cv2.BORDER_REFLECT101)

        vo_fwd = estimate_visual_motion(speckle, synth_frame(speckle, H, 0.08, 0.0, 0.0), H, vo_cfg)
        check("전진 8cm 합성 → dx_m>0(전방), dy_m/dtheta는 작음",
              vo_fwd["ok"] and vo_fwd["dx_m"] > 0.02 and
              abs(vo_fwd["dy_m"]) < 0.02 and abs(vo_fwd["dtheta_rad"]) < 0.05)

        vo_left = estimate_visual_motion(speckle, synth_frame(speckle, H, 0.0, 0.05, 0.0), H, vo_cfg)
        check("좌측 5cm 합성 → dy_m>0(좌측, +가 좌측 규약), dx_m/dtheta는 작음",
              vo_left["ok"] and vo_left["dy_m"] > 0.02 and
              abs(vo_left["dx_m"]) < 0.02 and abs(vo_left["dtheta_rad"]) < 0.05)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    ros_main()
