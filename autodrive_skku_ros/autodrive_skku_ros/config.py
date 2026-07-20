# 차량/센서 설정 — 새 환경에서는 이 파일(또는 bringup.launch.py 인자)만 바꾸면 된다.
# 포트가 None이면 자동 감지를 시도한다. 실패 시 arduino_port/lidar_port launch 인자로 지정.
import os

ARDUINO_PORT = None      # 예: "/dev/ttyACM0"
ARDUINO_BAUD = 9600
LIDAR_PORT = None        # 예: "/dev/ttyUSB0"
LIDAR_BAUD = 115200

# 카메라: 물리적으로 2대(전방/후방) — camera_node는 /camera/front, /camera/back만
# 발행하고, 전방 프레임을 신호등용(top)/차선용(bottom)으로 나누는 건 detection
# 쪽(mission_node)이 담당한다.
FRONT_CAMERA = 0         # /dev/video0
REAR_CAMERA = None       # T주차용 후방 카메라 인덱스. 없으면 None
CAMERA_SPLIT = True      # False면 mission_node가 top/bottom 양쪽에 원본 프레임을 그대로 전달
FRAME_WIDTH = 640        # 캡처 요청 해상도 — 센서 네이티브(랜드스케이프) 기준, 회전 전
FRAME_HEIGHT = 480

# 전방 카메라는 파노라믹(가로/landscape)으로 물리 마운트 — 세로(portrait) 마운트는
# 폐기(2026-07-16). 회전 보정 불필요.
FRONT_CAMERA_ROTATE = None  # None | "CW" | "CCW" | "180"

LOOP_HZ = 30             # 메인 제어 루프 주기
POSE_STALE_S = 0.5       # 이 시간 이상 /car/pose 갱신 없으면 pose_conf 강제 0 (odometry_node 중단 감시)

# 디지털 트윈 재현용 주행 로그(drive_logger.py) — mission_node가 매 틱 튜닝
# 설정값+발행 명령을 타임스탬프와 함께 JSON Lines로 남기는 기본 저장 위치
# (지도 교수 피드백, 2026-07-18). launch log_dir 인자로 덮어쓸 수 있다.
DRIVE_LOG_DIR = os.path.expanduser("~/autodrive_skku_logs")

# teleop_node 실행 중 /camera/front, /camera/back을 자동으로 mp4로 저장하는 위치.
# 실제 수신 프레임 간격과 무관하게 이 FPS로 인코딩한다(대략치 — 정확한 타이밍이
# 필요하면 나중에 프레임 타임스탬프 기반 가변 FPS로 바꿀 것).
TELEOP_RECORD_DIR = os.path.expanduser("~/autodrive_skku_logs/teleop_recordings")
TELEOP_RECORD_FPS = 20.0

DRIVE_SPEED = 100        # 기본 주행 속도 (-255..255, 실차 검증값)
SLOW_SPEED = 60          # 주차 등 저속 기동 속도

# ---- RP라이다 후방 장착 외부 파라미터 (2026-07-09 장착 실측) ----
# 회전축: 차체 후단에서 뒤로 75mm, 중심선(y=0), 지면에서 140mm — 다른 차량의
# 바퀴/차체 하부가 걸리는 높이. T주차 후진용 센서로, 각도 규약은 0도=차량 후방.
# 전방은 자차 차체에 막힘(전방 기준 ±~75도 wedge) → 전방 장애물 감지는
# 카메라(OBSTACLE_CAM)가 담당한다. sim/models/av_car/model.sdf 트윈과 동일.
LIDAR_MOUNT = dict(
    yaw_offset_deg=0.0,   # 라이다 원시 0도 ↔ 차량 후방 사이 오프셋 (장착 후 캘리브레이션)
    invert=False,         # 라이다 각도 증가 방향이 차량 기준(CCW)과 반대면 True
    to_rear_m=0.075,      # 뒤 범퍼 기준 거리 = 라이다 거리 + 0.075 (축이 범퍼 뒤에 있음)
)
LIDAR_SELF_MASK_DEG = 75.0   # 차량 전방 기준 |bearing| < 이 값 → 자차 차체 반사, 제거
LIDAR_REAR_SECTOR = 30       # 후방 ±N도 (T주차 후진 거리 판정에 사용)
LIDAR_SIDE_WINDOW_DEG = (75.0, 100.0)  # 좌/우 여유 판정 창 (차체 실루엣 밖 각도만)

# (구) 라이다 전방 장애물 정지 거리 — 후방 장착 이후 전방 감지는 카메라로 대체됨.
# 값은 회피 판단 기준 거리 감각(700mm)의 기록용으로만 유지.
OBSTACLE_STOP_M = 0.7

# 미션별 튜닝값(장애물 감지, 차선 변경, 정지선, T주차, 차선 인식, 신호등 판정)은
# 각 미션 파일 상단으로 옮겼다 — 소비하는 곳 근처에 두는 게 더 직관적이라는 판단
# (auto_ws 스타일 참고). road.py: OBSTACLE_CAM/LANE_CHANGE, lane_follow.py:
# LANE_EDGE/LANE_POI, traffic.py: STOP_LINE/TRAFFIC_PIXEL_RATIO,
# t_parking.py: T_PARKING. 여러 미션이 공유하는 값(흰색 임계, 펄스 주기)만
# 아래에 단일 소스로 둔다.

# 대회 규격 흰색(차선/정지선/주차선/장애물 차량 전부 동일 규격) HSV 임계 —
# 단일 소스. 감지기별로 다르게 찍히면 각 감지기 dict의 white_s_max/white_v_min
# override 키(None=이 값 사용)로 개별 조정한다.
WHITE_HSV = dict(s_max=60, v_min=180)

# steer_pulse() 반복 전송 최소 간격(초) — 차선 변경(road)과 T주차(t_parking)가
# 공유한다 (같은 액추에이터 특성이라 따로 튜닝할 이유가 없음).
STEER_PULSE_GAP_S = 0.15


def white_hsv(detector_cfg):
    """감지기 dict의 white_s_max/white_v_min override가 설정돼 있으면 그 값,
    None이면 공유 WHITE_HSV를 쓴다. (s_max, v_min) 반환."""
    return (detector_cfg.get("white_s_max") or WHITE_HSV["s_max"],
            detector_cfg.get("white_v_min") or WHITE_HSV["v_min"])

# 참고용 실측 차량 제원 (WSL ~/autonomousAIdrive/sim/models/av_car/model.sdf,
# kinematic_single_track_parameters.md 기반 측정치). 현재 조향은 120ms 펄스
# 방식이라 아래 값이 아직 제어 로직에 쓰이진 않음 — 차선 변경 궤적 계산 등
# 필요해질 때 참고.
WHEELBASE_M = 0.545
TRACK_WIDTH_M = 0.430
WHEEL_RADIUS_M = 0.10
STEERING_LIMIT_DEG = 20.0  # float 고정 — 정수면 각도 클램프(min/max) 결과가 int로
                            # 오염돼 Float32 발행 시 타입 assert로 죽는다(2026-07-17 실차)
FRONT_CAMERA_SENSOR_HFOV_DEG = 78   # Logitech C920 spec (센서 자체, landscape 기준)
FRONT_CAMERA_MOUNT_VFOV_DEG = 49    # 파노라믹(landscape) 마운트 실효 수직화각 (sim 기준 vFOV)

# ---- odometry_node 지면 투영 외부 파라미터 (미측정). model.sdf에 카메라 z가
# 0.47(계산치)/0.795(실측치, 규정 75cm 마운트 상한 초과로 플래그됨) 두 값으로
# 남아있어 서로 불일치 — 실차 측정 시 어느 쪽도 그대로 믿지 말고 새로 잴 것.
# height_m/tilt_deg가 None이면 estimate_visual_motion()이 항상 ok=False로
# 비활성 동작한다 (미장착 라이다 판정과 같은 fail-inert 방식).
CAMERA_MOUNT = dict(height_m=None, tilt_deg=None)

# ---- odometry_node 융합 파라미터. pwm_to_mps 미측정 시 커맨드-적분 항은 항상
# 0(정지)으로 취급된다 — PWM/조향 매핑과 마찬가지로 실차 튜닝 대상.
# min_pot_span_counts: arduino_node의 자체 min_span(POT "장착 여부" 판정, 기본 3)과
# 별개로, odometry가 POT 각도를 "신뢰"할지 정하는 자체 기준. 2026-07-16 실측
# 링크 커플링은 풀락 스윙이 ADC 4카운트뿐이라(min_span=3은 통과) 캘리브레이션
# 자체는 성공하지만 각도 분해능이 너무 거칠다 — 이 임계값 아래면 POT 각도를
# 쓰지 않고 펄스 카운트 적분으로 폴백한다. 링크→기어 교체 후 스윙이 넓어지면
# 자동으로 POT 쪽을 쓰게 된다(코드 변경 불필요).
ODOMETRY = dict(
    pwm_to_mps=None,
    deg_per_pulse=None,   # POT 폴백(펄스 카운트 적분)용. 미측정 시 조향각 0(직진)으로 취급
    min_pot_span_counts=8,
    vo_min_features=15,
    vo_min_inliers=10,
    goodfeatures=dict(maxCorners=200, qualityLevel=0.01, minDistance=7, blockSize=7),
    lk_win_size=(21, 21),
    lk_max_level=3,
    # ---- 칼만필터 융합(fuse(), 2026-07-18 — 상보 필터에서 전환) 파라미터.
    # kf_p0_*: 초기 분산(=사실상 미신뢰) — 크게 잡아 VO가 한 번도 성공 못 하면
    # (CAMERA_MOUNT 미측정 등) confidence가 계속 0에 가깝게 유지되게 한다
    # (기존 "미측정 시 confidence=0" 보장과 동일). kf_q_*: 예측(커맨드-적분)
    # 노이즈 — 클수록 P가 프레임마다 더 빨리 커져 VO를 더 신뢰. kf_r_*_base/
    # floor: VO 측정 노이즈 — 인라이어가 많을수록 base에서 floor까지 줄어듦.
    # kf_conf_*: confidence(0..kf_conf_max) 계산용 — p_ref는 "이 정도 분산이면
    # 중간 정도 신뢰"의 기준값. 전부 📏 실차 튜닝 대상(기존 fusion_vo_weight_max
    # 자리를 대신함 — min_pose_conf 게이팅 임계는 그대로 유효).
    kf_p0_pos=1e6,          # m^2
    kf_p0_theta=1e6,        # rad^2
    kf_q_pos=0.01,          # m^2/s 📏
    kf_q_theta=0.02,        # rad^2/s 📏
    kf_r_pos_base=0.01,     # m^2 📏
    kf_r_pos_floor=0.0009,  # m^2 📏
    kf_r_theta_base=0.02,   # rad^2 📏
    kf_r_theta_floor=0.0009,  # rad^2 📏
    kf_conf_p_ref_pos=0.01,    # m^2
    kf_conf_p_ref_theta=0.01,  # rad^2
    kf_conf_max=0.8,        # 기존 fusion_vo_weight_max와 동일한 상한/의미
)

# ---- arduino_node 라이브 조향 POT 스트림 칼만필터 (ADC count 공간,
# adc_to_deg() 변환 이전에 적용). calibrate_steering()의 _read_pot_median()
# (중앙값, tools/hw_test.py --pot 수동 스윕 전용)과는 별개 — 이건 매
# _publish_state 틱(LOOP_HZ)마다 발행되는 라이브 /car/steering_angle 스트림용.
ARDUINO_STEERING = dict(
    kf_process_noise=1.0,       # ADC^2/tick — 조향 자체의 실제 변화(정상적 회전) 허용
    kf_measurement_noise=4.0,   # ADC^2 — 1회 판독의 노이즈 분산 📏
)
