# 차량/센서 설정 — 새 환경에서는 이 파일(또는 bringup.launch.py 인자)만 바꾸면 된다.
# 포트가 None이면 자동 감지를 시도한다. 실패 시 arduino_port/lidar_port launch 인자로 지정.

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

# 전방 카메라는 세로(portrait)로 물리 마운트 — 상/하 스플릿의 수직 화각을
# 넓히기 위함(av_car 시뮬 모델 기준 vFOV 49°→78°, 아래 VEHICLE 참고 참조).
# C920은 하드웨어 회전을 지원하지 않으므로 캡처 직후 소프트웨어로 90도 보정
# 한다. 실제 마운트 방향(시계/반시계)에 따라 "CW"/"CCW"로 뒤집을 것 — 실차에서
# 좌우가 반전되면 이 값부터 확인.
FRONT_CAMERA_ROTATE = "CW"  # None | "CW" | "CCW" | "180"

LOOP_HZ = 30             # 메인 제어 루프 주기

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
# LANE_EDGE, traffic.py: STOP_LINE/TRAFFIC_PIXEL_RATIO, t_parking.py: T_PARKING
# (+ PARKING_LINE_WHITE/PARK_PULSE_GAP_S — 다른 파일 값과 공유하는 로컬 상수,
# 해당 파일 주석에 교차 참조 있음).

# 참고용 실측 차량 제원 (WSL ~/autonomousAIdrive/sim/models/av_car/model.sdf,
# kinematic_single_track_parameters.md 기반 측정치). 현재 조향은 120ms 펄스
# 방식이라 아래 값이 아직 제어 로직에 쓰이진 않음 — 차선 변경 궤적 계산 등
# 필요해질 때 참고.
WHEELBASE_M = 0.545
TRACK_WIDTH_M = 0.430
WHEEL_RADIUS_M = 0.10
STEERING_LIMIT_DEG = 20
FRONT_CAMERA_SENSOR_HFOV_DEG = 78   # Logitech C920 spec (센서 자체, landscape 기준)
FRONT_CAMERA_MOUNT_VFOV_DEG = 78    # portrait 마운트 후 실효 수직화각 (sim 기준, hfov<->vfov 교환)

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
    fusion_vo_weight_max=0.8,
    goodfeatures=dict(maxCorners=200, qualityLevel=0.01, minDistance=7, blockSize=7),
    lk_win_size=(21, 21),
    lk_max_level=3,
)
