# 차량/센서 설정 — 새 환경에서는 이 파일(또는 bringup.launch.py 인자)만 바꾸면 된다.
# 포트가 None이면 자동 감지를 시도한다. 실패 시 arduino_port/lidar_port launch 인자로 지정.

ARDUINO_PORT = None      # 예: "/dev/ttyACM0"
ARDUINO_BAUD = 9600
LIDAR_PORT = None        # 예: "/dev/ttyUSB0"
LIDAR_BAUD = 115200

# 카메라: C920 한 대를 상/하로 분할해 사용 (상단=신호등, 하단=차선)
FRONT_CAMERA = 0         # /dev/video0
REAR_CAMERA = None       # T주차용 후방 카메라 인덱스. 없으면 None
CAMERA_SPLIT = True      # False면 전방 프레임 전체를 top/bottom 양쪽에 그대로 전달
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

# 전방 장애물(흰색 장애물 차량) 카메라 감지 — road 미션 ④.
# 대회 규격: 장애물 차량·정지선·실선/점선 모두 흰색 → 형태로 구분한다.
# (차선=가늘고 세로로 김, 정지선=가로로 얇은 밴드, 장애물=폭·높이 모두 큰 블롭)
OBSTACLE_CAM = dict(
    s_max=60, v_min=180,        # HSV 흰색: 채도 낮고 밝기 높음
    roi_top=0.35, roi_bottom=0.95,   # bottom 프레임 세로 ROI (비율)
    col_lo=0.20, col_hi=0.80,   # 중앙 컬럼 밴드 — 우리 차선의 장애물만
    min_area_ratio=0.04,        # ROI 면적 대비 블롭 면적비 임계
    min_w_ratio=0.15,           # ROI 폭 대비 블롭 폭 (차선은 이보다 가늚)
    min_h_ratio=0.25,           # ROI 높이 대비 블롭 높이 (정지선은 이보다 낮음)
    min_fill=0.45,              # bbox 채움비 — 대각선 차선은 희박해서 탈락
)

# 차선 변경 기동 (road 미션 ③④) — 펄스(120ms)↔조향각 매핑 미측정, 전부 실차 튜닝 대상.
# 근거: 조향 ±20도 → 회전반경 L/tan20 ≈ 1.5m, 차선폭 0.85m → S자 각 구간 헤딩 ~40도.
LANE_CHANGE = dict(
    pulses=4,          # 진입/복귀 조향 펄스 횟수
    pulse_gap_s=0.15,  # 펄스 간 최소 간격 (steer_pulse 반복 전송 주기)
    out_s=1.5,         # 옆 차선으로 나가는 구간 지속 시간
    back_s=1.5,        # 반대 조향으로 차선 정렬하는 구간
    straight_s=0.8,    # 직진 안정화 구간
    speed=80,          # 기동 중 속도
    cooldown_s=2.0,    # 기동 후 재트리거 억제 시간
)

# 정지선(흰색) 인식 (traffic 미션 ①). row_fill 0.7: 횡단보도(진행방향 줄무늬,
# 폭 점유 ~60%)와 세로 차선이 행 채움비를 못 넘게 하는 값.
STOP_LINE = dict(
    s_max=60, v_min=180,   # HSV 흰색
    roi_top=0.55,          # bottom 프레임에서 이 비율 아래 행만 검사 (가까운 노면)
    row_fill=0.7,          # 행 폭 대비 흰 픽셀 비율 임계
    min_rows=6,            # 연속으로 임계를 넘어야 하는 행 수
    wait_max_s=6.0,        # 정지선 대기 중 신호등 미검출 시 재출발까지 시간 (교착 방지)
    cooldown_s=3.0,        # 재출발 후 같은 정지선 재트리거 억제
)

# T주차 (t_parking 미션) — 주차칸 950x1500mm(규정), 완료 후 3~5초 정지(규정) 기준.
T_PARKING = dict(
    side="R",              # 주차 슬롯이 있는 쪽 ('L'/'R') — 당일 코스 확인 후 설정
    map_scans=30,          # MAP_BUILD에서 누적할 스캔 수
    slot_gap_min_m=0.60,   # 주차 차량 사이 갭 최소 폭 (슬롯 판정)
    slot_max_lateral_m=2.0,  # 슬롯 후보로 인정할 최대 측면 거리
    align_tol_px=25,       # 후방캠 주차선 중점 정렬 허용 오차 (px)
    align_ticks=5,         # 연속 정렬 판정 틱 수
    turn_in_pulses=4,      # PARK 진입 조향 펄스 수
    turn_in_s=2.0,         # 슬롯 방향 후진 회전 구간
    straighten_s=1.5,      # 반대 조향으로 차체 정렬 구간
    rear_stop_m=0.30,      # 후방 이 거리 이내면 주차 완료 (뒤 범퍼 기준)
    hold_s=4.0,            # 완료 후 정지 유지 (규정 3~5초)
    park_max_s=12.0,       # PARK 상태 안전 타임아웃
)

# 팀 검증 완료된 차선 인식(edge_detection) 파라미터 (main3_c920_record.py)
# 주의: 이 값들은 원래 landscape bottom-half 프레임 기준 실차 튜닝값이다.
# FRONT_CAMERA_ROTATE로 portrait 마운트를 켜면 bottom 프레임의 크기/종횡비가
# 달라지므로 그대로 맞으리라는 보장이 없다 — portrait 마운트 전환 후 실차에서
# 재튜닝이 필요할 수 있음.
LANE_EDGE = dict(width=500, height=120, gap=40, threshold=150)

# 신호등 판정: 상단 프레임에서 해당 색 픽셀이 이 비율을 넘어야 인식
TRAFFIC_PIXEL_RATIO = 0.005

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
