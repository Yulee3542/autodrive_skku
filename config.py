# 차량/센서 설정 — 새 환경에서는 이 파일(또는 main.py 인자)만 바꾸면 된다.
# 포트가 None이면 자동 감지를 시도한다. 실패 시 --arduino / --lidar 인자로 지정.

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
OBSTACLE_STOP_M = 0.7    # 라이다 장애물 정지 거리 (m, 실차 검증값 700mm)
LIDAR_FRONT_SECTOR = 30  # 전방 ±N도만 장애물 판정에 사용

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
