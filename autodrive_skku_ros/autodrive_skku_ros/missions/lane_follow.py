import math
import time

from .. import filters
from .. import config as _config

try:
    from ..vendor import Function_Library as fl
except ImportError:  # 패키지 미설치 개발 환경 — 차선 인식 없이 골격만 동작
    fl = None

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


# ---------------- 튜닝 파라미터 ----------------
# 팀 검증 완료된 차선 인식(edge_detection) 파라미터 (main3_c920_record.py 실차
# 튜닝값). road.py/traffic.py가 공유하므로 두 파일이 함께 import하는 이 파일에 둔다.
# 📏 원래 landscape bottom-half 프레임 기준값 — config.FRONT_CAMERA_ROTATE로
# portrait 마운트를 켜면 bottom 프레임 크기/종횡비가 달라지므로 재튜닝이 필요할
# 수 있음.
LANE_EDGE = dict(width=500, height=120, gap=40, threshold=150)


def follow_lane(env, car, frame, lane_edge_config):
    """차선 인식 후 조향. road/traffic 미션이 공유하는 통합 지점.

    검증된 fl.edge_detection()을 그대로 쓰되, 한 프레임에서 예외가 나도
    (나쁜 프레임/일시적 CV 오류) 미션 루프 전체가 죽지 않도록 격리한다.
    실패 시에는 steer를 아예 호출하지 않는다 — direction=None과 동일하게
    "이전 조향 유지"로 취급 (실패를 "F"로 강제 리셋하면 그 자체가 실제
    조향 액추에이션이라 더 위험함).
    """
    if frame is None or env is None:
        return

    try:
        direction = env.edge_detection(frame, **lane_edge_config)
    except Exception as e:
        print(f"[lane_follow] edge_detection 실패, 이번 프레임 스킵: {e}")
        return

    if direction == fl.FORWARD:
        car.steer("F")
    elif direction == fl.LEFT:
        car.steer("L")
    elif direction == fl.RIGHT:
        car.steer("R")
    # None이면 이전 조향 유지 (steer()의 dedupe 특성상 재전송 없음)


# ---------------- POI 사다리꼴 다단 밴드 우측차선 추종 (2026-07-16 적용) ----------------
# vendor.Function_Library.edge_detection()(전체 프레임 Canny+Hough)이 차선 없는
# 환경(실내 등)에서 주변 구조물 엣지를 차선으로 오검출하는 문제 때문에 개발한
# 대안 경로 — vendor 코드는 그대로 두고(팀 검증 완료, 손대지 않기로 함) road
# 미션만 이쪽으로 전환. traffic 미션은 기존 follow_lane()/LANE_EDGE 그대로 사용.
#
# ---- BEV(원근변환) + 적응 임계 (2026-07-23, 팀 저장소 HANLAB_auto/yeoeun_traffic
# 브랜치의 lane_detector_node.py 이식) ----
# 이전 버전은 원본(원근 왜곡된) 프레임에 사다리꼴로 밴드를 좁혀가며 흉내만 냈고,
# 흰색 판정은 고정 그레이스케일 임계(170)였다. 팀원이 같은 트랙 조명에서 바로 그
# 고정 임계(170)가 실패하는 걸 실측 확인(steering_1.jpg 등)하고 "BEV로 먼저 워핑
# 후 그 안에서 Otsu 적응 임계"로 교체해 해결한 사례가 있어 그대로 이식한다.
# BEV 워프 이후에는 실제 지면 거리가 픽셀에 선형으로 대응되므로, 원래 있던
# "밴드마다 사다리꼴로 좁히는" 근사가 필요 없어진다(각 밴드는 BEV 폭 전체를 씀).
LANE_POI = dict(
    # BEV 캔버스 크기 (작을수록 연산 절약; 팀원 lane_detector_node.py와 동일 기본값)
    bev_w=320, bev_h=240,
    # 원근변환 소스 사다리꼴 4점(좌하,좌상,우상,우하) — roi_frac으로 잘라낸 하단
    # 프레임 "안에서의" 비율. 📏 기본값은 무왜곡(잘라낸 영역을 그대로 리사이즈만) —
    # 진짜 사다리꼴(원근보정)은 실차 캘리브 후 적용할 것: `--test <이미지>`로 BEV
    # 출력을 보며 차선 2줄이 수직 평행이 되도록 좌상/우상의 x비율을 안쪽으로
    # 좁히면 된다(팀원 캘리브 절차와 동일). 무왜곡 기본값을 쓰면 캘리브 전에도
    # 기존 지오메트리가 보존돼 회귀가 없다 — Otsu 적응 임계 개선만 즉시 적용됨.
    src_frac=(0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0),
    roi_frac=(0.67, 0.98),      # 원본 bottom 프레임에서 이 비율 구간만 잘라 BEV로 워프
    n_bands=4,                  # 밴드 개수 (원래 5단이었으나 가장 먼 단 제거)
    cluster_gap_px=15,          # 이 이상 컬럼이 비면 별도 클러스터로 분리
    min_cluster_mass=8,
    max_cluster_width_px=60,    # 이보다 넓은 블롭은 배경(바닥/벽)으로 간주해 제외
    min_row_span_frac=0.55,     # 밴드 세로의 이 비율 미만만 채우면 노이즈(반사 등)로 간주
    # 흰색 판정: BEV 내 Otsu 적응 임계(+바닥값). 채도 게이트(white_s_max)만
    # 대회 공유 config.WHITE_HSV와 같은 철학(stop_line/obstacle_cam도 이 방식,
    # None=공유 s_max 사용) — 이건 조명 밝기와 무관하게 "무채색인가"만 보므로
    # 공유해도 안전하다. v_min_floor는 별개: config.WHITE_HSV의 v_min=180은
    # "고정 임계로 믿을 수 있는 값"이라 Otsu 바닥값으로 재사용하면 Otsu가 잡을 수
    # 있는 어두운 조명(예: 차선 V~150)까지 도로 걸러버려 고정임계 실패를 그대로
    # 재현하게 된다(실측 확인 — 회귀 테스트 test_bev_adaptive_threshold_vs_fixed
    # 참고). 그래서 팀원 lane_detector_node.py의 V_MIN_FLOOR(110, "전부 어두울
    # 때만" 막는 용도)를 그대로 따로 쓴다.
    white_s_max=None,           # None=config.WHITE_HSV 공유 s_max
    v_min_floor=110,            # 📏 팀원 실측값 차용 — 전부 어두운 프레임에서 Otsu가
                                 # 너무 낮게(노이즈까지 흰색으로) 잡는 것만 방지하는
                                 # 용도라 config.WHITE_HSV(v_min=180, 고정임계용)보다
                                 # 훨씬 낮아야 함. 실차 조명에서 재확인 필요.
    # ---- 실선/점선 코리도어 락 (2026-07-23, lane_detector_node.py "corridor lock"
    # 이식 — 공개 Udacity Advanced-Lane-Finding 계열 구현들에서도 "이전 프레임 피팅
    # 결과 주변만 재검색"이 표준 패턴으로 확인됨). 직전 프레임에 SOLID로 확정된
    # 차선 경계 바깥을 이번 프레임 검색에서 제외해 벽/글레어 오검출을 줄인다.
    # 잠금 자체가 이번 프레임 자기 안에서 나온 판정을 다음 프레임에 쓰는 방식이라
    # 첫 프레임/미확정 시엔 제한 없이 전체 마스크를 그대로 쓴다(완전 폴백).
    corridor_lock_enabled=True,
    corridor_mask_margin_px=5,
    center_deadzone_deg=3.0,     # 이 각도 미만 조향 오차는 무시(직진 유지) — 예전 픽셀
                                  # deadzone(center_deadzone_px)의 각도판 (채터 방지)
    near_weight_decay=0.6,      # 밴드별 목표점 가중 평균(raw_target, 표시/진단용) 감쇠
    # ---- 프레임 간 스무딩: 칼만필터(filters.ScalarKalmanFilter). 이제 픽셀 타겟이
    # 아니라 pure-pursuit 조향각(delta_deg)을 스무딩한다 — 단위만 px²→deg²로 바뀌었을
    # 뿐 필터 자체(2026-07-18 EMA에서 전환)는 동일하다.
    kf_process_noise=4.0,         # deg²/tick — 조향각 자체의 프레임 간 변화 허용
    kf_measurement_noise=15.0,    # deg² — 한 프레임 raw delta_deg 측정의 노이즈 분산 📏
    kf_max_variance_deg=None,     # deg² — 이 이상 불확실해지면 추정 폐기(None=비활성)
    # ---- 곡선 구간용 Circular Hough Transform 보정 (2026-07-17, 교수님 제안) ----
    # BEV 워프 이후에는 실제 원형 곡률이 BEV에서도 원에 가깝게 보이므로(원근
    # 왜곡이 없어져서) 이 보정이 예전보다 더 정확해진다. POI 전체를 한 번
    # 이진화해 cv2.HoughCircles로 차선을 근사하는 원을 찾고, 기존 밴드 타겟과
    # hough_min_inlier_bands개 이상 일치할 때만 그 원 위의 매끄러운 점으로 밴드
    # 타겟을 대체한다. 원을 못 찾거나 안 맞으면 기존 밴드별 클러스터링 결과를
    # 그대로 쓴다(완전 폴백).
    hough_enabled=True,
    hough_dp=1.5,                # HoughCircles 누산기 해상도(1=원본, 클수록 저해상도/빠름)
    hough_param1=100,            # HoughCircles 내부 Canny 상단 임계(하단은 절반)
    hough_param2=25,             # 누산기 임계 — 낮을수록 관대(오검출 위험↑) 📏
    hough_min_radius_px=150,     # 이보다 작은 원은 노이즈로 배제 📏 (BEV 픽셀 기준)
    hough_max_radius_px=2000,    # 이보다 크면 사실상 직선 취급 📏
    hough_min_inlier_bands=3,    # 원 채택에 필요한 최소 일치 밴드 수
    hough_inlier_tol_px=25,      # 밴드 타겟과 원호 예측치 허용 오차(px)
    # ---- 실선/점선 분류 (2026-07-17, 팀원 C920 분류기 이식) ----
    # POI 전체 높이에서 클러스터 컬럼의 세로 점유율/최장연속구간/구간수로
    # 실선(solid)·점선(dashed)을 판정 — "밴드 안 클러스터 중 오른쪽 두 개가
    # 점선+실선"이라고 위치만으로 가정하던 기존 휴리스틱을 검증한다. 확실한
    # dashed+solid 쌍을 못 찾으면 기존 위치 휴리스틱으로 100% 폴백.
    lane_type_half_width_px=15,        # 분류용 밴드 반폭(클러스터 중심 기준)
    lane_type_row_min_pixels=2,        # 이 이상 흰 픽셀 있어야 그 행을 "점유"로 침
    lane_type_solid_coverage=0.70,     # 📏
    lane_type_solid_longest_ratio=0.45,  # 📏
    lane_type_dashed_min_coverage=0.15,  # 📏
    lane_type_dashed_max_coverage=0.70,  # 📏
    lane_type_dashed_min_runs=2,
    lane_type_none_coverage=0.05,      # 📏
    # ---- Pure-Pursuit 조향 (2026-07-23, 팀 저장소 lane_pure_pursuit_node.py의
    # 조향 수식 이식 — F1TENTH pure_pursuit 계열). 기존에는 raw_target(밴드 목표점
    # 가중평균)과 프레임 중앙의 픽셀 오프셋만 보고 deadzone bang-bang(L/R/F 중
    # 하나만 판정)으로 조향했다 — 곡률/lookahead 개념이 전혀 없었다. 이제
    # path_points(밴드별 목표점, 차량 기준 좌표로 환산)에서 lookahead 지점을 골라
    # 자전거모델 조향각(delta)을 계산한다:
    #   alpha = atan2(y_left, x_fwd)  (lookahead 지점 방위각)
    #   delta = atan2(2*WHEELBASE_M*sin(alpha), lookahead_distance)
    # config.WHEELBASE_M/STEERING_LIMIT_DEG(이미 존재하던 실측 차량 제원)를 그대로 쓴다.
    px_per_m=176.0,               # 📏 미실측 — 팀원 동일 BEV 폭(320) 기준값을 시작값으로
                                    # 차용. 실측 전에는 조향각 "크기"만 부정확할 수 있음
                                    # (방향은 항상 맞음 — STEERING_LIMIT_DEG로 clamp됨).
    # (bev_w/2, bev_h-1) = BEV 하단 중앙(앞범퍼). tuning.py의 None 슬롯(NONE_SENTINEL/
    # float 전용)과 섞이면 안 되므로(model_path와 동일 이유) 실제 좌표값을 기본값으로
    # 둔다 — bev_w/bev_h를 튜닝으로 바꾸면 이 값도 같이 갱신할 것.
    car_center_px=(160.0, 239.0),
    ld_min_m=0.35,                 # lookahead 거리 하한
    ld_max_m=1.20,                 # lookahead 거리 상한
    ld_gain=0.25,                  # ld = clamp(ld_gain * (speed_proxy/100), ld_min, ld_max)
    # ---- 조향각 → 펄스 이산화. 아두이노가 연속 조향각 명령을 못 받고(120ms 고정
    # 펄스만 가능) 각도 피드백도 아직 신뢰 못 하는 상태(POT 캘리 보류 중)라,
    # steer_est 폐루프 대신 "매 게이트 간격(config.STEER_PULSE_GAP_S)마다 현재
    # delta_deg가 deadzone을 넘으면 그 방향으로 펄스 1회" — 물리적으로 펄스가
    # STEER_PULSE_GAP_S(120ms 펄스+여유)보다 빠르게 나갈 수 없어 "펄스 개수"
    # 자체보다 "펄스가 지속되는 동안"이 사실상의 크기 신호가 된다. 예전
    # car.steer()(dedup) 방식은 방향이 안 바뀌면 이후 프레임에서 재전송을 아예
    # 안 해 사실상 한 번 툭 치고 끝났다 — car.steer_pulse()(강제 재전송)로 바꿔
    # deadzone 안으로 들어올 때까지 계속 보정하는 게 이번 변경의 핵심.
    deg_per_pulse=2.0,              # 📏 미실측 — 최소 추정치(보수적: 과도출력 방지 위해
                                     # 작게 잡음). 실측 후 road.py 상단 주석 절차 참고.
    # ---- 곡률 기반 개루프 속도 (팀원 SPEED_STRAIGHT/SPEED_CURVE 스케줄 이식).
    # config.DRIVE_SPEED(직선)/config.SLOW_SPEED(급커브) 단일 소스를 그대로 쓴다 —
    # 새 상수 도입 없이 기존 값만 재사용.
    speed_modulation_enabled=True,
    curve_steer_deg_for_min=14.0,   # 이 조향각 이상이면 최저속(config.SLOW_SPEED)
)


def _poi_find_clusters(binary, gap_px, min_mass, max_width, min_row_span_frac):
    """binary: 밴드의 2D 이진 이미지. 컬럼 방향으로 흰 픽셀 클러스터를 묶고
    (중심컬럼, 질량, 좌끝, 우끝) 목록을 좌->우로 반환. 폭/질량/세로연속성
    기준을 모두 만족해야 '차선'으로 인정한다."""
    band_h = binary.shape[0]
    col_sum = binary.sum(axis=0) / 255.0
    cols = np.where(col_sum > 0)[0]
    if len(cols) == 0:
        return []
    clusters = []
    start = cols[0]
    prev = cols[0]
    for c in cols[1:]:
        if c - prev > gap_px:
            clusters.append((start, prev))
            start = c
        prev = c
    clusters.append((start, prev))

    out = []
    for lo, hi in clusters:
        mass = int(col_sum[lo:hi + 1].sum())
        if mass < min_mass or (hi - lo) > max_width:
            continue
        sub = binary[:, lo:hi + 1]
        rows = np.where(sub.any(axis=1))[0]
        if len(rows) == 0:
            continue
        if (rows[-1] - rows[0] + 1) < min_row_span_frac * band_h:
            continue  # 세로로 충분히 이어지지 않음 -> 반사/얼룩 등 노이즈
        out.append(((lo + hi) / 2.0, mass, lo, hi))
    return out


def _classify_lane_type(binary_full, x, cfg):
    """POI 전체 높이(binary_full)에서 x 근처 세로 밴드의 행별 점유를 스캔해
    차선 종류를 추정한다(팀원 C920 프로토타입의 LaneClassifier 이식 —
    점유율/최장연속구간/구간수 기준). 'solid'/'dashed'/'none'/'unknown' 중
    하나를 반환."""
    h, w = binary_full.shape[:2]
    half = cfg["lane_type_half_width_px"]
    x0 = max(0, int(round(x - half)))
    x1 = min(w, int(round(x + half)))
    if x1 <= x0 or h == 0:
        return "none"
    band = binary_full[:, x0:x1]
    occupied = (band > 0).sum(axis=1) >= cfg["lane_type_row_min_pixels"]

    occupied_rows = int(occupied.sum())
    longest = current = runs = 0
    for is_occ in occupied:
        if is_occ:
            current += 1
            longest = max(longest, current)
        else:
            if current > 0:
                runs += 1
            current = 0
    if current > 0:
        runs += 1

    coverage = occupied_rows / h
    longest_ratio = longest / h

    if coverage > cfg["lane_type_solid_coverage"] and \
            longest_ratio > cfg["lane_type_solid_longest_ratio"]:
        return "solid"
    if runs >= cfg["lane_type_dashed_min_runs"] and \
            cfg["lane_type_dashed_min_coverage"] < coverage <= cfg["lane_type_dashed_max_coverage"]:
        return "dashed"
    if coverage < cfg["lane_type_none_coverage"]:
        return "none"
    return "unknown"


def _poi_pick_right_lane_center(clusters, binary_full=None, cfg=None):
    """실선/점선 분류로 검증된 (점선, 실선) 쌍이 있으면 그걸 우선 사용 —
    가장 오른쪽 solid를 찾고 그 왼쪽에서 가장 가까운 dashed를 찾아 그
    사이(3/4 지점, 실제 우측차선 중앙)를 목표로 삼는다. binary_full/cfg가
    없거나 확실한 쌍을 못 찾으면 기존 위치 휴리스틱으로 폴백(3개+: 우측
    실선+중앙 점선으로 가정한 오른쪽 두 개의 중점. 2개: 점선이 이번
    프레임엔 안 보인다고 보고 좌/우 실선 3/4 지점으로 보간). 1개 이하:
    추정 불가 -> None."""
    cols = [c[0] for c in clusters]

    if binary_full is not None and cfg is not None and len(cols) >= 2:
        types = [_classify_lane_type(binary_full, c, cfg) for c in cols]
        solid_i = next((i for i in range(len(cols) - 1, -1, -1)
                        if types[i] == "solid"), None)
        if solid_i is not None:
            dashed_i = next((j for j in range(solid_i - 1, -1, -1)
                             if types[j] == "dashed"), None)
            if dashed_i is not None:
                left, right = cols[dashed_i], cols[solid_i]
                return left + 0.75 * (right - left)

    if len(cols) >= 3:
        return (cols[-1] + cols[-2]) / 2.0
    if len(cols) == 2:
        left, right = cols[0], cols[1]
        return left + 0.75 * (right - left)
    return None


def _fit_lane_circle(binary, cfg):
    """BEV 전체 이진 마스크(0/255, 8bit 1채널)에서 차선을 근사하는 원 하나를
    Circular Hough Transform(cv2.HoughCircles)으로 찾는다. BEV 좌표계의
    (cx, cy, r) 또는 못 찾으면 None — 호출부가 기존 밴드 클러스터링으로
    폴백한다(직선/완만한 구간은 원이 안 잡히는 게 정상)."""
    h = binary.shape[0]
    circles = cv2.HoughCircles(
        binary, cv2.HOUGH_GRADIENT, dp=cfg["hough_dp"], minDist=max(h, 1),
        param1=cfg["hough_param1"], param2=cfg["hough_param2"],
        minRadius=cfg["hough_min_radius_px"], maxRadius=cfg["hough_max_radius_px"])
    if circles is None:
        return None
    cx, cy, r = circles[0][0]
    return float(cx), float(cy), float(r)


def _circle_x_at_y(cx, cy, r, y, prefer_x):
    """원(cx, cy, r) 위에서 y에 해당하는 x — 이차방정식 두 해 중 기존 밴드
    타겟(prefer_x)에 더 가까운 쪽을 선택. y가 원 밖(|y-cy|>r)이면 None."""
    dy = y - cy
    if abs(dy) > r:
        return None
    dx = math.sqrt(max(r * r - dy * dy, 0.0))
    x1, x2 = cx - dx, cx + dx
    return x1 if abs(x1 - prefer_x) <= abs(x2 - prefer_x) else x2


def _bev_warp(frame, config):
    """bottom 프레임 → roi_frac로 자른 뒤 src_frac 사다리꼴을 BEV(bev_w×bev_h)로
    원근변환. 반환: (bev_bgr, M) — M은 디버그/역변환용(현재는 오버레이도 BEV
    좌표계에 직접 그리므로 미사용, 추후 원본 프레임에 역투영할 때 대비)."""
    h, w = frame.shape[:2]
    y0 = int(h * config["roi_frac"][0])
    y1 = int(h * config["roi_frac"][1])
    roi = frame[y0:y1, :]
    rh, rw = roi.shape[:2]

    sf = config["src_frac"]
    src = np.float32([[sf[0] * rw, sf[1] * rh], [sf[2] * rw, sf[3] * rh],
                      [sf[4] * rw, sf[5] * rh], [sf[6] * rw, sf[7] * rh]])
    bev_w, bev_h = config["bev_w"], config["bev_h"]
    dst = np.float32([[0, bev_h], [0, 0], [bev_w, 0], [bev_w, bev_h]])
    M = cv2.getPerspectiveTransform(src, dst)
    bev = cv2.warpPerspective(roi, M, (bev_w, bev_h))
    return bev, M


def _bev_white_mask(bev_bgr, config):
    """BEV BGR 이미지 → 흰색(차선) 이진 마스크. Otsu 적응 임계(+바닥값)로 V를
    가르고, 채도(S)가 낮은(=무채색) 픽셀만 인정한다 — 팀원 lane_detector_node.py의
    '고정임계 실패 → BEV 내 Otsu 적응' 해결책을 그대로 이식."""
    hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    s = hsv[:, :, 1]
    # s_max만 공유(white_v_min은 안 건드림 — 고정임계용 값이라 Otsu 바닥값과
    # 의미가 다름, LANE_POI 상단 주석 참고). v_min_floor는 이 dict 자체의 값을
    # 그대로 쓴다(공유 아님).
    s_max, _shared_v_min_unused = _config.white_hsv(dict(white_s_max=config["white_s_max"]))
    v_min_floor = config["v_min_floor"]
    otsu_thr, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thr = max(float(otsu_thr), float(v_min_floor))
    mask = ((v >= thr) & (s <= s_max)).astype(np.uint8) * 255
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def analyze_lane_poi(frame, config=LANE_POI, corridor=None):
    """BEV 워프 + Otsu 적응 임계 + 사다리꼴 다단 밴드 분석 — 조향 없이 밴드/
    클러스터/목표점만 계산한다. follow_lane_poi(조향)와 debug_viz.draw_lane_poi
    (오버레이, BEV 좌표계로 그림)가 공유하는 순수 분석.

    corridor: LaneCenterTracker가 들고 있는 코리도어 락 상태 dict(선택) —
    {"left": x 또는 None, "right": x 또는 None}, 직전 프레임에 SOLID로 확정된
    좌/우 경계. None이면 코리도어 락 없이 전체 BEV 폭을 검색한다(콜드스타트와
    동일 — 첫 프레임 또는 락 미사용 호출).

    반환 details dict (frame 없음/cv2 미설치면 None):
      bev             : BEV로 워프된 BGR 이미지 (디버그 오버레이가 여기에 그림)
      cx, w, h        : BEV 프레임 중앙/크기 (bev_w/bev_h와 동일)
      bands           : [dict(y0, y1, clusters, target)] — 밴드 0=가장 가까움(BEV 하단).
                        clusters는 (중심컬럼, 질량, 좌끝, 우끝) BEV 픽셀좌표 목록.
      path_points     : [(y_mid, target_col)] 가까운 순 정렬 (BEV 픽셀좌표)
      raw_target      : 거리 가중 평균 목표 컬럼 (검출 실패 시 None, 표시/진단용
                        — 실제 조향은 follow_lane_poi의 pure-pursuit이 담당)
      corridor        : 이번 프레임 갱신된 코리도어 락 상태 (다음 프레임에 전달할 것)
    """
    if frame is None or cv2 is None or np is None:
        return None
    bev, _M = _bev_warp(frame, config)
    bev_h, bev_w = bev.shape[:2]
    cx = bev_w / 2.0
    n_bands = config["n_bands"]

    binary_full = _bev_white_mask(bev, config)

    search = binary_full
    if config["corridor_lock_enabled"] and corridor:
        margin = config["corridor_mask_margin_px"]
        search = binary_full.copy()
        if corridor.get("left") is not None:
            search[:, :max(0, int(corridor["left"] - margin))] = 0
        if corridor.get("right") is not None:
            search[:, int(corridor["right"] + margin):] = 0

    bands = []
    path_points = []
    band_h = bev_h / n_bands
    for i in range(n_bands):
        y1 = int(bev_h - i * band_h)
        y0 = int(bev_h - (i + 1) * band_h)
        binary = search[y0:y1, :]
        clusters = _poi_find_clusters(
            binary, config["cluster_gap_px"], config["min_cluster_mass"],
            config["max_cluster_width_px"], config["min_row_span_frac"])
        target = _poi_pick_right_lane_center(clusters, binary_full[y0:y1, :], config)
        bands.append(dict(y0=y0, y1=y1, clusters=clusters, target=target))
        if target is not None:
            path_points.append(((y0 + y1) // 2, target))

    path_points.sort(key=lambda p: -p[0])  # 가까운(아래) -> 먼(위) 순

    circle = None
    if config["hough_enabled"] and len(path_points) >= config["hough_min_inlier_bands"]:
        fit = _fit_lane_circle(binary_full, config)
        if fit is not None:
            ccx, ccy, cr = fit
            fitted, inliers = [], 0
            for y_mid, col in path_points:
                x_on_circle = _circle_x_at_y(ccx, ccy, cr, y_mid, col)
                if x_on_circle is None:
                    fitted.append((y_mid, col))
                    continue
                if abs(x_on_circle - col) <= config["hough_inlier_tol_px"]:
                    inliers += 1
                fitted.append((y_mid, x_on_circle))
            if inliers >= config["hough_min_inlier_bands"]:
                path_points = fitted
                circle = dict(cx=ccx, cy=ccy, r=cr, inliers=inliers)

    raw_target = None
    if path_points:
        weights_sum = 0.0
        weighted = 0.0
        for rank, (_y, col) in enumerate(path_points):
            weight = config["near_weight_decay"] ** rank
            weighted += col * weight
            weights_sum += weight
        raw_target = weighted / weights_sum

    # 다음 프레임 코리도어 락 갱신 — 이번 프레임에서 SOLID로 확정된 클러스터의
    # 가장 왼쪽/오른쪽 경계를 기록한다(가장 가까운 밴드=0 기준, 노면에 가장
    # 가까워 신뢰도가 높음). 확정 못하면 해당 방향은 None(제한 없음)으로 유지.
    new_corridor = {"left": None, "right": None}
    if bands and bands[0]["clusters"]:
        near_binary = binary_full[bands[0]["y0"]:bands[0]["y1"], :]
        types = [(c[0], c[2], c[3], _classify_lane_type(near_binary, c[0], config))
                 for c in bands[0]["clusters"]]
        solids = [(lo, hi) for (_c, lo, hi, t) in types if t == "solid"]
        if solids:
            new_corridor["left"] = min(lo for lo, _hi in solids)
            new_corridor["right"] = max(hi for _lo, hi in solids)

    return dict(bev=bev, cx=cx, w=bev_w, h=bev_h, bands=bands,
                path_points=path_points, raw_target=raw_target, circle=circle,
                corridor=new_corridor)


def _to_vehicle_frame(x_px, y_px, car_center_px, px_per_m):
    """BEV 픽셀좌표 (아래로 갈수록 y 증가) → 차량 기준 (x=전방, y=좌측) [m].
    팀 저장소 lane_pure_pursuit_node.py의 _to_vehicle_frame과 동일 공식."""
    dx_px = car_center_px[1] - y_px
    dy_px = car_center_px[0] - x_px
    return dx_px / px_per_m, dy_px / px_per_m


def _pick_lookahead(pts_vf, ld):
    """차량 기준 점들 중 전방거리가 ld 이상인 가장 가까운 점 선택. 없으면
    가장 먼 점(전부 ld보다 가까우면 그나마 제일 앞선 정보로 조향)."""
    cand = [(x, y) for (x, y) in pts_vf if x > 0.05]
    if not cand:
        return None
    cand.sort(key=lambda p: p[0])
    for x, y in cand:
        if math.hypot(x, y) >= ld:
            return x, y
    return cand[-1]


def _pure_pursuit_delta_deg(path_points, config, speed_proxy, debug=None):
    """path_points(BEV 픽셀, 가까운→먼) → pure-pursuit 조향각(도, 좌+/우-는
    car.steer_pulse 방향 부호와 아래 follow_lane_poi에서 맞춘다). 목표점을
    못 고르면 None.

    debug: dict를 넘기면 car_center_px/ld_m/lookahead_px/alpha_deg를 채운다
    (debug_viz.draw_lane_poi의 lookahead 마커/lookahead 원 오버레이용)."""
    car_center_px = config["car_center_px"]
    px_per_m = config["px_per_m"]
    pts_vf = [_to_vehicle_frame(x, y, car_center_px, px_per_m) for (y, x) in path_points]

    ld = min(config["ld_max_m"], max(config["ld_min_m"],
                                     config["ld_gain"] * (speed_proxy / 100.0)))
    if debug is not None:
        debug.update(car_center_px=car_center_px, ld_m=ld)
    goal = _pick_lookahead(pts_vf, ld)
    if goal is None:
        return None
    gx, gy = goal
    ld_actual = math.hypot(gx, gy)
    if ld_actual < 1e-6:
        return None
    alpha = math.atan2(gy, gx)
    delta = math.atan2(2.0 * _config.WHEELBASE_M * math.sin(alpha), ld_actual)
    delta_deg = math.degrees(delta)
    limit = _config.STEERING_LIMIT_DEG
    delta_deg = max(-limit, min(limit, delta_deg))
    if debug is not None:
        # 픽셀좌표로 역변환(_to_vehicle_frame의 역) — 오버레이가 BEV 위에 lookahead
        # 지점을 표시할 수 있게.
        lookahead_px = (car_center_px[0] - gy * px_per_m, car_center_px[1] - gx * px_per_m)
        debug.update(alpha_deg=math.degrees(alpha), lookahead_px=lookahead_px)
    return delta_deg


class LaneCenterTracker:
    """follow_lane_poi()의 프레임 간 상태(칼만필터로 스무딩된 조향각 + 코리도어
    락 + 조향 펄스 게이팅 시각)를 들고 있는 객체 -- 미션 on_start()에서 하나
    만들어 매 틱 재사용한다. .smoothed는 기존 호출부(debug_viz.py) 호환을 위해
    유지하는 읽기전용 별칭이다(이제 픽셀이 아니라 조향각[도] 단위)."""

    def __init__(self):
        self.kf = filters.ScalarKalmanFilter()
        self.corridor = None       # 코리도어 락 상태 (dict) — 첫 프레임엔 None(제한 없음)
        self._last_pulse_t = 0.0   # 마지막 steer_pulse 발행 시각 (게이팅용)

    def reset(self):
        self.kf = filters.ScalarKalmanFilter()
        self.corridor = None
        self._last_pulse_t = 0.0

    @property
    def smoothed(self):
        return self.kf.value()

    def update(self, raw_target, process_noise, measurement_noise):
        """raw_target이 있으면 predict+update, None이면 predict만(측정 없음 —
        추정값은 유지하고 분산만 커진다. 기존 EMA는 이 경우 아예 갱신을 안 해
        "얼마나 오래됐는지"에 대한 정보가 없었다). 반환: 갱신 후 추정값
        (아직 한 번도 초기화 안 됐으면 None)."""
        self.kf.predict(process_noise)
        if raw_target is not None:
            self.kf.update(raw_target, measurement_noise)
        return self.kf.value()


def follow_lane_poi(tracker, car, frame, config=LANE_POI, now=None):
    """BEV + Pure-Pursuit로 우측 차선을 추종해 조향+속도.

    follow_lane()과 동일한 안전 계약: 프레임 예외/미검출 시 조향/속도를 새로
    내리지 않고 이전 상태를 유지한다(실패를 "F"로 강제 리셋하면 그 자체가
    실제 조향 액추에이션이라 더 위험함).

    조향: analyze_lane_poi의 path_points에서 속도 적응형 lookahead 지점을
    골라 자전거모델 조향각(delta_deg)을 계산 → 칼만필터로 스무딩 → deadzone
    (center_deadzone_deg) 밖이면 config.STEER_PULSE_GAP_S 간격으로
    car.steer_pulse(방향)를 계속 발행한다(구 car.steer()는 방향이 안 바뀌면
    재전송을 안 해 사실상 한 번만 툭 치고 끝났음 — steer_pulse 강제재전송으로
    바꿔 deadzone 안으로 들어올 때까지 계속 보정).
    속도: speed_modulation_enabled면 |delta_deg|가 클수록(급커브)
    config.DRIVE_SPEED~config.SLOW_SPEED 사이로 감속.

    반환: analyze_lane_poi details에 delta_deg/smoothed/direction/lookahead를
    더한 dict — 디버그 오버레이용. 분석 자체가 불가하면 None. 기존 호출부는
    반환값을 무시해도 동작이 같다.
    """
    clock = now or time.monotonic
    t = clock()
    try:
        details = analyze_lane_poi(frame, config, corridor=tracker.corridor)
        if details is None:
            return None
        tracker.corridor = details["corridor"]

        pp_dbg = {}
        speed_proxy = float(_config.DRIVE_SPEED)
        raw_delta_deg = _pure_pursuit_delta_deg(details["path_points"], config, speed_proxy,
                                                debug=pp_dbg)
        smoothed = tracker.update(raw_delta_deg,
                                  config["kf_process_noise"], config["kf_measurement_noise"])
        variance = tracker.kf.variance()
        details.update(pp_dbg)
        details.update(delta_deg=raw_delta_deg, smoothed=smoothed, variance=variance,
                       direction=None, deadzone=config["center_deadzone_deg"])
        max_var = config["kf_max_variance_deg"]
        if smoothed is None or (max_var is not None and variance > max_var):
            return details  # 추정 불가/미검출 또는 너무 불확실 -> 이전 상태 유지

        deadzone = config["center_deadzone_deg"]
        if abs(smoothed) <= deadzone:
            direction = "F"
            car.steer("F")
        else:
            # delta_deg 부호 규약(atan2 기반 표준 bicycle model): 양수=목표점이
            # 차량 기준 좌측(y_left>0) → 좌회전. _to_vehicle_frame()에서 목표가
            # 프레임 우측(x_px > cx)이면 y_left<0 → delta<0이 되는 것으로 이미
            # 검산함 — 기존 픽셀 오프셋 방식(offset>0=우측→"R")과 부호가 반대인
            # 대신 좌/우 자체의 의미는 동일(양수=좌측 목표).
            direction = "L" if smoothed > 0 else "R"
            if t - tracker._last_pulse_t >= _config.STEER_PULSE_GAP_S:
                car.steer_pulse(direction)
                tracker._last_pulse_t = t

        if config["speed_modulation_enabled"]:
            frac = min(1.0, abs(smoothed) / config["curve_steer_deg_for_min"])
            speed = int(round(_config.DRIVE_SPEED * (1 - frac) + _config.SLOW_SPEED * frac))
            car.drive(speed)

        details.update(direction=direction)
        return details
    except Exception as e:
        print(f"[lane_follow] follow_lane_poi 실패, 이번 프레임 스킵: {e}")
        return None
