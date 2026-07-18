import math

from .. import filters

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
# 하단 1/3을 사다리꼴 POI(원근 반영 — 가까운 밴드는 전체 폭, 먼 밴드일수록
# 좁힘)로 잡고, 그 안을 4개 수평 밴드로 나눠 각 밴드에서 우측 차선(중앙 점선~
# 우측 실선)의 중심점을 찾아 거리 가중 평균한다. 웹캠 대상 독립 프로토타입
# 검증: D:\...\prototypes\lane_center_poi_windows_test.py (WSL 카메라
# usbipd 문제로 실제 ROS 파이프라인 검증 전 Windows 네이티브 캡처로 튜닝함,
# 자세한 경과는 memory project_autodrive_skku_lane_center_poi_prototype 참고).
LANE_POI = dict(
    white_thresh=170,           # 그레이스케일 흰색 임계값 (조명에 따라 튜닝)
    roi_frac=(0.67, 0.98),      # POI: 프레임 하단 1/3
    n_bands=4,                  # 밴드 개수 (원래 5단이었으나 가장 먼 단 제거)
    cluster_gap_px=15,          # 이 이상 컬럼이 비면 별도 클러스터로 분리
    min_cluster_mass=8,
    max_cluster_width_px=60,    # 이보다 넓은 블롭은 배경(바닥/벽)으로 간주해 제외
    min_row_span_frac=0.55,     # 밴드 세로의 이 비율 미만만 채우면 노이즈(반사 등)로 간주
    center_deadzone_px=20,      # 이 안쪽이면 직진(F) 유지
    near_weight_decay=0.6,      # 밴드별 목표점 가중 평균 시 먼 밴드일수록 이 비율로 감쇠
    trapezoid_near_half_frac=0.50,  # 가장 가까운 밴드: 중앙 기준 ±50%(=전체 폭)
    trapezoid_far_half_frac=0.22,   # 가장 먼 밴드: 중앙 기준 ±22%로 좁힘
    # ---- 프레임 간 스무딩: 칼만필터(filters.ScalarKalmanFilter, 2026-07-18
    # EMA에서 전환) — 매 프레임 raw_target을 측정으로 삼아 predict+update한다.
    # kf_process_noise(Q)가 클수록 최근 측정을 더 빨리 따라가고(반응성↑),
    # kf_measurement_noise(R)가 클수록 한 프레임의 튐을 더 무시한다(안정성↑).
    kf_process_noise=4.0,        # px^2/tick — 밴드 목표점 자체의 프레임 간 변화 허용
    kf_measurement_noise=15.0,   # px^2 — 한 프레임 raw_target 측정의 노이즈 분산 📏
    kf_max_variance_px=None,     # px^2 — 이 이상 불확실해지면 추정 폐기(None=비활성,
                                  # 기존처럼 미검출 프레임에도 마지막 조향을 무기한 유지)
    # ---- 곡선 구간용 Circular Hough Transform 보정 (2026-07-17, 교수님 제안) ----
    # POI 전체를 한 번 이진화해 cv2.HoughCircles로 차선을 근사하는 원을 찾고,
    # 기존 밴드 타겟과 hough_min_inlier_bands개 이상 일치할 때만 그 원 위의
    # 매끄러운 점으로 밴드 타겟을 대체한다. 원을 못 찾거나 안 맞으면 기존
    # 밴드별 클러스터링 결과를 그대로 쓴다(완전 폴백) — 직선/완만한 구간에서
    # 기존 검증된 동작을 그대로 보존하기 위함.
    hough_enabled=True,
    hough_dp=1.5,                # HoughCircles 누산기 해상도(1=원본, 클수록 저해상도/빠름)
    hough_param1=100,            # HoughCircles 내부 Canny 상단 임계(하단은 절반)
    hough_param2=25,             # 누산기 임계 — 낮을수록 관대(오검출 위험↑) 📏
    hough_min_radius_px=150,     # 이보다 작은 원은 노이즈로 배제 📏
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


def _poi_band_rows(h, roi_frac, n_bands, i):
    """밴드 i(0=가장 가까움/화면 아래, n_bands-1=가장 멂/위)의 (y0, y1)."""
    y_bottom = int(h * roi_frac[1])
    y_top = int(h * roi_frac[0])
    band_h = (y_bottom - y_top) / n_bands
    y1 = y_bottom - int(i * band_h)
    y0 = y_bottom - int((i + 1) * band_h)
    return max(y0, y_top), y1


def _poi_band_x_range(w, cx, n_bands, i, near_half, far_half):
    """밴드 i의 (x_lo, x_hi) -- 사다리꼴: 멀수록 좁아짐(원근 반영)."""
    t = i / max(n_bands - 1, 1)
    half_frac = near_half + t * (far_half - near_half)
    half_px = half_frac * w
    return max(0, int(cx - half_px)), min(w, int(cx + half_px))


def _fit_lane_circle(binary, x_off, y_off, cfg):
    """POI 전체 이진 마스크(0/255, 8bit 1채널)에서 차선을 근사하는 원 하나를
    Circular Hough Transform(cv2.HoughCircles)으로 찾는다. 절대좌표
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
    return float(cx) + x_off, float(cy) + y_off, float(r)


def _circle_x_at_y(cx, cy, r, y, prefer_x):
    """원(cx, cy, r) 위에서 y에 해당하는 x — 이차방정식 두 해 중 기존 밴드
    타겟(prefer_x)에 더 가까운 쪽을 선택. y가 원 밖(|y-cy|>r)이면 None."""
    dy = y - cy
    if abs(dy) > r:
        return None
    dx = math.sqrt(max(r * r - dy * dy, 0.0))
    x1, x2 = cx - dx, cx + dx
    return x1 if abs(x1 - prefer_x) <= abs(x2 - prefer_x) else x2


class LaneCenterTracker:
    """follow_lane_poi()의 프레임 간 스무딩 상태(filters.ScalarKalmanFilter)를
    들고 있는 객체 -- 미션 on_start()에서 하나 만들어 매 틱 재사용한다.
    .smoothed는 기존 호출부(road.py/t_parking.py/debug_viz.py) 호환을 위해
    유지하는 읽기전용 별칭이다."""

    def __init__(self):
        self.kf = filters.ScalarKalmanFilter()

    def reset(self):
        self.kf = filters.ScalarKalmanFilter()

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


def analyze_lane_poi(frame, config=LANE_POI):
    """POI 사다리꼴 다단 밴드 분석 — 조향 없이 밴드/클러스터/목표점만 계산한다.
    follow_lane_poi(조향)와 debug_viz.draw_lane_poi(오버레이)가 공유하는 순수 분석.

    반환 details dict (frame 없음/cv2 미설치면 None):
      cx, w, h        : 프레임 중앙/크기
      bands           : [dict(y0, y1, x_lo, x_hi, clusters, target)] — 밴드 0=가장 가까움.
                        clusters는 (중심컬럼, 질량, 좌끝, 우끝) 절대좌표 목록.
      path_points     : [(y_mid, target_col)] 가까운 순 정렬
      raw_target      : 거리 가중 평균 목표 컬럼 (검출 실패 시 None)
    """
    if frame is None or cv2 is None or np is None:
        return None
    h, w = frame.shape[:2]
    cx = w / 2.0
    n_bands = config["n_bands"]

    # POI 전체(모든 밴드를 합친 y범위) 이진 마스크 — 실선/점선 분류와
    # Circular Hough 피팅이 공유해서 쓴다(중복 연산 방지).
    y_top = int(h * config["roi_frac"][0])
    y_bottom = int(h * config["roi_frac"][1])
    gray_full = cv2.cvtColor(frame[y_top:y_bottom, :], cv2.COLOR_BGR2GRAY)
    _, binary_full = cv2.threshold(gray_full, config["white_thresh"], 255,
                                   cv2.THRESH_BINARY)

    bands = []
    path_points = []
    for i in range(n_bands):
        y0, y1 = _poi_band_rows(h, config["roi_frac"], n_bands, i)
        x_lo, x_hi = _poi_band_x_range(
            w, cx, n_bands, i,
            config["trapezoid_near_half_frac"], config["trapezoid_far_half_frac"])
        band = frame[y0:y1, x_lo:x_hi]
        gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, config["white_thresh"], 255, cv2.THRESH_BINARY)
        clusters = _poi_find_clusters(
            binary, config["cluster_gap_px"], config["min_cluster_mass"],
            config["max_cluster_width_px"], config["min_row_span_frac"])
        clusters = [(c + x_lo, m, lo + x_lo, hi + x_lo) for (c, m, lo, hi) in clusters]
        target = _poi_pick_right_lane_center(clusters, binary_full, config)
        bands.append(dict(y0=y0, y1=y1, x_lo=x_lo, x_hi=x_hi,
                          clusters=clusters, target=target))
        if target is not None:
            path_points.append(((y0 + y1) // 2, target))

    path_points.sort(key=lambda p: -p[0])  # 가까운(아래) -> 먼(위) 순

    circle = None
    if config["hough_enabled"] and len(path_points) >= config["hough_min_inlier_bands"]:
        fit = _fit_lane_circle(binary_full, 0, y_top, config)
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
            # 기존 밴드 타겟과 hough_min_inlier_bands개 이상 일치할 때만 채택 —
            # 아니면 fitted를 버리고 path_points(기존 클러스터링 결과)를 그대로 둔다.
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

    return dict(cx=cx, w=w, h=h, bands=bands,
                path_points=path_points, raw_target=raw_target, circle=circle)


def follow_lane_poi(tracker, car, frame, config=LANE_POI):
    """POI 사다리꼴 다단 밴드로 우측 차선 중심을 추종해 조향.

    follow_lane()과 동일한 안전 계약: 프레임 예외/미검출 시 steer를 호출하지
    않고 이전 조향을 유지한다(실패를 "F"로 강제 리셋하면 그 자체가 실제
    조향 액추에이션이라 더 위험함).

    반환: analyze_lane_poi details에 smoothed/offset/direction을 더한 dict —
    디버그 오버레이용. 분석 자체가 불가하면 None. 기존 호출부는 반환값을
    무시해도 동작이 같다.
    """
    try:
        details = analyze_lane_poi(frame, config)
        if details is None:
            return None
        smoothed = tracker.update(details["raw_target"],
                                  config["kf_process_noise"], config["kf_measurement_noise"])
        variance = tracker.kf.variance()
        details.update(smoothed=smoothed, variance=variance, offset=None, direction=None,
                       deadzone=config["center_deadzone_px"],
                       roi_frac=config["roi_frac"])
        max_var = config["kf_max_variance_px"]
        if smoothed is None or (max_var is not None and variance > max_var):
            return details  # 추정 불가/미검출 또는 너무 불확실 -> 이전 조향 유지

        offset = smoothed - details["cx"]
        deadzone = config["center_deadzone_px"]
        if offset > deadzone:
            direction = "R"
        elif offset < -deadzone:
            direction = "L"
        else:
            direction = "F"
        car.steer(direction)
        details.update(smoothed=smoothed, offset=offset, direction=direction)
        return details
    except Exception as e:
        print(f"[lane_follow] follow_lane_poi 실패, 이번 프레임 스킵: {e}")
        return None
