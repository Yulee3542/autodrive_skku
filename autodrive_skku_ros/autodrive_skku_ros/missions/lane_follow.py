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
    smoothing_alpha=0.3,        # 프레임 간 스무딩(EMA) 반영 비율
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


def _poi_pick_right_lane_center(clusters):
    """3개+: 우측 실선 + 중앙 점선의 중점(우측 차선 중앙). 2개: 점선이 이번
    프레임엔 안 보인다고 보고(점선이라 정상) 좌/우 실선 3/4 지점으로 보간.
    1개 이하: 추정 불가 -> None."""
    cols = [c[0] for c in clusters]
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


class LaneCenterTracker:
    """follow_lane_poi()의 프레임 간 스무딩 상태(EMA)를 들고 있는 객체 --
    미션 on_start()에서 하나 만들어 매 틱 재사용한다."""

    def __init__(self):
        self.smoothed = None

    def reset(self):
        self.smoothed = None


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
        target = _poi_pick_right_lane_center(clusters)
        bands.append(dict(y0=y0, y1=y1, x_lo=x_lo, x_hi=x_hi,
                          clusters=clusters, target=target))
        if target is not None:
            path_points.append(((y0 + y1) // 2, target))

    path_points.sort(key=lambda p: -p[0])  # 가까운(아래) -> 먼(위) 순
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
                path_points=path_points, raw_target=raw_target)


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
        details.update(smoothed=tracker.smoothed, offset=None, direction=None,
                       deadzone=config["center_deadzone_px"],
                       roi_frac=config["roi_frac"])
        if details["raw_target"] is None:
            return details  # 이전 조향 유지 (smoothed는 직전 값 그대로)

        alpha = config["smoothing_alpha"]
        tracker.smoothed = details["raw_target"] if tracker.smoothed is None else (
            (1 - alpha) * tracker.smoothed + alpha * details["raw_target"])

        offset = tracker.smoothed - details["cx"]
        deadzone = config["center_deadzone_px"]
        if offset > deadzone:
            direction = "R"
        elif offset < -deadzone:
            direction = "L"
        else:
            direction = "F"
        car.steer(direction)
        details.update(smoothed=tracker.smoothed, offset=offset, direction=direction)
        return details
    except Exception as e:
        print(f"[lane_follow] follow_lane_poi 실패, 이번 프레임 스킵: {e}")
        return None
