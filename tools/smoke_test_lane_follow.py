#!/usr/bin/env python3
"""차선 추종 통합 경로 + portrait 카메라 회전 보정 + POI 곡선(Circular Hough
Transform) 검출을 실제 하드웨어 없이 점검하는 스모크 테스트.

follow_lane()(vendor edge_detection 경로)은 2026-07-17부로 어떤 미션도 더 이상
쓰지 않는다(road/traffic 모두 follow_lane_poi로 통일) — 이 파일의 관련 테스트는
그날 발견된 vendor HoughLinesP shape 회귀(OpenCV 버전에 따라 line[0] 언패킹
실패)의 회귀 가드로만 남겨둔다.

사용법: python tools/smoke_test_lane_follow.py
(cv2/numpy 필요 — pip install opencv-python-headless numpy)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                 "autodrive_skku_ros"))

try:
    import cv2
    import numpy as np
except ImportError as e:
    print(f"[X ] cv2/numpy 미설치: {e}")
    print("     pip install opencv-python-headless numpy 로 설치 후 재실행")
    sys.exit(1)

from autodrive_skku_ros.missions.lane_follow import (
    follow_lane, follow_lane_poi, analyze_lane_poi, LaneCenterTracker, LANE_POI,
    _fit_lane_circle, _circle_x_at_y, _classify_lane_type, _poi_pick_right_lane_center)
from autodrive_skku_ros.vendor import Function_Library as fl


class FakeCar:
    """실제 ArduinoNode 대신 steer()/steer_pulse()/drive() 호출만 기록하는 더미."""

    def __init__(self):
        self.calls = []
        self.steers = []
        self.pulses = []
        self.drives = []

    def steer(self, direction):
        self.calls.append(direction)
        self.steers.append(direction)

    def steer_pulse(self, direction):
        self.calls.append(direction)
        self.pulses.append(direction)

    def drive(self, v):
        self.drives.append(v)


class FakeClock:
    """follow_lane_poi(now=...)에 주입하는 가짜 단조 시계 (missions 테스트와 동일 패턴)."""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_line_frame(width=640, height=240):
    """근사 수직선이 있는 합성 BGR 프레임 (차선처럼 보이도록)."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.line(frame, (width // 2, 0), (width // 2, height - 1), (255, 255, 255), 3)
    return frame


def make_blank_frame(width=640, height=240):
    return np.zeros((height, width, 3), dtype=np.uint8)


def make_two_line_frame(width=640, height=240, c1=200, c2=440):
    """세로 흰 차선 2줄(analyze_lane_poi가 우측 차선 중앙을 보간할 수 있는
    최소 구성) — 순수 직선이라 원(circle)이 잡히면 안 됨(회귀 검증용)."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.line(frame, (c1, 0), (c1, height - 1), (255, 255, 255), 6)
    cv2.line(frame, (c2, 0), (c2, height - 1), (255, 255, 255), 6)
    return frame


def check(name, condition):
    status = "OK" if condition else "X "
    print(f"  [{status}] {name}")
    return condition


def test_follow_lane_no_crash():
    print("== follow_lane 예외 없이 동작 ==")
    env = fl.libCAMERA()
    car = FakeCar()
    lane_edge = dict(width=500, height=120, gap=40, threshold=150)

    ok = True
    ok &= check("frame=None → no-op",
                _no_raise(lambda: follow_lane(env, car, None, lane_edge)))
    ok &= check("env=None → no-op",
                _no_raise(lambda: follow_lane(None, car, make_line_frame(), lane_edge)))
    ok &= check("빈 프레임(선 없음) → 예외 없음",
                _no_raise(lambda: follow_lane(env, car, make_blank_frame(), lane_edge)))
    ok &= check("선 있는 프레임 → 예외 없음",
                _no_raise(lambda: follow_lane(env, car, make_line_frame(), lane_edge)))
    ok &= check("깨진 lane_edge_config(TypeError 유발) → 예외 전파 없이 스킵",
                _no_raise(lambda: follow_lane(env, car, make_line_frame(), {"bogus_key": 1})))
    return ok


def _no_raise(fn):
    try:
        fn()
        return True
    except Exception as e:
        print(f"      -> 예외 발생: {e}")
        return False


def test_portrait_rotation_shapes():
    print("== portrait 회전 보정 shape 점검 ==")
    landscape = np.zeros((480, 640, 3), dtype=np.uint8)
    ok = True
    ok &= check(f"landscape shape {landscape.shape} == (480, 640, 3)",
                landscape.shape == (480, 640, 3))

    rotated = cv2.rotate(landscape, cv2.ROTATE_90_CLOCKWISE)
    ok &= check(f"90도 회전 후 shape {rotated.shape} == (640, 480, 3)",
                rotated.shape == (640, 480, 3))

    h = rotated.shape[0]
    top, bottom = rotated[:h // 2, :], rotated[h // 2:, :]
    ok &= check(f"상하 스플릿 top shape {top.shape} == (320, 480, 3)",
                top.shape == (320, 480, 3))
    ok &= check(f"상하 스플릿 bottom shape {bottom.shape} == (320, 480, 3)",
                bottom.shape == (320, 480, 3))
    return ok


def test_circle_x_at_y():
    print("== _circle_x_at_y 순수 기하 계산 ==")
    ok = True
    # 중심(100,100) 반지름 50: y=중심 -> 두 해(50,150), prefer_x로 선택
    ok &= check("prefer_x가 우측 해에 가까우면 우측 해",
                _circle_x_at_y(100, 100, 50, 100, 140) == 150)
    ok &= check("prefer_x가 좌측 해에 가까우면 좌측 해",
                _circle_x_at_y(100, 100, 50, 100, 60) == 50)
    ok &= check("접점(y=cy+r) -> 해 하나로 수렴",
                abs(_circle_x_at_y(100, 100, 50, 150, 999) - 100) < 1e-6)
    ok &= check("원 밖(|y-cy|>r) -> None",
                _circle_x_at_y(100, 100, 50, 200, 100) is None)
    return ok


def test_fit_lane_circle():
    print("== _fit_lane_circle (Circular Hough Transform, BEV 좌표계) ==")
    ok = True
    cfg = dict(LANE_POI)

    ring = np.zeros((600, 600), dtype=np.uint8)
    cv2.circle(ring, (300, 300), 200, 255, 4)  # hough_min/max_radius_px(150~2000) 범위 안
    fit = _fit_lane_circle(ring, cfg)
    ok &= check("뚜렷한 원 -> 검출됨", fit is not None)
    if fit is not None:
        cx, cy, r = fit
        ok &= check(f"중심 근사 일치 ({cx:.0f},{cy:.0f})~=(300,300)",
                    abs(cx - 300) < 20 and abs(cy - 300) < 20)
        ok &= check(f"반지름 근사 일치 ({r:.0f})~=200",
                    abs(r - 200) < 20)

    blank_img = np.zeros((400, 400), dtype=np.uint8)
    ok &= check("빈 이미지 -> None(원 없음)",
                _fit_lane_circle(blank_img, cfg) is None)
    return ok


def _identity_bev_cfg(w, h, **overrides):
    """BEV 워프를 사실상 무왜곡 항등변환으로 만드는 cfg — src_frac을 프레임
    전체를 덮는 사각형으로, bev_w/bev_h를 프레임 크기와 동일하게 둔다.
    (실제 캘리브된 사다리꼴이 아닌) 순수 클러스터링/피킹 로직만 검증하고 싶은
    테스트에서 픽셀 좌표를 그대로 보존하기 위해 쓴다."""
    cfg = dict(LANE_POI)
    cfg.update(roi_frac=(0.0, 1.0), bev_w=w, bev_h=h,
              src_frac=(0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0),
              car_center_px=(w / 2.0, h - 1.0))  # 기본값(160,239)은 bev_w=320 가정 —
                                                  # 이 테스트는 bev_w=w로 맞췄으니 같이 갱신
    cfg.update(overrides)
    return cfg


def test_analyze_lane_poi_straight_unaffected():
    """핵심 회귀 기준: 직선(원이 아닌) 프레임에서는 Hough 원이 안 잡히거나
    안 맞아 circle=None으로 폴백, raw_target은 기존 밴드 클러스터링 결과
    그대로(원 검출 도입 전과 동일)여야 한다. BEV 워프는 항등변환 cfg로 무력화
    (사다리꼴 캘리브 자체가 아니라 클러스터링/피킹 로직 회귀만 확인하는 테스트)."""
    print("== analyze_lane_poi 직선 2줄 프레임 -> Hough 폴백(회귀 없음) ==")
    ok = True
    w, h = 640, 240
    frame = make_two_line_frame(width=w, height=h)
    cfg = _identity_bev_cfg(w, h)
    details = analyze_lane_poi(frame, cfg)
    ok &= check("직선 2줄 -> circle=None (원이 아니므로 폴백)",
                details["circle"] is None)
    expected = 200 + 0.75 * (440 - 200)  # _poi_pick_right_lane_center 보간값
    ok &= check(f"raw_target 기존 밴드 보간값과 일치 (~{expected:.0f})",
                details["raw_target"] is not None
                and abs(details["raw_target"] - expected) < 5)
    return ok


def _make_column_mask(height=100, width=300, columns=None):
    """columns: {center_x: [(row_lo, row_hi), ...]} — 각 컬럼(±15px 밴드)의
    켜진 행 구간을 지정해 실선/점선/미확정 패턴을 합성한다."""
    mask = np.zeros((height, width), dtype=np.uint8)
    for cx, runs in (columns or {}).items():
        x0, x1 = max(0, cx - 15), min(width, cx + 15)
        for r0, r1 in runs:
            mask[r0:r1, x0:x1] = 255
    return mask


def test_classify_lane_type():
    print("== _classify_lane_type (팀원 C920 분류기 이식) ==")
    ok = True
    cfg = dict(LANE_POI)

    solid_mask = _make_column_mask(columns={50: [(0, 100)]})
    ok &= check("전체 높이 연속 -> solid",
                _classify_lane_type(solid_mask, 50, cfg) == "solid")

    dashed_mask = _make_column_mask(
        columns={50: [(0, 10), (20, 30), (40, 50), (60, 70), (80, 90)]})
    ok &= check("주기적 구간(런 5개, 커버리지 50%) -> dashed",
                _classify_lane_type(dashed_mask, 50, cfg) == "dashed")

    none_mask = _make_column_mask(columns={})
    ok &= check("빈 컬럼 -> none",
                _classify_lane_type(none_mask, 50, cfg) == "none")

    unknown_mask = _make_column_mask(columns={50: [(25, 75)]})  # 런 1개, 커버리지 50%
    ok &= check("연속 1구간(50% 커버리지, solid/dashed 기준 다 못 넘음) -> unknown",
                _classify_lane_type(unknown_mask, 50, cfg) == "unknown")
    return ok


def test_poi_pick_right_lane_center_classification():
    """핵심 검증: 위치(순서)만 보는 기존 휴리스틱과 실제로 다른 답을 내는
    시나리오 — 가장 오른쪽 클러스터가 '실선처럼 안 생겼으면' 무시하고 진짜
    실선을 찾아야 한다."""
    print("== _poi_pick_right_lane_center — 분류 기반이 위치 휴리스틱과 갈리는 경우 ==")
    ok = True
    cfg = dict(LANE_POI)
    # x=50: dashed(런5개), x=150: solid(전체 연속), x=250: 연속 1구간(50%) -> unknown
    binary_full = _make_column_mask(columns={
        50: [(0, 10), (20, 30), (40, 50), (60, 70), (80, 90)],
        150: [(0, 100)],
        250: [(25, 75)],
    })
    clusters = [(50, 100, 45, 55), (150, 100, 145, 155), (250, 100, 245, 255)]

    old_heuristic = (clusters[-2][0] + clusters[-1][0]) / 2.0  # (150+250)/2=200
    new_result = _poi_pick_right_lane_center(clusters, binary_full, cfg)
    expected = 50 + 0.75 * (150 - 50)  # dashed(50)~solid(150) 3/4 지점 = 125

    ok &= check(f"분류 기반 결과({new_result})가 위치 휴리스틱({old_heuristic})과 다름",
                new_result != old_heuristic)
    ok &= check(f"진짜 dashed(50)~solid(150) 3/4 지점(~{expected}) 채택",
                new_result is not None and abs(new_result - expected) < 1e-6)

    # binary_full/cfg 없이 호출하면(하위 호환) 기존 위치 휴리스틱 그대로
    fallback = _poi_pick_right_lane_center(clusters)
    ok &= check("binary_full 없이 호출 -> 기존 위치 휴리스틱으로 100% 폴백",
                fallback == old_heuristic)
    return ok


# ---- Pure-Pursuit 조향 (2026-07-23) ----
# 부호 규약을 잘못 뒤집으면(구현 중 실제로 한 번 걸렸던 버그) 실차에서 반대
# 방향으로 조향하는 심각한 회귀라, 여기서 명시적으로 좌/우 각각 고정한다.
# center_deadzone_deg(3도)를 확실히 넘도록 목표점을 중심에서 크게 치우치게
# 배치(c1=20,c2=260 -> 목표 200, 중심 320보다 120px 왼쪽 / c1=380,c2=620 ->
# 목표 560, 중심보다 240px 오른쪽).

def _run_until_pulse(tracker, car, frame, cfg, clock, max_ticks=20):
    """deadzone/칼만필터 초기 수렴 때문에 첫 틱에 바로 안 나올 수 있어, pulse가
    나올 때까지(혹은 max_ticks까지) 게이트 간격만큼 시계를 돌리며 반복 호출."""
    for _ in range(max_ticks):
        follow_lane_poi(tracker, car, frame, cfg, now=clock)
        if car.pulses:
            return
        clock.advance(0.2)  # STEER_PULSE_GAP_S(0.15s)보다 넉넉히 크게


def test_pure_pursuit_direction_sign():
    print("== follow_lane_poi Pure-Pursuit 조향 부호 (좌/우 고정 회귀가드) ==")
    ok = True
    w, h = 640, 240
    cfg = _identity_bev_cfg(w, h)

    left_frame = make_two_line_frame(width=w, height=h, c1=20, c2=260)   # 목표 200 (중심 320보다 왼쪽)
    tracker, car, clk = LaneCenterTracker(), FakeCar(), FakeClock()
    _run_until_pulse(tracker, car, left_frame, cfg, clk)
    ok &= check("목표가 중심보다 왼쪽 -> steer_pulse('L')",
                car.pulses and car.pulses[-1] == "L")

    right_frame = make_two_line_frame(width=w, height=h, c1=380, c2=620)  # 목표 560 (중심보다 오른쪽)
    tracker2, car2, clk2 = LaneCenterTracker(), FakeCar(), FakeClock()
    _run_until_pulse(tracker2, car2, right_frame, cfg, clk2)
    ok &= check("목표가 중심보다 오른쪽 -> steer_pulse('R')",
                car2.pulses and car2.pulses[-1] == "R")

    centered_frame = make_two_line_frame(width=w, height=h, c1=185, c2=425)  # 목표 365, 중심 320+45
    # 45px는 위 좌/우 케이스(±120/±240px)보다 훨씬 작음 — deadzone_deg 근방 거동 참고용.
    # (deadzone 자체는 각도 단위라 px 임계로 정확히 대응되진 않지만, 크게 벗어난
    # 좌/우 케이스와 대비해 "거의 중앙"에서 방향이 급변하지 않는지 확인)
    tracker3, car3, clk3 = LaneCenterTracker(), FakeCar(), FakeClock()
    for _ in range(5):
        follow_lane_poi(tracker3, car3, centered_frame, cfg, now=clk3)
        clk3.advance(0.2)
    ok &= check("거의 중앙 목표 -> steer('F') 유지 또는 그쪽 방향으로만 펄스(반대방향 없음)",
                all(p != "L" for p in car3.pulses) or all(p != "R" for p in car3.pulses))
    return ok


def test_follow_lane_poi_steer_pulse_gating():
    """예전 car.steer()(dedup)는 방향이 안 바뀌면 이후 프레임에서 재전송을 안 해
    사실상 한 번만 툭 치고 끝났다 — car.steer_pulse()(강제 재전송)로 바꿔
    deadzone 밖인 동안 계속 보정해야 한다(단, STEER_PULSE_GAP_S 간격은 지킬 것)."""
    print("== follow_lane_poi steer_pulse 게이팅 (지속 보정, 예전 dedup 한계 해소) ==")
    ok = True
    w, h = 640, 240
    cfg = _identity_bev_cfg(w, h)
    frame = make_two_line_frame(width=w, height=h, c1=20, c2=260)  # 목표 200, 계속 'L'이어야 함
    tracker, car, clk = LaneCenterTracker(), FakeCar(), FakeClock()

    for _ in range(3):  # 칼만필터 수렴 + 첫 펄스까지
        follow_lane_poi(tracker, car, frame, cfg, now=clk)
        if car.pulses:
            break
        clk.advance(0.2)
    n_after_warmup = len(car.pulses)
    ok &= check("워밍업 후 최소 1회 펄스 발행", n_after_warmup >= 1)

    # 게이트 간격(STEER_PULSE_GAP_S) 안에서는 재호출해도 추가 펄스 없어야 함
    follow_lane_poi(tracker, car, frame, cfg, now=clk)
    ok &= check("게이트 간격 안 재호출 -> 추가 펄스 없음", len(car.pulses) == n_after_warmup)

    # 간격을 넘겨 여러 틱 더 돌리면 계속 같은 방향으로 펄스가 이어져야 함(지속 보정)
    for _ in range(5):
        clk.advance(0.2)
        follow_lane_poi(tracker, car, frame, cfg, now=clk)
    ok &= check("간격 지나 계속 호출 -> 펄스가 계속 추가됨(지속 보정, 예전엔 여기서 멈췄음)",
                len(car.pulses) > n_after_warmup)
    ok &= check("전부 같은 방향('L')", all(p == "L" for p in car.pulses))
    return ok


def test_follow_lane_poi_speed_modulation():
    print("== follow_lane_poi 곡률 기반 속도 감속 ==")
    ok = True
    w, h = 640, 240
    from autodrive_skku_ros import config
    cfg = _identity_bev_cfg(w, h)

    # 목표 = 280+0.75*(333-280) = 319.75 ~= 중심(320) — 자전거모델 공식이
    # (2*WHEELBASE_M/lookahead_거리) 배율로 픽셀 오프셋을 증폭하므로(실측: 640px
    # 프레임에서 10px 편차만으로도 delta~13도가 나옴 — px_per_m 캘리브 전이라
    # 정상), "거의 직진"을 안정적으로 재현하려면 실제로 거의 정확히 중앙이어야 함
    straight_frame = make_two_line_frame(width=w, height=h, c1=280, c2=333)
    tracker, car, clk = LaneCenterTracker(), FakeCar(), FakeClock()
    for _ in range(5):
        follow_lane_poi(tracker, car, straight_frame, cfg, now=clk)
        clk.advance(0.2)
    ok &= check(f"거의 직진 -> 속도 ~= config.DRIVE_SPEED({config.DRIVE_SPEED})",
                car.drives and abs(car.drives[-1] - config.DRIVE_SPEED) <= 5)

    curve_frame = make_two_line_frame(width=w, height=h, c1=20, c2=260)  # 목표 200, 큰 편차(급커브 상당)
    tracker2, car2, clk2 = LaneCenterTracker(), FakeCar(), FakeClock()
    for _ in range(5):
        follow_lane_poi(tracker2, car2, curve_frame, cfg, now=clk2)
        clk2.advance(0.2)
    ok &= check(f"급커브 상당 -> 속도 < config.DRIVE_SPEED (config.SLOW_SPEED={config.SLOW_SPEED} 쪽으로 감속)",
                car2.drives and car2.drives[-1] < config.DRIVE_SPEED)
    return ok


def test_bev_adaptive_threshold_vs_fixed():
    """이 트랙에서 실제로 실패가 확인된 시나리오 재현: 차선 V~150, 배경 V~100 —
    고정 임계 170이면 차선(150<170)도 배경(100<170)도 전부 "흰색 아님"으로 걸러져
    완전 미검출된다. Otsu 적응 임계는 150/100 사이 어딘가에 임계를 잡아 정상
    분리해야 한다."""
    print("== BEV Otsu 적응 임계 vs 예전 고정임계(170) 실패 시나리오 ==")
    ok = True
    w, h = 640, 240
    cfg = _identity_bev_cfg(w, h)

    frame = np.full((h, w, 3), 100, dtype=np.uint8)  # 어두운 노면 배경 (V~100)
    cv2.line(frame, (200, 0), (200, h - 1), (150, 150, 150), 8)  # 차선 (V~150, 여전히 170 미만)
    cv2.line(frame, (440, 0), (440, h - 1), (150, 150, 150), 8)

    details = analyze_lane_poi(frame, cfg)
    ok &= check("어두운 조명(차선 V~150 < 고정임계 170)에서도 raw_target 검출됨"
                " (Otsu 적응 임계 덕분)", details is not None and details["raw_target"] is not None)
    return ok


def test_analyze_uses_full_height_classification():
    """회귀가드 (2026-07-23에 실제로 발생했던 버그): analyze_lane_poi가
    _poi_pick_right_lane_center에 **POI 전체 높이** 마스크를 넘겨야 한다.
    밴드 슬라이스(60px)를 넘기면 점선 한 마디가 그 밴드를 꽉 채워 solid/unknown으로
    보여 dashed+solid 쌍 판정이 무너지고, 조용히 위치 휴리스틱으로 폴백한다
    (실패해도 예외가 안 나고 값만 달라져서 기존 테스트로는 안 잡혔음)."""
    print("== analyze_lane_poi: 실선/점선 분류가 POI 전체 높이 기준인지 (회귀가드) ==")
    ok = True
    H, W = 240, 640          # 실제 bottom 프레임 규격
    frame = np.zeros((H, W, 3), np.uint8)
    cv2.rectangle(frame, (100, 0), (118, H), (200, 200, 200), -1)   # 좌 실선
    cv2.rectangle(frame, (500, 0), (518, H), (200, 200, 200), -1)   # 우 실선
    for y in range(0, H, 24):                                        # 중앙 점선
        cv2.rectangle(frame, (300, y), (318, y + 12), (200, 200, 200), -1)

    details = analyze_lane_poi(frame, LANE_POI)
    # BEV 가로 0.5배(640->320): 좌 실선~54.5, 중앙 점선~154.5, 우 실선~254.5
    dashed_x, solid_x = 154.5, 254.5
    expect_pair = dashed_x + 0.75 * (solid_x - dashed_x)   # 분류 기반 = 229.5
    expect_positional = (solid_x + dashed_x) / 2.0          # 위치 휴리스틱 = 204.5
    got = details["raw_target"]
    ok &= check(f"raw_target({got:.1f})이 dashed+solid 쌍 기준값({expect_pair:.1f})과 일치",
                got is not None and abs(got - expect_pair) < 3)
    ok &= check(f"위치 휴리스틱 폴백값({expect_positional:.1f})이 아님 (그게 나오면 회귀)",
                got is not None and abs(got - expect_positional) > 3)
    # 코리도어 락도 같은 이유로 전체 높이 분류 — 중앙 점선을 solid로 잠그면 안 된다
    corr = details["corridor"]
    ok &= check(f"코리도어가 좌우 실선에만 잠김 (left={corr['left']}, right={corr['right']}), "
                "중앙 점선(≈145~159)을 경계로 삼지 않음",
                corr["left"] is not None and corr["right"] is not None
                and corr["left"] < 100 and corr["right"] > 200)
    return ok


def test_pure_pursuit_gain_not_saturating():
    """회귀가드: lookahead(ld_min_m)가 축거보다 짧으면 delta=atan(2L·sinα/ld)의
    이득 2L/ld가 1을 크게 넘어 조향각이 즉시 ±STEERING_LIMIT_DEG로 포화된다
    (2026-07-23 실측: ld=0.35에서 BEV 폭의 6%만 치우쳐도 포화 → 사실상 bang-bang,
    게다가 속도까지 항상 SLOW_SPEED로 바닥). 정상 범위에서는 포화되면 안 된다."""
    print("== Pure-Pursuit 이득이 포화되지 않는지 (ld vs 축거 회귀가드) ==")
    from autodrive_skku_ros import config
    from autodrive_skku_ros.missions.lane_follow import _pure_pursuit_delta_deg
    ok = True
    C = LANE_POI
    ld = min(C["ld_max_m"], max(C["ld_min_m"], C["ld_gain"] * (config.DRIVE_SPEED / 100.0)))
    gain = 2.0 * config.WHEELBASE_M / ld
    ok &= check(f"기본 속도에서 이득 2L/ld = {gain:.2f} 가 2.0 미만 (ld={ld:.2f}m, 축거={config.WHEELBASE_M}m)",
                gain < 2.0)

    n, bh = C["n_bands"], C["bev_h"] / C["n_bands"]

    def delta_for(offset_px):
        pts = []
        for i in range(n):
            y1 = int(C["bev_h"] - i * bh)
            y0 = int(C["bev_h"] - (i + 1) * bh)
            pts.append(((y0 + y1) // 2, C["bev_w"] / 2.0 + offset_px))
        pts.sort(key=lambda p: -p[0])
        dbg = {}
        return _pure_pursuit_delta_deg(pts, C, float(config.DRIVE_SPEED), debug=dbg), dbg

    d_small, dbg_small = delta_for(20)     # BEV 폭의 6%
    d_mid, dbg_mid = delta_for(80)         # 25%
    ok &= check(f"20px(6%) 치우침 -> delta {d_small:+.1f}deg, 포화 아님",
                not dbg_small["saturated"])
    ok &= check(f"80px(25%) 치우침 -> delta {d_mid:+.1f}deg, 포화 아님",
                not dbg_mid["saturated"])
    ok &= check("치우침이 커질수록 조향각도 커짐 (비례 제어가 살아있음)",
                abs(d_mid) > abs(d_small) + 1.0)
    # 속도 변조가 한 점에 눌려붙지 않고 실제로 범위를 갖는지
    def speed_for(d):
        frac = min(1.0, abs(d) / C["curve_steer_deg_for_min"])
        return round(config.DRIVE_SPEED * (1 - frac) + config.SLOW_SPEED * frac)
    ok &= check(f"속도 변조 범위 유지 (20px->{speed_for(d_small)}, 80px->{speed_for(d_mid)}) "
                "— 항상 SLOW_SPEED로 바닥치지 않음",
                speed_for(d_small) > config.SLOW_SPEED + 10)
    return ok


def main():
    results = [
        test_follow_lane_no_crash(),
        test_portrait_rotation_shapes(),
        test_circle_x_at_y(),
        test_fit_lane_circle(),
        test_analyze_lane_poi_straight_unaffected(),
        test_classify_lane_type(),
        test_poi_pick_right_lane_center_classification(),
        test_pure_pursuit_direction_sign(),
        test_follow_lane_poi_steer_pulse_gating(),
        test_follow_lane_poi_speed_modulation(),
        test_bev_adaptive_threshold_vs_fixed(),
        test_analyze_uses_full_height_classification(),
        test_pure_pursuit_gain_not_saturating(),
    ]
    passed = all(results)
    print("\n결과:", "이상 없음" if passed else "위 [X] 항목 확인 필요")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
