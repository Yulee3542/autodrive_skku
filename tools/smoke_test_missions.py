#!/usr/bin/env python3
"""나머지 미션(road ③④ / traffic ① / t_parking ①~④) 테스트 구현을 실제
하드웨어 없이 점검하는 스모크 테스트.

- 카메라: 합성 프레임으로 흰색끼리(차선/정지선/횡단보도/장애물) 교차 오검출 검증
- 미션: FakeCar(호출 기록) + FakeClock(_now 주입)으로 상태머신 end-to-end

라이다 후방 장착 지오메트리 + ROS LaserScan 변환 순수 함수 테스트는
nodes/lidar_node.py의 로컬 --selftest로 이관됨
(python3 -m autodrive_skku_ros.nodes.lidar_node --selftest).

사용법: python tools/smoke_test_missions.py
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

from autodrive_skku_ros import config
from autodrive_skku_ros.missions import traffic as traffic_mod
from autodrive_skku_ros.missions.road import RoadMission, detect_obstacle_ahead, OBSTACLE_CAM, LANE_CHANGE
from autodrive_skku_ros.missions.traffic import TrafficMission, STOP_LINE
from autodrive_skku_ros.missions.t_parking import TParkingMission, T_PARKING


class FakeCar:
    """실제 ArduinoNode 대신 호출만 기록하는 더미."""

    def __init__(self):
        self.calls = []
        self.steers = []
        self.pulses = []
        self.drives = []

    def go(self):
        self.calls.append(("go", None))

    def stop(self):
        self.calls.append(("stop", None))

    def drive(self, v):
        self.drives.append(v)
        self.calls.append(("drive", v))

    def steer(self, d):
        self.steers.append(d)
        self.calls.append(("steer", d))

    def steer_pulse(self, d):
        self.pulses.append(d)
        self.calls.append(("steer_pulse", d))


class FakeClock:
    """mission._now에 주입하는 가짜 단조 시계."""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def sensors(top=None, bottom=None, rear=None, scan=None, pose=None, pose_conf=0.0):
    # pose=None/pose_conf=0.0 기본값 = 오도메트리 미보정 상태 — 기존 테스트가
    # 이 기본값으로 전부 통과해야 fail-inert(오도메트리 없이는 동작 불변)가 증명된다.
    return {"top": top, "bottom": bottom, "rear": rear,
            "lidar_min_m": None, "lidar_scan": scan, "state": None,
            "pose": pose, "pose_conf": pose_conf}


# ---- 합성 프레임 (bottom 프레임 규격: portrait 스플릿 후 320x480) ----

H, W = 320, 480


def blank():
    return np.zeros((H, W, 3), dtype=np.uint8)


def lane_line_frame():
    """세로 흰 차선 1줄 — 장애물/정지선으로 오인하면 안 됨."""
    f = blank()
    cv2.line(f, (W // 2, 0), (W // 2, H - 1), (255, 255, 255), 6)
    return f


def stop_line_frame():
    """가로 흰 정지선 밴드 (하단 ROI 안, 폭 전체)."""
    f = blank()
    cv2.rectangle(f, (0, 260), (W - 1, 280), (255, 255, 255), -1)
    return f


def zebra_frame():
    """횡단보도: 진행방향 줄무늬 5개 — 행 채움비 ~0.375라 정지선 아님."""
    f = blank()
    for k in range(5):
        x = int((k + 0.5) * W / 5) - 18
        cv2.rectangle(f, (x, 180), (x + 36, H - 1), (255, 255, 255), -1)
    return f


def obstacle_frame():
    """중앙의 큰 흰 블롭 = 흰색 장애물 차량."""
    f = blank()
    cv2.rectangle(f, (180, 150), (300, 250), (255, 255, 255), -1)
    return f


def color_frame(bgr):
    f = np.zeros((240, 320, 3), dtype=np.uint8)
    f[:, :] = bgr
    return f


def rear_lines_frame(c1, c2):
    """후방캠: 세로 주차선 2줄 (컬럼 c1, c2)."""
    f = blank()
    for c in (c1, c2):
        cv2.line(f, (c, 0), (c, H - 1), (255, 255, 255), 6)
    return f


def check(name, condition):
    status = "OK" if condition else "X "
    print(f"  [{status}] {name}")
    return bool(condition)


# ---- 1. 카메라 흰색 구분 (차선/정지선/횡단보도/장애물) ----

def test_white_discrimination():
    print("== 흰색끼리 형태 구분 (교차 오검출 방지) ==")
    cam = OBSTACLE_CAM
    m = TrafficMission()
    ok = True
    ok &= check("장애물 블롭 → 장애물 True", detect_obstacle_ahead(obstacle_frame(), cam))
    ok &= check("세로 차선 → 장애물 False", not detect_obstacle_ahead(lane_line_frame(), cam))
    ok &= check("정지선 밴드 → 장애물 False", not detect_obstacle_ahead(stop_line_frame(), cam))
    ok &= check("횡단보도 → 장애물 False", not detect_obstacle_ahead(zebra_frame(), cam))
    ok &= check("None/빈 프레임 → 장애물 False",
                not detect_obstacle_ahead(None, cam) and not detect_obstacle_ahead(blank(), cam))
    ok &= check("정지선 밴드 → 정지선 True", m.stop_line_detected(stop_line_frame()))
    ok &= check("세로 차선 → 정지선 False", not m.stop_line_detected(lane_line_frame()))
    ok &= check("횡단보도 → 정지선 False", not m.stop_line_detected(zebra_frame()))
    ok &= check("장애물 블롭 → 정지선 False", not m.stop_line_detected(obstacle_frame()))
    ok &= check("None 프레임 → 정지선 False", not m.stop_line_detected(None))
    return ok


# ---- 1b. vendor 폴백 상수 동기화 가드 ----

def test_vendor_fallback_sync():
    """traffic.py의 vendor 폴백 복사본(_FALLBACK)이 실제 vendor 상수와 일치하는지.
    vendor 파일은 수정 금지라 자동 동기화가 불가능해 이 가드로 드리프트를 잡는다.
    vendor 미설치 환경에서는 검증할 대상이 없으므로 통과 처리."""
    print("== vendor 폴백 상수 동기화 ==")
    if traffic_mod.fl is None:
        print("  [OK] vendor 미설치 환경 — 폴백 자체가 사용 중, 비교 생략")
        return True
    from autodrive_skku_ros.vendor import Function_Library as v
    fb = traffic_mod._FALLBACK
    ok = True
    ok &= check("RED/GREEN 인덱스 일치", (v.RED, v.GREEN) == (fb["RED"], fb["GREEN"]))
    # vendor HUE_THRESHOLD는 4색(RED/GREEN/BLUE/YELLOW) 튜플이지만 traffic은
    # RED/GREEN만 쓰므로 그 두 항목만 일치하면 된다.
    ok &= check("HUE_THRESHOLD[RED/GREEN] 일치",
                list(v.HUE_THRESHOLD[v.RED]) == list(fb["HUE_THRESHOLD"][fb["RED"]]) and
                list(v.HUE_THRESHOLD[v.GREEN]) == list(fb["HUE_THRESHOLD"][fb["GREEN"]]))
    ok &= check("SATURATION 일치", v.SATURATION == fb["SATURATION"])
    return ok


# ---- 2. traffic 미션 FSM (정지선 대기 / 교착 가드 / 빨간불 / cooldown) ----

def test_traffic_fsm():
    print("== traffic 미션 상태머신 ==")
    ok = True
    m, car, clk = TrafficMission(), FakeCar(), FakeClock()
    m.on_start(car, config)
    m._now = clk

    m.step(sensors(bottom=stop_line_frame()), car)
    ok &= check("정지선 → 정지 + wait='line'", m.wait == "line" and car.drives[-1] == 0)

    clk.advance(STOP_LINE["wait_max_s"] + 0.1)
    m.step(sensors(bottom=blank()), car)
    ok &= check("신호등 미검출 대기 초과 → 재출발 (교착 방지)",
                m.wait is None and car.drives[-1] == config.DRIVE_SPEED)

    m.step(sensors(top=color_frame((0, 0, 0)), bottom=stop_line_frame()), car)
    ok &= check("cooldown 중 정지선 재트리거 안 함", m.wait is None)

    clk.advance(STOP_LINE["cooldown_s"] + 0.1)
    m.step(sensors(top=color_frame((0, 0, 255)), bottom=blank()), car)
    ok &= check("빨간불 → 정지 + wait='red'", m.wait == "red" and car.drives[-1] == 0)

    clk.advance(60.0)
    m.step(sensors(bottom=blank()), car)
    ok &= check("빨간불 대기는 타임아웃 없음 (초록불까지 무기한)",
                m.wait == "red" and car.drives[-1] == 0)

    m.step(sensors(top=color_frame((0, 255, 0)), bottom=blank()), car)
    ok &= check("초록불 → 재출발", m.wait is None and car.drives[-1] == config.DRIVE_SPEED)
    return ok


# ---- 3. road 미션: 회피 방향 + 차선 변경 페이즈 머신 ----

def test_road_lane_change():
    print("== road 미션 장애물 회피 차선 변경 ==")
    ok = True
    m, car, clk = RoadMission(), FakeCar(), FakeClock()
    m.on_start(car, config)
    m._now = clk

    left_blocked = [(15, 270, 400)]              # 좌측 0.4m에 반사
    ok &= check("좌측 막힘/우측 빈 스캔 → 'R'", m.pick_avoid_direction(left_blocked) == "R")
    both = [(15, 270, 2000), (15, 90, 500)]      # 좌 2.0m, 우 0.5m
    ok &= check("좌 여유 > 우 여유 → 'L'", m.pick_avoid_direction(both) == "L")
    ok &= check("스캔 None → 기본 'L'", m.pick_avoid_direction(None) == "L")

    m.step(sensors(bottom=obstacle_frame()), car)   # 장애물 → 기동 시작
    ok &= check("장애물 감지 → 차선 변경 시작", m._lc_phase is not None)

    lc = LANE_CHANGE
    for _ in range(200):
        if m._lc_phase is None:
            break
        clk.advance(0.2)
        m.step(sensors(bottom=obstacle_frame()), car)
    expected = ["L"] * lc["pulses"] + ["R"] * (2 * lc["pulses"]) + ["L"] * lc["pulses"]
    ok &= check("기동 종료 (페이즈 None)", m._lc_phase is None)
    ok &= check(f"펄스 시퀀스 {expected} 일치", car.pulses == expected)
    ok &= check("종료 시 steer('F') + 주행 속도 복귀",
                car.steers[-1] == "F" and car.drives[-1] == config.DRIVE_SPEED)

    n_pulses = len(car.pulses)
    m.step(sensors(bottom=obstacle_frame()), car)   # cooldown 중 재트리거 금지
    ok &= check("cooldown 중 재트리거 안 함",
                m._lc_phase is None and len(car.pulses) == n_pulses)
    return ok


# ---- 3b. road 차선 변경: 오도메트리 거리 조건 (타이밍 폴백 검증) ----

def test_lane_change_distance_mode():
    print("== road 차선 변경 오도메트리 거리 조건 ==")
    ok = True
    orig_out_m = LANE_CHANGE["out_m"]
    try:
        LANE_CHANGE["out_m"] = 1.0

        # (a) pose_conf 충족: 이동 거리 도달 시 out_s 전에 OUT 종료
        m, car, clk = RoadMission(), FakeCar(), FakeClock()
        m.on_start(car, config)
        m._now = clk
        m.step(sensors(bottom=obstacle_frame(), pose=(0.0, 0.0, 0.0), pose_conf=0.9), car)
        ok &= check("장애물 → OUT 시작 (pose0 기록)", m._lc_phase == "OUT"
                    and m._lc_pose0 == (0.0, 0.0, 0.0))
        clk.advance(0.2)  # out_s(1.5)보다 훨씬 이른 시점
        m.step(sensors(bottom=blank(), pose=(1.2, 0.0, 0.0), pose_conf=0.9), car)
        ok &= check("1.2m 이동(>=out_m 1.0) → 타이밍 전에 BACK 전환", m._lc_phase == "BACK")

        # (b) pose_conf=0(미보정): 거리 무시, 기존 타이밍 그대로
        m2, car2, clk2 = RoadMission(), FakeCar(), FakeClock()
        m2.on_start(car2, config)
        m2._now = clk2
        m2.step(sensors(bottom=obstacle_frame(), pose=(0.0, 0.0, 0.0), pose_conf=0.0), car2)
        clk2.advance(0.2)
        m2.step(sensors(bottom=blank(), pose=(5.0, 0.0, 0.0), pose_conf=0.0), car2)
        ok &= check("conf=0이면 거리 무시 — 0.2s에는 아직 OUT (fail-inert)",
                    m2._lc_phase == "OUT")
        clk2.advance(LANE_CHANGE["out_s"])
        m2.step(sensors(bottom=blank(), pose=(5.0, 0.0, 0.0), pose_conf=0.0), car2)
        ok &= check("conf=0이어도 타이밍(out_s)으로는 정상 전환", m2._lc_phase == "BACK")
    finally:
        LANE_CHANGE["out_m"] = orig_out_m
    return ok


# ---- 4. t_parking 미션 end-to-end ----

def test_t_parking():
    print("== t_parking 상태머신 end-to-end ==")
    ok = True
    m, car, clk = TParkingMission(), FakeCar(), FakeClock()
    m.on_start(car, config)
    m._now = clk
    p = T_PARKING

    any_scan = [(15, 0, 3000)]
    for _ in range(p["map_scans"]):
        clk.advance(0.1)
        m.step(sensors(scan=any_scan), car)
    ok &= check("MAP_BUILD → FIND_SLOT (스캔 누적 완료)", m.state == "FIND_SLOT")

    # 우측(side='R') 주차 차량 2대 + 사이 갭: bearing -120/-75 (원시 60/105), 1.0m
    slot_scan = [(15, 58, 1000), (15, 60, 1000), (15, 103, 1000), (15, 105, 1000)]
    m.step(sensors(scan=slot_scan), car)
    ok &= check("갭 폭>=slot_gap_min → REVERSE_ALIGN", m.state == "REVERSE_ALIGN")
    ok &= check("슬롯 위치 기록됨", m._slot is not None)

    m.step(sensors(rear=rear_lines_frame(100, 300)), car)  # 중점 200, 오차 -40
    ok &= check("주차선 치우침 → 후진 반전 조향 'R'", car.steers[-1] == "R")

    for _ in range(p["align_ticks"]):
        clk.advance(0.1)
        m.step(sensors(rear=rear_lines_frame(140, 340)), car)  # 중점 240 = 중앙
    ok &= check("정렬 연속 판정 → PARK", m.state == "PARK")

    parked_scan = [(15, 0, 200)]  # 후방 0.275m (뒤범퍼 기준) <= rear_stop_m
    for _ in range(300):
        if m.state == "DONE":
            break
        clk.advance(0.2)
        m.step(sensors(rear=rear_lines_frame(140, 340), scan=parked_scan), car)
    ok &= check("PARK 기동 완료 → DONE", m.state == "DONE")
    ok &= check("TURN_IN 'L' 펄스 발생 (side='R' 후진 진입)", "L" in car.pulses)
    ok &= check("STRAIGHTEN 'R' 펄스 발생", "R" in car.pulses)
    ok &= check("후진 구동 발생", any(v < 0 for v in car.drives))

    m.step(sensors(), car)
    ok &= check("DONE에서 stop() 호출", ("stop", None) in car.calls)
    return ok


def main():
    results = [
        test_white_discrimination(),
        test_vendor_fallback_sync(),
        test_traffic_fsm(),
        test_road_lane_change(),
        test_lane_change_distance_mode(),
        test_t_parking(),
    ]
    passed = all(results)
    print("\n결과:", "이상 없음" if passed else "위 [X] 항목 확인 필요")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
