#!/usr/bin/env python3
"""나머지 미션(road ③④ / traffic ① / t_parking ①~④) 테스트 구현을 실제
하드웨어 없이 점검하는 스모크 테스트.

- 라이다: 후방 장착(0도=차량 후방) 지오메트리 순수 함수 검증 + 합성 스캔
- 카메라: 합성 프레임으로 흰색끼리(차선/정지선/횡단보도/장애물) 교차 오검출 검증
- 미션: FakeCar(호출 기록) + FakeClock(_now 주입)으로 상태머신 end-to-end

사용법: python tools/smoke_test_missions.py
(cv2/numpy 필요 — pip install opencv-python-headless numpy)
"""
import math
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
from autodrive_skku_ros.missions.road import RoadMission, detect_obstacle_ahead
from autodrive_skku_ros.missions.traffic import TrafficMission
from autodrive_skku_ros.missions.t_parking import TParkingMission
from autodrive_skku_ros.nodes.lidar_node import (vehicle_bearing_deg, filter_self,
                                  rear_min_m, side_clearance_m,
                                  laserscan_msg_to_tuples, scan_to_ranges)

MOUNT = config.LIDAR_MOUNT
MASK = config.LIDAR_SELF_MASK_DEG


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


def sensors(top=None, bottom=None, rear=None, scan=None):
    return {"top": top, "bottom": bottom, "rear": rear,
            "lidar_min_m": None, "lidar_scan": scan, "state": None}


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


# ---- 1. 라이다 후방 0도 지오메트리 ----

def test_lidar_geometry():
    print("== 라이다 후방 장착(0도=후방) 지오메트리 ==")
    ok = True
    ok &= check("원시 0도 → 차량 bearing ±180 (후방→전방 기준)",
                abs(abs(vehicle_bearing_deg(0, MOUNT)) - 180.0) < 1e-9)
    ok &= check("원시 180도 → bearing 0 (차량 전방)",
                abs(vehicle_bearing_deg(180, MOUNT)) < 1e-9)
    ok &= check("전방 wedge(|b|<75)는 자차 반사로 제거",
                filter_self([(15, 180, 500), (15, 170, 500)], MOUNT, MASK) == [])
    ok &= check("후방/측면 반사는 유지",
                len(filter_self([(15, 0, 500), (15, 60, 800)], MOUNT, MASK)) == 2)
    r = rear_min_m([(15, 0, 250)], MOUNT, config.LIDAR_REAR_SECTOR, MASK)
    ok &= check(f"rear_min_m: 라이다 250mm → 뒤범퍼 기준 {r} == 0.325",
                r is not None and abs(r - 0.325) < 1e-9)
    left = side_clearance_m([(15, 270, 800)], "L", MOUNT,
                            config.LIDAR_SIDE_WINDOW_DEG, MASK)
    ok &= check(f"side_clearance_m L (원시 270도=좌측 90도) == 0.8",
                left is not None and abs(left - 0.8) < 1e-9)
    ok &= check("반대쪽 창엔 안 잡힘",
                side_clearance_m([(15, 270, 800)], "R", MOUNT,
                                 config.LIDAR_SIDE_WINDOW_DEG, MASK) is None)
    # 시뮬 실측: 자차 코너 반사(bearing ~76도, 0.26m)는 근거리 게이트로 제거
    corner = (15, 256, 260)
    ok &= check("자차 코너 반사(0.26m)는 측면 여유에서 제외",
                side_clearance_m([corner], "L", MOUNT,
                                 config.LIDAR_SIDE_WINDOW_DEG, MASK) is None)
    both = side_clearance_m([corner, (15, 270, 800)], "L", MOUNT,
                            config.LIDAR_SIDE_WINDOW_DEG, MASK)
    ok &= check("코너 반사 섞여도 실제 장애물(0.8m)만 반환",
                both is not None and abs(both - 0.8) < 1e-9)
    return ok


# ---- 1b. ROS LaserScan ↔ 튜플 변환 (rclpy 없이 순수 함수만 검증) ----

class FakeLaserScan:
    """sensor_msgs/LaserScan을 흉내내는 최소 더미 — rclpy 설치 없이도 테스트 가능."""

    def __init__(self, ranges, angle_min=-math.pi, angle_increment=None,
                 range_min=0.05, range_max=12.0):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = (2 * math.pi / len(ranges)) if angle_increment is None \
            else angle_increment
        self.range_min = range_min
        self.range_max = range_max


def test_scan_conversion():
    print("== ROS LaserScan ↔ 튜플 변환 (laserscan_msg_to_tuples / scan_to_ranges) ==")
    ok = True

    # 4점 스캔: 0=angle_min(-180도), 이후 90도 간격. inf/range 밖은 스킵돼야 함.
    msg = FakeLaserScan(ranges=[1.0, float("inf"), 0.5, 100.0])
    tuples = laserscan_msg_to_tuples(msg)
    ok &= check("inf/range_max 밖 레이는 스킵 (4개 중 2개만 남음)", len(tuples) == 2)
    ok &= check("range_mm 변환 정확 (1.0m → 1000mm)",
                any(abs(dist_mm - 1000.0) < 1e-6 for _q, _a, dist_mm in tuples))
    ok &= check("angle_deg 변환 정확 (angle_min=-180도 그대로)",
                any(abs(angle_deg - (-180.0)) < 1e-6 for _q, angle_deg, _d in tuples))

    # 스캔 없음 → 전부 NaN
    start, end, ranges_empty = scan_to_ranges([], MOUNT, MASK, n_bins=8)
    ok &= check("빈 스캔 → 전부 NaN", all(math.isnan(r) for r in ranges_empty))
    ok &= check("start/end_angle == -pi/+pi",
                abs(start + math.pi) < 1e-9 and abs(end - math.pi) < 1e-9)

    # 자차 반사(전방 wedge)는 제거되고, 후방 반사는 해당 bin에 남아야 함
    start, end, ranges_m = scan_to_ranges([(15, 180, 500), (15, 0, 800)], MOUNT, MASK, n_bins=8)
    ok &= check("전방 자차 반사는 제거되어 전방 bin이 NaN",
                math.isnan(ranges_m[len(ranges_m) // 2]))
    ok &= check("후방(bearing 0) 반사는 남음 (NaN 아닌 bin 존재)",
                any(not math.isnan(r) for r in ranges_m))
    return ok


# ---- 2. 카메라 흰색 구분 (차선/정지선/횡단보도/장애물) ----

def test_white_discrimination():
    print("== 흰색끼리 형태 구분 (교차 오검출 방지) ==")
    cam = config.OBSTACLE_CAM
    m = TrafficMission()
    m.config = config
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


# ---- 3. traffic 미션 FSM (정지선 대기 / 교착 가드 / 빨간불 / cooldown) ----

def test_traffic_fsm():
    print("== traffic 미션 상태머신 ==")
    ok = True
    m, car, clk = TrafficMission(), FakeCar(), FakeClock()
    m.on_start(car, config)
    m._now = clk

    m.step(sensors(bottom=stop_line_frame()), car)
    ok &= check("정지선 → 정지 + wait='line'", m.wait == "line" and car.drives[-1] == 0)

    clk.advance(config.STOP_LINE["wait_max_s"] + 0.1)
    m.step(sensors(bottom=blank()), car)
    ok &= check("신호등 미검출 대기 초과 → 재출발 (교착 방지)",
                m.wait is None and car.drives[-1] == config.DRIVE_SPEED)

    m.step(sensors(top=color_frame((0, 0, 0)), bottom=stop_line_frame()), car)
    ok &= check("cooldown 중 정지선 재트리거 안 함", m.wait is None)

    clk.advance(config.STOP_LINE["cooldown_s"] + 0.1)
    m.step(sensors(top=color_frame((0, 0, 255)), bottom=blank()), car)
    ok &= check("빨간불 → 정지 + wait='red'", m.wait == "red" and car.drives[-1] == 0)

    clk.advance(60.0)
    m.step(sensors(bottom=blank()), car)
    ok &= check("빨간불 대기는 타임아웃 없음 (초록불까지 무기한)",
                m.wait == "red" and car.drives[-1] == 0)

    m.step(sensors(top=color_frame((0, 255, 0)), bottom=blank()), car)
    ok &= check("초록불 → 재출발", m.wait is None and car.drives[-1] == config.DRIVE_SPEED)
    return ok


# ---- 4. road 미션: 회피 방향 + 차선 변경 페이즈 머신 ----

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

    lc = config.LANE_CHANGE
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


# ---- 5. t_parking 미션 end-to-end ----

def test_t_parking():
    print("== t_parking 상태머신 end-to-end ==")
    ok = True
    m, car, clk = TParkingMission(), FakeCar(), FakeClock()
    m.on_start(car, config)
    m._now = clk
    p = config.T_PARKING

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
        test_lidar_geometry(),
        test_scan_conversion(),
        test_white_discrimination(),
        test_traffic_fsm(),
        test_road_lane_change(),
        test_t_parking(),
    ]
    passed = all(results)
    print("\n결과:", "이상 없음" if passed else "위 [X] 항목 확인 필요")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
