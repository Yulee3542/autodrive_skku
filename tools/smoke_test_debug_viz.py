#!/usr/bin/env python3
"""디버그 오버레이(debug_viz.py + 감지기 debug out-dict) 스모크 테스트.

headless 검증: 합성 프레임으로 analyze/감지기를 돌려 debug dict가 채워지는지,
각 draw_* 함수가 입력과 같은 크기의 vis를 예외 없이 반환하는지, 그리고
analyze_lane_poi의 offset 부호가 follow_lane_poi의 실제 조향과 일치하는지.

사용법: python tools/smoke_test_debug_viz.py
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
    sys.exit(1)

from autodrive_skku_ros import config, debug_viz
from autodrive_skku_ros.missions import lane_follow
from autodrive_skku_ros.missions.road import detect_obstacle_ahead, OBSTACLE_CAM
from autodrive_skku_ros.missions.traffic import TrafficMission, detect_light_color
from autodrive_skku_ros.missions.t_parking import TParkingMission

from smoke_test_missions import (FakeCar, blank, obstacle_frame, stop_line_frame,
                                 rear_lines_frame, color_frame, H, W)


def check(name, condition):
    status = "OK" if condition else "X "
    print(f"  [{status}] {name}")
    return bool(condition)


def lane_poi_frame():
    """POI 사다리꼴 안에 좌실선/중앙점선/우실선 세 줄 — 우측 차선 중앙이
    프레임 중앙보다 오른쪽에 있어 'R' 조향이 나와야 하는 프레임."""
    f = blank()
    for x in (150, 240, 330):
        cv2.line(f, (x, 0), (x, H - 1), (255, 255, 255), 6)
    return f


def _vis_ok(vis, frame):
    return vis is not None and vis.shape == frame.shape and vis.dtype == frame.dtype


def test_lane_poi_analysis():
    print("== analyze_lane_poi / follow_lane_poi 일관성 ==")
    ok = True
    frame = lane_poi_frame()

    details = lane_follow.analyze_lane_poi(frame)
    ok &= check("details 스키마 (cx/bands/path_points/raw_target)",
                details is not None and
                all(k in details for k in ("cx", "w", "h", "bands", "path_points", "raw_target")))
    ok &= check("밴드 수 == n_bands", len(details["bands"]) == lane_follow.LANE_POI["n_bands"])
    ok &= check("우측 차선 중앙 검출 (raw_target 존재)", details["raw_target"] is not None)

    tracker, car = lane_follow.LaneCenterTracker(), FakeCar()
    d2 = lane_follow.follow_lane_poi(tracker, car, frame)
    ok &= check("follow_lane_poi가 details 반환 (direction 포함)",
                d2 is not None and d2.get("direction") is not None)
    # analyze의 offset 부호와 실제 조향이 일치해야 한다 (첫 프레임: smoothed=raw)
    expected = "R" if d2["offset"] > d2["deadzone"] else \
               "L" if d2["offset"] < -d2["deadzone"] else "F"
    ok &= check(f"offset({d2['offset']:+.0f}px) 부호와 조향({car.steers[-1]}) 일치",
                car.steers[-1] == expected == d2["direction"])
    ok &= check("우측 치우침 프레임 → 'R' 조향", car.steers[-1] == "R")

    ok &= check("None 프레임 → None (조향 없음)",
                lane_follow.analyze_lane_poi(None) is None)
    return ok


def test_debug_out_dicts():
    print("== 감지기 debug out-dict 채움 (반환값 불변) ==")
    ok = True

    dbg = {}
    r = detect_obstacle_ahead(obstacle_frame(), OBSTACLE_CAM, debug=dbg)
    ok &= check("장애물: debug 채움 + 판정 True 유지",
                r is True and dbg.get("result") is True and "roi" in dbg and dbg["blobs"])
    dbg2 = {}
    r2 = detect_obstacle_ahead(blank(), OBSTACLE_CAM, debug=dbg2)
    ok &= check("빈 프레임: 판정 False + debug result False",
                r2 is False and dbg2.get("result") is False)

    m = TrafficMission()
    dbg3 = {}
    r3 = m.stop_line_detected(stop_line_frame(), debug=dbg3)
    ok &= check("정지선: debug 채움 + 판정 True 유지",
                r3 is True and dbg3.get("result") is True and "row_frac" in dbg3)

    dbg4 = {}
    r4 = detect_light_color(color_frame((0, 0, 255)), 0.005, debug=dbg4)
    ok &= check("신호등: debug 채움 + 'red' 판정 유지",
                r4 == "red" and dbg4.get("result") == "red" and dbg4["red"] > 0)

    tp = TParkingMission()

    class _Car:
        def go(self):
            pass
    tp.on_start(_Car(), config)
    dbg5 = {}
    r5 = tp.reverse_lane_steer(rear_lines_frame(100, 300), debug=dbg5)
    ok &= check("주차선: debug 채움(클러스터 2개) + 조향 유지",
                r5 in ("F", "L", "R") and len(dbg5.get("clusters", [])) == 2
                and dbg5.get("steer") == r5)
    return ok


def test_draw_functions():
    print("== draw_* headless 렌더 (shape/dtype 보존, 예외 없음) ==")
    ok = True
    frame = lane_poi_frame()

    tracker, car = lane_follow.LaneCenterTracker(), FakeCar()
    details = lane_follow.follow_lane_poi(tracker, car, frame)
    ok &= check("draw_lane_poi", _vis_ok(debug_viz.draw_lane_poi(frame, details), frame))
    ok &= check("draw_lane_poi no-data", _vis_ok(debug_viz.draw_lane_poi(frame, None), frame))

    dbg = {}
    detect_obstacle_ahead(obstacle_frame(), OBSTACLE_CAM, debug=dbg)
    ok &= check("draw_obstacle", _vis_ok(debug_viz.draw_obstacle(obstacle_frame(), dbg),
                                         obstacle_frame()))
    ok &= check("draw_obstacle no-data", _vis_ok(debug_viz.draw_obstacle(blank(), {}), blank()))

    m = TrafficMission()
    dbg2 = {}
    m.stop_line_detected(stop_line_frame(), debug=dbg2)
    ok &= check("draw_stop_line", _vis_ok(debug_viz.draw_stop_line(stop_line_frame(), dbg2),
                                          stop_line_frame()))

    dbg3 = {}
    detect_light_color(color_frame((0, 255, 0)), 0.005, debug=dbg3)
    ok &= check("draw_traffic_light",
                _vis_ok(debug_viz.draw_traffic_light(color_frame((0, 255, 0)), dbg3),
                        color_frame((0, 255, 0))))

    tp = TParkingMission()

    class _Car:
        def go(self):
            pass
    tp.on_start(_Car(), config)
    dbg4 = {}
    tp.reverse_lane_steer(rear_lines_frame(140, 340), debug=dbg4)
    ok &= check("draw_parking_line",
                _vis_ok(debug_viz.draw_parking_line(rear_lines_frame(140, 340), dbg4),
                        rear_lines_frame(140, 340)))
    ok &= check("draw_parking_line no-data",
                _vis_ok(debug_viz.draw_parking_line(blank(), {}), blank()))
    return ok


def main():
    results = [
        test_lane_poi_analysis(),
        test_debug_out_dicts(),
        test_draw_functions(),
    ]
    passed = all(results)
    print("\n결과:", "이상 없음" if passed else "위 [X] 항목 확인 필요")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
