#!/usr/bin/env python3
"""tuning.py(ROS 파라미터 ↔ 튜닝 dict 바인딩)의 순수 함수 스모크 테스트.

핵심 검증: apply_value가 미션이 매 틱 읽는 바로 그 dict *객체*를 in-place로
바꾸는지(정체성) — rebind 실수가 생기면 파라미터를 set해도 미션에 전파되지
않는 조용한 버그가 되므로 여기서 잡는다. rclpy 불필요.

사용법: python tools/smoke_test_tuning.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                 "autodrive_skku_ros"))

from autodrive_skku_ros import config, tuning
from autodrive_skku_ros.missions import lane_follow, road, t_parking, traffic


def check(name, condition):
    status = "OK" if condition else "X "
    print(f"  [{status}] {name}")
    return bool(condition)


def _leaf_count(d):
    return sum(_leaf_count(v) if isinstance(v, dict) else 1 for v in d.values())


def test_flatten_coverage():
    """flatten_bindings가 모든 튜닝 dict의 모든 리프 키를 빠짐없이 커버하는지."""
    print("== flatten 전수 커버리지 ==")
    ok = True
    dicts, attrs = tuning.tunable_dicts(), tuning.tunable_attrs()
    bindings = tuning.flatten_bindings(dicts, attrs)
    expected = sum(_leaf_count(d) for d in dicts.values()) + len(attrs)
    ok &= check(f"mission 바인딩 수 {len(bindings)} == 리프 키 합 {expected}",
                len(bindings) == expected)
    ok &= check("대표 키 존재 (lane_poi.n_bands / t_parking.side / speed.drive)",
                all(k in bindings for k in
                    ("lane_poi.n_bands", "t_parking.side", "speed.drive")))

    odo = tuning.flatten_bindings(tuning.odometry_tunable_dicts())
    ok &= check("odometry 중첩 dict 평탄화 (odometry.goodfeatures.maxCorners)",
                "odometry.goodfeatures.maxCorners" in odo)
    ok &= check("camera_mount.height_m(None) → 파라미터 기본값 0.0",
                tuning.to_param_value(odo["camera_mount.height_m"].default)
                == tuning.NONE_SENTINEL)
    return ok


def test_apply_identity():
    """apply_value가 미션이 참조하는 바로 그 dict 객체를 in-place로 바꾸는지."""
    print("== in-place 반영 정체성 ==")
    ok = True
    bindings = tuning.flatten_bindings(tuning.tunable_dicts(), tuning.tunable_attrs())

    orig = lane_follow.LANE_POI["n_bands"]
    try:
        tuning.apply_value(bindings["lane_poi.n_bands"], 5)
        # road.py가 기본 인자로 잡아둔 객체(follow_lane_poi의 config=LANE_POI)와
        # 동일 객체이므로 그쪽에서도 보여야 한다
        ok &= check("LANE_POI in-place 반영 (5)",
                    lane_follow.LANE_POI["n_bands"] == 5)
        ok &= check("int 타입 유지", isinstance(lane_follow.LANE_POI["n_bands"], int))
        ok &= check("binding container가 모듈 dict와 동일 객체",
                    bindings["lane_poi.n_bands"].container is lane_follow.LANE_POI)
    finally:
        lane_follow.LANE_POI["n_bands"] = orig

    # t_parking은 on_start에서 self.p = T_PARKING 참조를 잡는다 — 그 후 set해도 보여야 함
    m = t_parking.TParkingMission()
    orig_hold = t_parking.T_PARKING["hold_s"]
    try:
        class _Car:
            def go(self):
                pass
        m.on_start(_Car(), config)
        tuning.apply_value(bindings["t_parking.hold_s"], 5.0)
        ok &= check("on_start 후 set해도 self.p에 전파 (참조 동일성)", m.p["hold_s"] == 5.0)
    finally:
        t_parking.T_PARKING["hold_s"] = orig_hold

    orig_speed = config.DRIVE_SPEED
    try:
        tuning.apply_value(bindings["speed.drive"], 120)
        ok &= check("attr 바인딩 setattr 반영 (config.DRIVE_SPEED)", config.DRIVE_SPEED == 120)
    finally:
        config.DRIVE_SPEED = orig_speed

    orig_ratio = traffic.TRAFFIC_PIXEL_RATIO
    try:
        tuning.apply_value(bindings["traffic.pixel_ratio"], 0.01)
        ok &= check("모듈 스칼라 setattr 반영 (TRAFFIC_PIXEL_RATIO)",
                    traffic.TRAFFIC_PIXEL_RATIO == 0.01)
    finally:
        traffic.TRAFFIC_PIXEL_RATIO = orig_ratio
    return ok


def test_type_roundtrip():
    """tuple/None/str 값의 파라미터 표현 ↔ 파이썬 값 왕복."""
    print("== 타입 라운드트립 ==")
    ok = True
    bindings = tuning.flatten_bindings(tuning.tunable_dicts(), tuning.tunable_attrs())

    b_roi = bindings["lane_poi.roi_frac"]
    ok &= check("tuple → list 파라미터", tuning.to_param_value(b_roi.default) == [0.67, 0.98])
    orig = lane_follow.LANE_POI["roi_frac"]
    try:
        tuning.apply_value(b_roi, [0.5, 0.9])
        ok &= check("list set → tuple 복원", lane_follow.LANE_POI["roi_frac"] == (0.5, 0.9)
                    and isinstance(lane_follow.LANE_POI["roi_frac"], tuple))
        raised = False
        try:
            tuning.apply_value(b_roi, [0.5])
        except ValueError:
            raised = True
        ok &= check("배열 길이 불일치 → ValueError", raised)
    finally:
        lane_follow.LANE_POI["roi_frac"] = orig

    b_ws = bindings["obstacle_cam.white_s_max"]
    orig_ws = road.OBSTACLE_CAM["white_s_max"]
    try:
        ok &= check("None 기본값 → 파라미터 0.0",
                    tuning.to_param_value(b_ws.default) == 0.0)
        tuning.apply_value(b_ws, 80.0)
        ok &= check("0이 아닌 값 set → override 활성 (80.0)",
                    road.OBSTACLE_CAM["white_s_max"] == 80.0)
        tuning.apply_value(b_ws, 0.0)
        ok &= check("0.0 set → None 복원 (공유 WHITE_HSV로 폴백)",
                    road.OBSTACLE_CAM["white_s_max"] is None)
    finally:
        road.OBSTACLE_CAM["white_s_max"] = orig_ws

    b_side = bindings["t_parking.side"]
    orig_side = t_parking.T_PARKING["side"]
    try:
        tuning.apply_value(b_side, "L")
        ok &= check("str 파라미터 반영 (side='L')", t_parking.T_PARKING["side"] == "L")
    finally:
        t_parking.T_PARKING["side"] = orig_side
    return ok


def main():
    results = [
        test_flatten_coverage(),
        test_apply_identity(),
        test_type_roundtrip(),
    ]
    passed = all(results)
    print("\n결과:", "이상 없음" if passed else "위 [X] 항목 확인 필요")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
