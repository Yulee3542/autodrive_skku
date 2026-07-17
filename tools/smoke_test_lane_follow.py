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
    follow_lane, analyze_lane_poi, LANE_POI, _fit_lane_circle, _circle_x_at_y)
from autodrive_skku_ros.vendor import Function_Library as fl


class FakeCar:
    """실제 ArduinoNode 대신 steer() 호출만 기록하는 더미."""

    def __init__(self):
        self.calls = []

    def steer(self, direction):
        self.calls.append(direction)


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
    print("== _fit_lane_circle (Circular Hough Transform) ==")
    ok = True
    cfg = dict(LANE_POI)

    ring = np.zeros((600, 600), dtype=np.uint8)
    cv2.circle(ring, (300, 300), 200, 255, 4)  # hough_min/max_radius_px(150~2000) 범위 안
    fit = _fit_lane_circle(ring, x_off=0, y_off=0, cfg=cfg)
    ok &= check("뚜렷한 원 -> 검출됨", fit is not None)
    if fit is not None:
        cx, cy, r = fit
        ok &= check(f"중심 근사 일치 ({cx:.0f},{cy:.0f})~=(300,300)",
                    abs(cx - 300) < 20 and abs(cy - 300) < 20)
        ok &= check(f"반지름 근사 일치 ({r:.0f})~=200",
                    abs(r - 200) < 20)
        ok &= check("x_off/y_off 절대좌표 반영",
                    _fit_lane_circle(ring, x_off=50, y_off=30, cfg=cfg)[0] - cx == 50)

    blank_img = np.zeros((400, 400), dtype=np.uint8)
    ok &= check("빈 이미지 -> None(원 없음)",
                _fit_lane_circle(blank_img, 0, 0, cfg) is None)
    return ok


def test_analyze_lane_poi_straight_unaffected():
    """핵심 회귀 기준: 직선(원이 아닌) 프레임에서는 Hough 원이 안 잡히거나
    안 맞아 circle=None으로 폴백, raw_target은 기존 밴드 클러스터링 결과
    그대로(원 검출 도입 전과 동일)여야 한다."""
    print("== analyze_lane_poi 직선 2줄 프레임 -> Hough 폴백(회귀 없음) ==")
    ok = True
    frame = make_two_line_frame()
    details = analyze_lane_poi(frame)
    ok &= check("직선 2줄 -> circle=None (원이 아니므로 폴백)",
                details["circle"] is None)
    expected = 200 + 0.75 * (440 - 200)  # _poi_pick_right_lane_center 보간값
    ok &= check(f"raw_target 기존 밴드 보간값과 일치 (~{expected:.0f})",
                details["raw_target"] is not None
                and abs(details["raw_target"] - expected) < 5)
    return ok


def main():
    results = [
        test_follow_lane_no_crash(),
        test_portrait_rotation_shapes(),
        test_circle_x_at_y(),
        test_fit_lane_circle(),
        test_analyze_lane_poi_straight_unaffected(),
    ]
    passed = all(results)
    print("\n결과:", "이상 없음" if passed else "위 [X] 항목 확인 필요")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
