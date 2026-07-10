#!/usr/bin/env python3
"""차선 추종 통합 경로(lane_follow.follow_lane) + portrait 카메라 회전 보정을
실제 하드웨어 없이 점검하는 스모크 테스트.

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

from autodrive_skku_ros.missions.lane_follow import follow_lane
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


def main():
    results = [
        test_follow_lane_no_crash(),
        test_portrait_rotation_shapes(),
    ]
    passed = all(results)
    print("\n결과:", "이상 없음" if passed else "위 [X] 항목 확인 필요")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
