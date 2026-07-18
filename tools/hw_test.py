#!/usr/bin/env python3
"""실제 아두이노에 연결해 전진/조향 모듈을 직접 눈으로 확인하는 수동 테스트.

tools/run_tests.py(하드웨어 없는 로직 스모크 테스트)와 달리 이 스크립트는
실제로 모터를 움직인다 — 반드시 바퀴를 지면에서 띄운 상태에서 실행할 것.
기존에 검증된 시리얼 프로토콜(README '시리얼 프로토콜' 절, ArduinoNode)을
그대로 사용하므로 펌웨어/배선 변경이 필요 없다.

사용법:
    python tools/hw_test.py                       # 전진+조향 둘 다
    python tools/hw_test.py --forward              # 전진 모듈만
    python tools/hw_test.py --no-steer              # 조향만 빼고 (=전진만)
    python tools/hw_test.py --port /dev/ttyACM0 --speed 80 --duration 2
    python tools/hw_test.py --pot                   # 조향 POT 캘리브레이션 span 진단만
                                                     # (전진/조향 테스트는 생략)
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                 "autodrive_skku_ros"))

from autodrive_skku_ros import config
from autodrive_skku_ros.missions.road import LANE_CHANGE
from autodrive_skku_ros.nodes.arduino_node import ArduinoNode
from autodrive_skku_ros.nodes.ports import autodetect_ports

MODULES = {
    "forward": "전진(좌우 구동 모터) 테스트",
    "steer": "조향(스티어링 모터) 테스트",
}


def test_forward(car, speed, duration):
    print(f"\n### [forward] {MODULES['forward']} — 속도 {speed}, {duration}초")
    car.go()
    car.drive(speed)
    time.sleep(duration)
    car.drive(0)
    car.stop()
    print("  -> 바퀴가 전진 방향으로 돌았는지 확인하세요")


def test_steer(car, pulse_gap_s):
    print(f"\n### [steer] {MODULES['steer']} — L 펄스 -> 정지 -> R 펄스 -> 정지")
    car.steer("L")
    print("  -> L 펄스 전송: 바퀴가 좌측으로 돌아가는지 확인하세요")
    time.sleep(pulse_gap_s)
    car.steer("F")
    time.sleep(1.0)
    car.steer("R")
    print("  -> R 펄스 전송: 바퀴가 우측으로 돌아가는지 확인하세요")
    time.sleep(pulse_gap_s)
    car.steer("F")


def test_pot(car):
    """조향 POT 좌/우 풀락 ADC를 수동으로 1회 측정한다 — ArduinoNode.calibrate_steering()
    (자동 좌/우 풀락 스윕 + 중앙값 필터)을 사람이 직접 실행할 때만 호출한다.

    지도 교수 피드백(2026-07-18)에 따라 arduino_node는 더 이상 기동 시마다
    이 스윕을 자동으로 돌리지 않는다(중앙 정렬은 하드웨어 텐션 스프링이 담당).
    이 스크립트로 한 번 측정해 나온 adc_left/adc_right를 bringup.launch.py의
    steering_adc_left/steering_adc_right 인자로 고정 입력해 재사용할 것."""
    print("\n### [pot] 조향 POT 좌/우 풀락 수동 측정 — 좌/우 풀락 스윕 중...")
    adc_left, adc_right = car.calibrate_steering()
    if adc_left is None:
        print("  -> POT 미검출/응답 없음 — 미장착이거나 배선(A6/기준전압) 문제로 추정")
        return
    span = abs(adc_left - adc_right)
    min_span = config.ODOMETRY["min_pot_span_counts"]
    print(f"  -> adc_left={adc_left}, adc_right={adc_right}, span={span} "
          f"(odometry_node 신뢰 기준: min_pot_span_counts={min_span})")
    if span < min_span:
        print(f"  ⚠ span이 기준 미달 — POT 축 결합(조향 회전이 POT에 제대로 "
              "전달되는지)·기준전압(5V/GND) 점검 필요")
        print("     이대로면 각도 해상도가 부족해 odometry_node가 펄스 폴백으로 동작합니다")
    else:
        print("  span 양호 — odometry_node가 POT 각도를 신뢰해 사용합니다")
        print(f"  -> bringup.launch.py에 고정 입력: "
              f"steering_adc_left:={adc_left} steering_adc_right:={adc_right}")


def build_parser():
    parser = argparse.ArgumentParser(description="실차 전진/조향 모듈 수동 테스트")
    parser.add_argument("--port", default=config.ARDUINO_PORT,
                        help="아두이노 시리얼 포트 (기본: 자동 감지)")
    parser.add_argument("--speed", type=int, default=config.SLOW_SPEED,
                        help=f"전진 테스트 속도, -255..255 (기본: {config.SLOW_SPEED} — 저속 권장)")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="전진 테스트 지속 시간(초, 기본 2.0)")
    for key, desc in MODULES.items():
        parser.add_argument(f"--{key}", dest=key, action=argparse.BooleanOptionalAction,
                            default=None, help=desc)
    parser.add_argument("--pot", action="store_true",
                        help="조향 POT 캘리브레이션 span 진단만 실행 "
                             "(전진/조향 모듈 없이 이것만 하고 종료)")
    return parser


def select_modules(args):
    """아무 플래그도 없으면 전체, --x가 하나라도 있으면 opt-in(그것만),
    --no-x만 있으면 opt-out(그것만 제외)."""
    explicit_true = [k for k in MODULES if getattr(args, k) is True]
    if explicit_true:
        return explicit_true
    explicit_false = [k for k in MODULES if getattr(args, k) is False]
    if explicit_false:
        return [k for k in MODULES if k not in explicit_false]
    return list(MODULES)


def main():
    args = build_parser().parse_args()

    port = args.port
    if port is None:
        port, _ = autodetect_ports()
    if port is None:
        print("[hw_test] 아두이노 포트를 찾지 못했습니다 — --port로 직접 지정하세요")
        sys.exit(1)

    if args.pot:
        print(f"POT 캘리브레이션 span 진단 (포트: {port})")
        print("!! 조향 모터가 좌우로 스윕합니다 — 바퀴가 걸리지 않는지 확인하세요 !!")
        if input("계속하려면 y 입력 > ").strip().lower() != "y":
            print("취소됨")
            return
        car = ArduinoNode(port, config.ARDUINO_BAUD)
        try:
            test_pot(car)
        except KeyboardInterrupt:
            print("\n중단됨")
        finally:
            car.close()
        return

    selected = select_modules(args)
    print(f"실행할 모듈: {', '.join(selected)} (포트: {port})")
    print("!! 바퀴를 지면에서 띄운 상태인지 확인하세요 — 실제로 모터가 움직입니다 !!")
    if input("계속하려면 y 입력 > ").strip().lower() != "y":
        print("취소됨")
        return

    car = ArduinoNode(port, config.ARDUINO_BAUD)
    try:
        if "forward" in selected:
            test_forward(car, args.speed, args.duration)
        if "steer" in selected:
            test_steer(car, LANE_CHANGE["pulse_gap_s"])
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        car.stop()
        car.close()
        print("\n[hw_test] 차량 정지 및 연결 종료")


if __name__ == "__main__":
    main()
