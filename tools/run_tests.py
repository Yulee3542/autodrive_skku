#!/usr/bin/env python3
"""모듈별로 켜고 끌 수 있는 통합 스모크 테스트 러너.

check_env.py / smoke_test_lane_follow.py / smoke_test_missions.py의 개별 테스트
함수를 그대로 재사용한다 — 세 파일 모두 이 스크립트와 무관하게 단독 실행도 계속
가능하다 (해당 파일들은 수정하지 않음).

cv2/numpy가 없는 환경에서도 안전하게 동작한다: 실제로 선택된 모듈만 지연
임포트하고, 의존성이 없어서 실패하면 그 모듈만 SKIP으로 표시하고 나머지 선택된
모듈은 계속 실행한다.

사용법:
    python tools/run_tests.py                    # 전체 실행
    python tools/run_tests.py --list              # 모듈 목록 + 설명만 출력
    python tools/run_tests.py --lidar --camera     # opt-in: 이 둘만 실행
    python tools/run_tests.py --no-parking         # opt-out: parking만 제외하고 전부
"""
import argparse
import importlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # 형제 스크립트(check_env 등) 임포트용


class ModuleUnavailable(RuntimeError):
    """선택된 모듈이 의존성 누락 등으로 실행할 수 없을 때."""


_import_cache = {}


def _load(modname):
    if modname in _import_cache:
        cached = _import_cache[modname]
        if isinstance(cached, Exception):
            raise cached
        return cached
    try:
        mod = importlib.import_module(modname)
    except (ImportError, SystemExit) as e:
        err = ModuleUnavailable(f"{modname} 임포트 실패 (의존성 누락): {e}")
        _import_cache[modname] = err
        raise err
    _import_cache[modname] = mod
    return mod


def _run_env():
    m = _load("check_env")
    return all([m.check_imports(), m.check_cameras(), m.check_serial_ports()])


def _run_lidar():
    return _load("smoke_test_missions").test_lidar_geometry()


def _run_camera():
    return _load("smoke_test_missions").test_white_discrimination()


def _run_lane_follow():
    m = _load("smoke_test_lane_follow")
    return all([m.test_follow_lane_no_crash(), m.test_portrait_rotation_shapes()])


def _run_traffic():
    return _load("smoke_test_missions").test_traffic_fsm()


def _run_road():
    return _load("smoke_test_missions").test_road_lane_change()


def _run_parking():
    return _load("smoke_test_missions").test_t_parking()


MODULES = {
    "env": ("환경/하드웨어 점검 (패키지·카메라·시리얼)", _run_env),
    "lidar": ("라이다 후방 장착 지오메트리 순수 함수", _run_lidar),
    "camera": ("카메라 흰색 형태 구분 (차선/정지선/횡단보도/장애물)", _run_camera),
    "lane_follow": ("차선 추종 통합 경로 + portrait 회전 보정", _run_lane_follow),
    "traffic": ("traffic 미션 상태머신 (정지선/신호등)", _run_traffic),
    "road": ("road 미션 장애물 회피 차선 변경", _run_road),
    "parking": ("t_parking 미션 상태머신 end-to-end", _run_parking),
}


def build_parser():
    parser = argparse.ArgumentParser(description="모듈별 on/off 통합 테스트 러너")
    parser.add_argument("--list", action="store_true", help="모듈 목록만 출력하고 종료")
    for key, (desc, _fn) in MODULES.items():
        parser.add_argument(f"--{key.replace('_', '-')}", dest=key,
                            action=argparse.BooleanOptionalAction, default=None,
                            help=desc)
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

    if args.list:
        print("사용 가능한 테스트 모듈:")
        for key, (desc, _fn) in MODULES.items():
            print(f"  {key:<12} {desc}")
        return

    selected = select_modules(args)
    results = {}
    for key in selected:
        desc, fn = MODULES[key]
        print(f"\n### [{key}] {desc}")
        try:
            results[key] = "PASS" if fn() else "FAIL"
        except ModuleUnavailable as e:
            print(f"  [skip] {e}")
            results[key] = "SKIP"

    print("\n==== 요약 ====")
    for key in selected:
        print(f"  {key:<12} {results[key]}")

    sys.exit(1 if any(v == "FAIL" for v in results.values()) else 0)


if __name__ == "__main__":
    main()
