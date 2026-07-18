"""디지털 트윈 재현용 주행 로그 — 순수 함수/클래스, ROS 불필요.

mission_node._tick()이 매 제어 틱(LOOP_HZ)마다 현재 튜닝 설정값(tuning.install()
바인딩의 *라이브* 값 — ros2 param set으로 기동 중 바뀌어도 그 시점 값을 반영)과
실제 발행된 조향/속도/게이트 명령을 타임스탬프와 함께 한 줄(JSON)로 남긴다.
파일 하나 = 주행 세션 하나. 디지털 트윈이 이 파일을 그대로 재생하면 같은
설정·같은 명령 시퀀스로 주행을 재현해 볼 수 있다 (지도 교수 피드백,
2026-07-18).

오프라인 셀프테스트: python3 -m autodrive_skku_ros.drive_logger --selftest
"""
import json
import os
import time


def make_log_path(log_dir, mission="drive"):
    """log_dir 아래 '<mission>_<기동시각>.jsonl' 경로를 만들고 디렉토리를
    생성해둔다(파일 자체는 DriveLogger가 open 시 생성)."""
    os.makedirs(log_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(log_dir, f"{mission}_{stamp}.jsonl")


def snapshot_bindings(bindings):
    """tuning.install()이 반환한 {param_name: Binding}에서 *현재* 값만 읽어
    JSON 직렬화 가능한 평탄 dict로 만든다 — dict 바인딩은 container[key],
    attr 바인딩은 getattr(container, key)로 읽으므로 ros2 param set으로
    기동 중 바뀐 값도 그 시점 그대로 반영된다(정적 사본이 아님)."""
    out = {}
    for name, b in bindings.items():
        out[name] = b.container[b.key] if b.kind == "dict" else getattr(b.container, b.key)
    return out


class DriveLogger:
    """한 줄 = 한 틱. append-only JSON Lines — 세션 중간에 봐도 앞부분은 이미
    유효한 JSON이라 tail -f 등으로 실시간 확인 가능."""

    def __init__(self, path):
        self.path = path
        self._f = open(path, "a", buffering=1, encoding="utf-8")

    def log(self, tuning_snapshot, commands, mission=None, state=None, t=None):
        record = {
            "t": time.time() if t is None else t,
            "mission": mission,
            "state": state,
            "tuning": tuning_snapshot,
            "commands": commands,
        }
        self._f.write(json.dumps(record, ensure_ascii=False) + "\n")

    def close(self):
        self._f.close()


def selftest():
    """ROS 불필요 — make_log_path/DriveLogger/snapshot_bindings를 임시 디렉토리에서
    검증한다."""
    import shutil
    import tempfile
    from collections import namedtuple

    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    tmp_dir = tempfile.mkdtemp(prefix="drive_log_selftest_")
    try:
        log_dir = os.path.join(tmp_dir, "logs")
        path = make_log_path(log_dir, mission="road")
        check("make_log_path: 디렉토리를 미리 생성함", os.path.isdir(log_dir))
        check("make_log_path: 파일명이 미션명으로 시작하고 .jsonl로 끝남",
              os.path.basename(path).startswith("road_") and path.endswith(".jsonl"))

        Binding = namedtuple("Binding", "kind container key default")
        live_dict = {"speed": 80}
        bindings = {"speed.drive": Binding("dict", live_dict, "speed", 80)}
        snap1 = snapshot_bindings(bindings)
        check("snapshot_bindings: dict 바인딩 현재값 읽음", snap1 == {"speed.drive": 80})
        live_dict["speed"] = 120  # ros2 param set으로 기동 중 바뀐 상황 시뮬
        snap2 = snapshot_bindings(bindings)
        check("snapshot_bindings: 정적 사본이 아니라 매번 라이브 값을 읽음",
              snap2 == {"speed.drive": 120})

        class _Obj:
            pass
        obj = _Obj()
        obj.gap_s = 0.15
        bindings["steer.pulse_gap_s"] = Binding("attr", obj, "gap_s", 0.15)
        snap3 = snapshot_bindings(bindings)
        check("snapshot_bindings: attr 바인딩도 함께 읽음",
              snap3 == {"speed.drive": 120, "steer.pulse_gap_s": 0.15})

        logger = DriveLogger(path)
        logger.log(snap2, {"steer": "F", "drive": 80, "go": True},
                   mission="road", state=1, t=1000.0)
        logger.log(snap3, {"steer": "L", "drive": 80, "go": True},
                   mission="road", state=1, t=1000.1)
        logger.close()

        with open(path, "r", encoding="utf-8") as f:
            lines = [json.loads(line) for line in f if line.strip()]
        check("JSONL: 두 번 log() -> 두 줄, 각각 유효한 JSON", len(lines) == 2)
        check("기록된 타임스탬프가 그대로 보존됨",
              lines[0]["t"] == 1000.0 and lines[1]["t"] == 1000.1)
        check("mission/state 필드가 그대로 보존됨",
              lines[0]["mission"] == "road" and lines[0]["state"] == 1)
        check("틱마다 다른 tuning 스냅샷이 그대로 보존됨(라이브 변경 반영)",
              lines[0]["tuning"]["speed.drive"] == 120
              and lines[1]["tuning"]["steer.pulse_gap_s"] == 0.15)
        check("commands 필드가 그대로 보존됨(조향/속도/게이트)",
              lines[0]["commands"] == {"steer": "F", "drive": 80, "go": True}
              and lines[1]["commands"]["steer"] == "L")
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    print("drive_logger.py는 라이브러리 모듈입니다 — "
          "python3 -m autodrive_skku_ros.drive_logger --selftest")
