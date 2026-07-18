#!/usr/bin/env python3
"""실차 튜닝 세션에서 ros2 param set으로 바꾼 값을 뽑아내는 도구.

`ros2 param dump`로 각 노드의 현재 파라미터를 읽어 dict 기본값과 비교하고:
  1) 기본값과 달라진 값만 모아 ros2 params-file(YAML)로 저장한다
     (기본 config/mission_tuning.yaml). 다음 기동 때
     `ros2 launch autodrive_skku_ros bringup.launch.py mission:=road \
        tuning_params:=config/mission_tuning.yaml` 로 넘기면 튜닝이 유지된다.
  2) 소스 dict에 최종 반영할 diff를 사람이 읽게 출력한다 — 세션이 끝나면
     이 diff대로 dict 기본값을 고쳐 커밋하는 것이 정석이다(YAML은 임시 저장).

ROS 2 환경(WSL, source install/setup.bash)에서 노드가 떠 있는 동안 실행할 것:
    python3 tools/dump_tuning.py                 # 세 노드 전부 시도
    python3 tools/dump_tuning.py --node /mission_node
    python3 tools/dump_tuning.py --out /tmp/tuning.yaml
"""
import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                 "autodrive_skku_ros"))

from autodrive_skku_ros import tuning  # noqa: E402

# 노드별 바인딩(기본값 소스)과, diff를 소스에 반영할 때 찾아갈 위치 안내.
NODE_BINDINGS = {
    "/mission_node": lambda: tuning.flatten_bindings(
        tuning.tunable_dicts(), tuning.tunable_attrs()),
    "/odometry_node": lambda: tuning.flatten_bindings(
        tuning.odometry_tunable_dicts()),
    "/lidar_geometry_node": lambda: tuning.flatten_bindings(
        tuning.lidar_tunable_dicts(), tuning.lidar_tunable_attrs()),
    "/arduino_bridge_node": lambda: tuning.flatten_bindings(
        tuning.arduino_tunable_dicts()),
}

NS_LOCATION = {
    "white": "config.py WHITE_HSV",
    "lane_poi": "missions/lane_follow.py LANE_POI",
    "lane_edge": "missions/lane_follow.py LANE_EDGE",
    "lane_change": "missions/road.py LANE_CHANGE",
    "obstacle_cam": "missions/road.py OBSTACLE_CAM",
    "stop_line": "missions/traffic.py STOP_LINE",
    "t_parking": "missions/t_parking.py T_PARKING",
    "lidar_mount": "config.py LIDAR_MOUNT",
    "camera_mount": "config.py CAMERA_MOUNT",
    "odometry": "config.py ODOMETRY",
    "steering": "config.py ARDUINO_STEERING",
    "speed": "config.py DRIVE_SPEED/SLOW_SPEED",
    "steer": "config.py STEER_PULSE_GAP_S",
    "traffic": "missions/traffic.py TRAFFIC_PIXEL_RATIO",
    "lidar": "config.py LIDAR_SELF_MASK_DEG 등",
}


def dump_node_params(node_name):
    """ros2 param dump <node> → {param_name: value} (도트 평탄화). 노드가 없으면 None."""
    import yaml
    try:
        out = subprocess.run(["ros2", "param", "dump", node_name],
                             capture_output=True, text=True, timeout=20)
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        print(f"[!] ros2 param dump 실행 실패: {e}")
        return None
    if out.returncode != 0:
        print(f"[!] {node_name}: {out.stderr.strip() or '노드를 찾지 못함'}")
        return None
    doc = yaml.safe_load(out.stdout)
    params = doc[node_name]["ros__parameters"]

    flat = {}

    def _walk(prefix, d):
        for k, v in d.items():
            name = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict):
                _walk(name, v)
            else:
                flat[name] = v

    _walk("", params)
    return flat


def normalize(value):
    """비교용 정규화 — YAML의 list와 tuple, int/float 표기 차이를 흡수한다."""
    if isinstance(value, (list, tuple)):
        return [float(x) for x in value]
    if isinstance(value, bool) or isinstance(value, str):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    return value


def diff_node(node_name, bindings):
    """{param_name: (default_param_value, current_value)} — 달라진 것만."""
    current = dump_node_params(node_name)
    if current is None:
        return None
    changed = {}
    for name, b in bindings.items():
        if name not in current:
            continue
        default_pv = tuning.to_param_value(b.default)
        if normalize(current[name]) != normalize(default_pv):
            changed[name] = (default_pv, current[name])
    return changed


def nest(flat):
    """{'a.b.c': v} → {'a': {'b': {'c': v}}} (params-file 형식용)."""
    root = {}
    for name, v in flat.items():
        parts = name.split(".")
        d = root
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = v
    return root


def main():
    parser = argparse.ArgumentParser(description="튜닝 파라미터 diff 추출/저장")
    parser.add_argument("--node", action="append",
                        help="대상 노드 (기본: 알려진 세 노드 전부)")
    parser.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "mission_tuning.yaml"),
        help="변경분 params-file 저장 경로 (변경이 없으면 안 만듦)")
    args = parser.parse_args()

    import yaml

    nodes = args.node or list(NODE_BINDINGS)
    out_doc = {}
    any_change = False

    for node_name in nodes:
        if node_name not in NODE_BINDINGS:
            print(f"[!] 알 수 없는 노드 {node_name} — 대상: {', '.join(NODE_BINDINGS)}")
            continue
        changed = diff_node(node_name, NODE_BINDINGS[node_name]())
        if changed is None:
            continue
        if not changed:
            print(f"[=] {node_name}: 기본값에서 달라진 파라미터 없음")
            continue
        any_change = True
        print(f"\n[{node_name}] 기본값과 달라진 파라미터 — 소스 dict에 반영 후 커밋할 것:")
        for name, (default_pv, cur) in sorted(changed.items()):
            ns = name.split(".")[0]
            loc = NS_LOCATION.get(ns, "?")
            print(f"  {name}: {default_pv!r} → {cur!r}    ({loc})")
        out_doc[node_name] = {"ros__parameters": nest({n: c for n, (_d, c) in changed.items()})}

    if any_change:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            yaml.safe_dump(out_doc, f, allow_unicode=True, sort_keys=True)
        print(f"\n저장됨: {args.out}")
        print("다음 기동 시 유지: ros2 launch autodrive_skku_ros bringup.launch.py "
              f"mission:=<m> tuning_params:={args.out}")
    else:
        print("\n변경분 없음 — params-file을 만들지 않았습니다.")


if __name__ == "__main__":
    main()
