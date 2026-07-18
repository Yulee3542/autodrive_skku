"""실차 튜닝용 ROS 2 런타임 파라미터 ↔ 튜닝 dict 바인딩.

미션/노드가 매 틱 읽는 튜닝 dict(LANE_POI, LANE_CHANGE, T_PARKING …)를 ROS 2
파라미터로 노출해, 주행 중 `ros2 param set /mission_node lane_poi.white_thresh 150`
처럼 재기동 없이 값을 바꿀 수 있게 한다. 세션에서 확정한 값은
tools/dump_tuning.py로 뽑아 dict 기본값에 반영해 커밋한다.

핵심 규약:
- 미션 코드는 모듈 레벨 dict *객체*를 매 틱 참조하므로(예: t_parking의
  `self.p = T_PARKING`, road의 `config=LANE_POI` 기본 인자) 반영은 반드시
  in-place(`d[key] = v`)로 한다 — 모듈 어트리뷰트를 새 dict로 rebind하면
  이미 잡아둔 참조에는 전파되지 않는다. 모듈 스칼라(DRIVE_SPEED 등)만 setattr.
- None(미측정/override 미사용) 값은 ROS 파라미터로 0.0으로 표현한다
  (NONE_SENTINEL). 0.0을 set하면 다시 None으로 돌아간다.
- tuple 값(roi_frac 등)은 double/integer array 파라미터로 노출된다.
- 파라미터 콜백 스레드의 반영은 키 단위 스칼라 대입(GIL 원자적)이라 tick과
  경합해도 값이 찢어지지 않는다. 다만 기동(차선 변경/주차) 중 변경은 다음
  틱부터 적용되므로 기동 중이 아닐 때 바꾸는 것을 권장.

순수 함수(flatten_bindings/apply_value 등)는 ROS 없이 테스트 가능
(tools/smoke_test_tuning.py). rclpy는 install() 안에서만 import한다.
"""
from collections import namedtuple

from . import config

NONE_SENTINEL = 0.0  # None(미측정) 값의 ROS 파라미터 표현 — 0.0 set → None 복원

# kind: "dict"(container[key]=v, in-place) | "attr"(setattr(container, key, v))
Binding = namedtuple("Binding", "kind container key default")


def tunable_dicts():
    """mission_node가 노출할 namespace → 살아있는 튜닝 dict 매핑.

    여기 나열된 dict 객체가 미션이 매 틱 읽는 바로 그 객체여야 한다(위 규약).
    lidar_mount는 mission_node 프로세스의 사본 기준 — lidar_node에도 같은
    이름으로 노출되므로 라이다 캘리브레이션 시 양쪽 노드에 같이 set할 것.
    """
    from .missions import lane_follow, road, t_parking, traffic
    return {
        "white": config.WHITE_HSV,
        "lane_poi": lane_follow.LANE_POI,
        "lane_edge": lane_follow.LANE_EDGE,
        "lane_change": road.LANE_CHANGE,
        "obstacle_cam": road.OBSTACLE_CAM,
        "stop_line": traffic.STOP_LINE,
        "t_parking": t_parking.T_PARKING,
        "lidar_mount": config.LIDAR_MOUNT,
    }


def tunable_attrs():
    """mission_node가 노출할 param 이름 → (모듈, 어트리뷰트) 매핑 —
    dict가 아닌 모듈 스칼라용 setattr 경로."""
    from .missions import traffic
    return {
        "speed.drive": (config, "DRIVE_SPEED"),
        "speed.slow": (config, "SLOW_SPEED"),
        "steer.pulse_gap_s": (config, "STEER_PULSE_GAP_S"),
        "traffic.pixel_ratio": (traffic, "TRAFFIC_PIXEL_RATIO"),
        "lidar.self_mask_deg": (config, "LIDAR_SELF_MASK_DEG"),
        "lidar.rear_sector": (config, "LIDAR_REAR_SECTOR"),
        "lidar.side_window_deg": (config, "LIDAR_SIDE_WINDOW_DEG"),
    }


def odometry_tunable_dicts():
    """odometry_node가 노출할 dict — 실차에서 rebuild 없이 CAMERA_MOUNT/
    ODOMETRY 실측값을 넣을 수 있게 한다 (0.0=미측정 규약)."""
    return {
        "camera_mount": config.CAMERA_MOUNT,
        "odometry": config.ODOMETRY,
    }


def arduino_tunable_dicts():
    """arduino_bridge_node가 노출할 dict — 라이브 조향 POT 스트림 칼만필터
    Q/R(config.ARDUINO_STEERING). calibrate_steering()의 중앙값 스윕과는
    무관 — /car/steering_angle 라이브 발행 경로만 튜닝한다."""
    return {"steering": config.ARDUINO_STEERING}


def lidar_tunable_dicts():
    """lidar_node(lidar_geometry_node)가 노출할 dict."""
    return {"lidar_mount": config.LIDAR_MOUNT}


def lidar_tunable_attrs():
    return {
        "lidar.self_mask_deg": (config, "LIDAR_SELF_MASK_DEG"),
        "lidar.rear_sector": (config, "LIDAR_REAR_SECTOR"),
    }


def flatten_bindings(dicts=None, attrs=None):
    """namespace dict/attr 매핑을 {param_name: Binding}으로 평탄화한다.
    중첩 dict(예: ODOMETRY['goodfeatures'])는 'ns.sub.key'로 내려간다."""
    out = {}

    def _walk(ns, d):
        for key, val in d.items():
            name = f"{ns}.{key}"
            if isinstance(val, dict):
                _walk(name, val)
            else:
                out[name] = Binding("dict", d, key, val)

    for ns, d in (dicts or {}).items():
        _walk(ns, d)
    for name, (module, attr) in (attrs or {}).items():
        out[name] = Binding("attr", module, attr, getattr(module, attr))
    return out


def to_param_value(default):
    """dict/attr의 파이썬 기본값 → declare_parameter에 넘길 ROS 파라미터 값.
    None → NONE_SENTINEL(double), tuple → list(array 파라미터)."""
    if default is None:
        return NONE_SENTINEL
    if isinstance(default, tuple):
        return list(default)
    return default


def restore_py_value(binding, value):
    """ROS 파라미터 값 → 기본값과 같은 파이썬 타입으로 복원.
    잘못된 값(배열 길이 불일치 등)은 ValueError."""
    default = binding.default
    if default is None:
        # 미측정/override 슬롯: 0(.0)이면 None, 아니면 숫자 그대로
        return None if float(value) == NONE_SENTINEL else float(value)
    if isinstance(default, tuple):
        if value is None or len(value) != len(default):
            raise ValueError(f"배열 길이가 {len(default)}이어야 함: {value!r}")
        return tuple(type(e)(x) for e, x in zip(default, value))
    if isinstance(default, bool):
        return bool(value)
    if isinstance(default, int):
        return int(value)
    if isinstance(default, float):
        return float(value)
    if isinstance(default, str):
        return str(value)
    raise ValueError(f"지원하지 않는 값 타입: {type(default).__name__}")


def apply_value(binding, value):
    """ROS 파라미터 값을 살아있는 dict(in-place)/모듈 어트리뷰트에 반영한다."""
    py_val = restore_py_value(binding, value)
    if binding.kind == "dict":
        binding.container[binding.key] = py_val
    else:
        setattr(binding.container, binding.key, py_val)
    return py_val


def install(node, dicts=None, attrs=None, on_change=None):
    """노드에 튜닝 파라미터를 선언하고 write-back 콜백을 건다.

    - 선언 기본값 = 현재 dict/어트리뷰트 값 → 파라미터를 안 건드리면 동작 불변.
    - 기동 시 launch/params-file(tuning_params:=)로 들어온 override를 즉시 반영.
    - 이후 ros2 param set은 add_on_set_parameters_callback에서 반영.
    - on_change(param_name): 반영 직후 호출되는 훅 — 캐시 무효화용
      (예: odometry_node의 호모그래피 재계산).

    반환: {param_name: Binding} (dump_tuning 등 진단용).
    """
    from rcl_interfaces.msg import SetParametersResult

    bindings = flatten_bindings(dicts, attrs)
    for name, b in bindings.items():
        node.declare_parameter(name, to_param_value(b.default))
        # declare가 적용한 기동 시점 override(있다면)를 dict에도 반영
        val = node.get_parameter(name).value
        try:
            apply_value(b, val)
        except ValueError as e:
            node.get_logger().warn(f"튜닝 파라미터 {name} 기동값 무시: {e}")

    def _cb(params):
        for p in params:
            b = bindings.get(p.name)
            if b is None:
                continue  # mission/show 등 다른 파라미터는 관여하지 않음
            try:
                py_val = apply_value(b, p.value)
            except (ValueError, TypeError) as e:
                return SetParametersResult(successful=False,
                                           reason=f"{p.name}: {e}")
            node.get_logger().info(f"튜닝 반영: {p.name} = {py_val!r}")
            if on_change is not None:
                on_change(p.name)
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(_cb)
    return bindings
