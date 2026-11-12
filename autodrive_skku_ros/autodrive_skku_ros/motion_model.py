"""지연 보상: 자전거 모델로 과거 경로를 "지금" 시점까지 앞으로 굴린다.

선배(taegyu) auto_drive_py 프로젝트의 planning/motion_model.py 이식 (2026-08-03).
실차에서 캡처→인지→판단까지 중앙값 지연이 0.29초로 측정됐고(원본 프로젝트
실측), 그동안 차는 계속 움직이므로 보정 없는 경로는 이미 지나온 지면을
가리킨다. lane_follow.py의 Pure Pursuit이 이 모듈로 마지막 관측 경로를
"명령된" 속도/조향으로 적분해 지금 차량 좌표계로 옮긴 뒤 lookahead를 고른다.

이 모듈은 외부 의존이 numpy뿐이라 원본을 거의 그대로 가져왔다 — 로직을
바꾸지 않았다. 팀 config.WHEELBASE_M(0.545)을 기본 축거로 쓰도록 기본값만
맞췄다.

**필터는 정보를 만들어내지 못한다.** 인지가 실제로 차선을 놓친 구간에서는
이게 순수 추측항법(dead reckoning)이고 오차가 경과시간에 비례해 커진다.
두 규칙으로 그 오차를 제한한다:

* 예측 구간(prediction horizon)에 상한이 있다 — 넘으면 경로를 버리고, 이건
  추종기 쪽에서 "경로 없음"으로 이어져 fail-closed로 멈춘다.
* 적분에 쓰는 속도/조향은 **명령값**이다(실측 아님) — 이 차는 조향 POT
  캘리브레이션이 아직 미완이라(README 15절) 예측 오차가 누적되고, 그래서
  상한을 짧게 유지해야 한다.

전 구간에서 쓰는 좌표계: 후륜축 지면 좌표계, 미터/라디안, x=전방, y=**좌측**,
psi=반시계 양수. 따라서 양의 조향은 좌회전이고, 양의 횡방향 이동은 좌측이다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


# 이 크기 미만의 조향[rad]은 직진으로 취급한다. 원호 해가 tan(delta)로
# 나누므로 delta가 0에 가까워지면 발산한다.
_STRAIGHT_STEERING_RAD = 1e-6


def _finite(value) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


@dataclass(frozen=True)
class Displacement:
    """한 구간 동안의 후륜축 기준 이동량.

    ``forward_m``은 +x(전방), ``lateral_m``은 **좌측**이 양수,
    ``heading_rad``는 반시계 방향이 양수. 단위는 미터/라디안.
    """

    forward_m: float
    lateral_m: float
    heading_rad: float

    @property
    def distance_m(self) -> float:
        return math.hypot(self.forward_m, self.lateral_m)

    def to_dict(self) -> dict:
        return {
            "forward_m": round(self.forward_m, 5),
            "lateral_m": round(self.lateral_m, 5),
            "heading_deg": round(math.degrees(self.heading_rad), 4),
            "distance_m": round(self.distance_m, 5),
        }


def integrate_bicycle(
    speed_mps: float,
    steering_rad: float,
    wheelbase_m: float,
    duration_s: float,
) -> Displacement:
    """속도·조향이 일정하다고 보고 구간 동안의 후륜축 이동을 정확히 계산.

    속도/조향이 구간 내내 일정하면 후륜축은 정확한 원호를 그리므로, 수치
    적분 대신 닫힌 형태를 쓴다 — 더 정확하고 스텝 수를 튜닝할 필요가 없다.

        R = L / tan(delta),  dpsi = v t / R
        dx = R sin(dpsi),    dy = R (1 - cos(dpsi))

    조향이 0에 가까우면 직진 극한으로 넘어간다. 양의 ``steering_rad``는
    좌회전이며 양의 ``lateral_m``/``heading``을 만든다.

    입력이 유한하지 않거나, 축거가 양수가 아니거나, duration이 음수이면
    ValueError — 호출부가 조용히 0으로 대체하면 안 된다.
    """

    if not all(
        _finite(value)
        for value in (speed_mps, steering_rad, wheelbase_m, duration_s)
    ):
        raise ValueError("bicycle integration needs finite inputs")
    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive")
    if duration_s < 0.0:
        raise ValueError("duration_s must be non-negative")

    travel = float(speed_mps) * float(duration_s)
    if abs(steering_rad) < _STRAIGHT_STEERING_RAD:
        return Displacement(forward_m=travel, lateral_m=0.0, heading_rad=0.0)

    radius = wheelbase_m / math.tan(steering_rad)
    heading = travel / radius
    return Displacement(
        forward_m=radius * math.sin(heading),
        lateral_m=radius * (1.0 - math.cos(heading)),
        heading_rad=heading,
    )


def shift_points_to_current_frame(points_xy, displacement: Displacement):
    """경로점을 차량이 **지금** 있는 좌표계로 다시 표현한다.

    차량이 ``displacement``만큼 움직였으므로, 옛 좌표계의 점 p는 지금
    좌표계에서 ``R(-dpsi) (p - d)``가 된다: 이동한 거리만큼 가까워지고
    헤딩 변화만큼 반대로 회전한다. 입출력 모두 (N, 2) 배열, 후륜축 기준
    미터, x=전방/y=좌측.
    """

    if not isinstance(displacement, Displacement):
        raise TypeError("displacement must be Displacement")
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    if points.size == 0:
        return points
    shifted = points - np.array(
        [displacement.forward_m, displacement.lateral_m], dtype=np.float64
    )
    angle = -displacement.heading_rad
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    rotation = np.array([[cos_a, -sin_a], [sin_a, cos_a]], dtype=np.float64)
    return shifted @ rotation.T


@dataclass(frozen=True)
class LatencyCompensationConfig:
    """지연 보상 한계값. 기본 축거는 팀 config.WHEELBASE_M(0.545) 실측값."""

    wheelbase_m: float = 0.545
    # 들고 있는 경로를 얼마나 앞으로 예측해도 되는지의 상한[s]. 넘으면
    # 경로를 버린다 — 추종기 쪽에서는 "경로 소실"과 동일하게 처리된다.
    #
    # 속도/조향은 **명령값**이라(실측 아님) 오차가 누적된다. 이 상한의
    # 목적은 추측항법이 "인지가 실제로 차선을 놓친 구간"을 그럴듯하게
    # 감춰버리는 것을 막는 데 있다.
    maximum_prediction_s: float = 0.60
    # 이보다 오래된 예측은 여전히 쓰지만 degraded로 표시한다[s]. 신뢰도
    # 하향일 뿐 차를 세우지는 않는다.
    degraded_prediction_s: float = 0.35

    def __post_init__(self) -> None:
        values = (
            self.wheelbase_m,
            self.maximum_prediction_s,
            self.degraded_prediction_s,
        )
        if not all(_finite(value) for value in values):
            raise ValueError("latency compensation parameters must be finite")
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if self.maximum_prediction_s <= 0.0:
            raise ValueError("maximum_prediction_s must be positive")
        if not 0.0 < self.degraded_prediction_s <= self.maximum_prediction_s:
            raise ValueError(
                "degraded_prediction_s must be positive and not exceed the maximum"
            )


@dataclass(frozen=True)
class PredictedPath:
    """지연 보상된 경로 하나, 또는 없을 경우 그 이유."""

    valid: bool
    reason: str
    points: tuple[tuple[float, float], ...] = ()
    prediction_s: float = 0.0
    displacement: Displacement | None = None
    degraded: bool = False

    def to_dict(self) -> dict:
        return {
            "valid": bool(self.valid),
            "reason": self.reason,
            "prediction_s": round(float(self.prediction_s), 4),
            "degraded": bool(self.degraded),
            "point_count": len(self.points),
            "displacement": (
                None if self.displacement is None else self.displacement.to_dict()
            ),
        }


class LatencyCompensator:
    """마지막 관측 경로를 들고 있다가 지금 시점까지 앞으로 굴린다.

    인지 갱신 사이의 공백을 메우는 역할이다. 새로 관측된 경로가 들어오면
    기존 경로를 통째로 교체하고(그 순간 누적 예측오차가 0이 됨), 새 경로가
    없으면 마지막으로 명령한 속도/조향으로 계속 앞으로 민다.
    ``maximum_prediction_s``를 넘으면 무효 결과를 반환한다.
    """

    def __init__(self, config: LatencyCompensationConfig | None = None):
        self.config = config or LatencyCompensationConfig()
        self._points: np.ndarray | None = None
        self._stamp_s: float | None = None

    def reset(self) -> None:
        self._points = None
        self._stamp_s = None

    def observe(self, points_xy, stamp_s: float) -> None:
        """새로 관측된 경로로 들고 있던 경로를 교체한다.

        ``points_xy``는 후륜축 기준 미터, ``stamp_s``는 그 인지 프레임의
        벽시계 시각. 점이 2개 미만이거나 유한하지 않은 좌표가 있으면
        저장하지 않고 비운다 — 잘못된 경로보다 경로 없음이 더 안전하다.
        """

        if not _finite(stamp_s) or float(stamp_s) < 0.0:
            raise ValueError("stamp_s must be finite and non-negative")
        points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        if len(points) < 2 or not np.isfinite(points).all():
            self.reset()
            return
        self._points = points
        self._stamp_s = float(stamp_s)

    def predict(
        self, now_s: float, speed_mps: float, steering_rad: float
    ) -> PredictedPath:
        """들고 있는 경로를 지금 차량 좌표계로 표현해 반환한다.

        ``speed_mps``/``steering_rad``는 차량에 마지막으로 **명령한** 값
        (좌측이 양의 조향)이며, ``stamp_s`` 이후 경과한 움직임을 적분하는
        데 쓴다. 다음 중 하나라도 해당하면 무효 — 아직 경로가 관측된 적
        없음, 로컬 시계가 유한하지 않음, 들고 있는 stamp가 미래를 가리킴,
        ``maximum_prediction_s``를 초과함, 예측이 경로를 후륜축 뒤로
        밀어냄. 호출부는 무효를 "경로 없음"과 동일하게 다뤄야 한다.
        """

        if self._points is None or self._stamp_s is None:
            return PredictedPath(valid=False, reason="no_observed_path")
        if not _finite(now_s):
            return PredictedPath(valid=False, reason="invalid_local_clock")
        elapsed = float(now_s) - self._stamp_s
        if elapsed < 0.0:
            return PredictedPath(valid=False, reason="path_stamp_in_future")
        if elapsed > self.config.maximum_prediction_s:
            return PredictedPath(
                valid=False,
                reason="prediction_horizon_exceeded",
                prediction_s=elapsed,
            )

        displacement = integrate_bicycle(
            speed_mps, steering_rad, self.config.wheelbase_m, elapsed
        )
        shifted = shift_points_to_current_frame(self._points, displacement)
        # 차가 이미 지나친 점(x <= 0)은 버린다 — y(x)로 표현할 수 없고
        # 전방을 보는 추종기에는 쓸모가 없다.
        ahead = shifted[shifted[:, 0] > 0.0]
        if len(ahead) < 2:
            return PredictedPath(
                valid=False,
                reason="predicted_path_is_behind_the_vehicle",
                prediction_s=elapsed,
                displacement=displacement,
            )
        return PredictedPath(
            valid=True,
            reason="",
            points=tuple((float(x), float(y)) for x, y in ahead),
            prediction_s=elapsed,
            displacement=displacement,
            degraded=elapsed > self.config.degraded_prediction_s,
        )


# ============================ 셀프테스트 ============================
# 다른 순수 로직 모듈(filters.py 등)과 동일 패턴: check(name, ok) 누적,
# tools/run_tests.py --motion-model로 등록해 돌린다.

def selftest():
    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    # ---- integrate_bicycle ----
    straight = integrate_bicycle(1.0, 0.0, 0.545, 2.0)
    check("직진(steering=0): forward=speed*t, lateral=0",
          abs(straight.forward_m - 2.0) < 1e-9 and straight.lateral_m == 0.0)

    left_turn = integrate_bicycle(1.0, math.radians(10), 0.545, 1.0)
    check("좌회전(steering>0) -> lateral_m 양수(좌측)", left_turn.lateral_m > 0.0)
    check("좌회전 -> heading_rad 양수(반시계)", left_turn.heading_rad > 0.0)

    right_turn = integrate_bicycle(1.0, -math.radians(10), 0.545, 1.0)
    check("우회전(steering<0) -> lateral_m 음수(우측)", right_turn.lateral_m < 0.0)

    try:
        integrate_bicycle(1.0, 0.0, -0.1, 1.0)
        check("축거<=0 -> ValueError", False)
    except ValueError:
        check("축거<=0 -> ValueError", True)

    try:
        integrate_bicycle(1.0, 0.0, 0.5, -1.0)
        check("duration<0 -> ValueError", False)
    except ValueError:
        check("duration<0 -> ValueError", True)

    # ---- shift_points_to_current_frame ----
    disp = Displacement(forward_m=1.0, lateral_m=0.0, heading_rad=0.0)
    shifted = shift_points_to_current_frame([(2.0, 0.0)], disp)
    check("전진만 한 경우 -> 점이 그만큼 가까워짐(2.0 -> 1.0)",
          abs(shifted[0, 0] - 1.0) < 1e-9 and abs(shifted[0, 1]) < 1e-9)

    # ---- LatencyCompensator ----
    comp = LatencyCompensator(LatencyCompensationConfig(
        wheelbase_m=0.545, maximum_prediction_s=0.6, degraded_prediction_s=0.35))
    result0 = comp.predict(now_s=1.0, speed_mps=0.1, steering_rad=0.0)
    check("관측 전 predict() -> invalid(no_observed_path)",
          not result0.valid and result0.reason == "no_observed_path")

    comp.observe([(0.5, 0.0), (1.0, 0.0), (1.5, 0.0)], stamp_s=1.0)
    fresh = comp.predict(now_s=1.0, speed_mps=0.1, steering_rad=0.0)
    check("관측 직후(elapsed=0) -> valid, 원본 점 그대로",
          fresh.valid and abs(fresh.points[0][0] - 0.5) < 1e-9)

    later = comp.predict(now_s=1.2, speed_mps=0.1, steering_rad=0.0)
    check("0.2초 경과, 직진 0.1m/s -> 점들이 0.02m 가까워짐",
          later.valid and abs(later.points[0][0] - 0.48) < 1e-6)
    check("degraded_prediction_s(0.35) 미만이면 degraded=False",
          not later.degraded)

    stale = comp.predict(now_s=1.5, speed_mps=0.1, steering_rad=0.0)
    check("degraded_prediction_s(0.35) 넘으면 degraded=True (여전히 valid)",
          stale.valid and stale.degraded)

    expired = comp.predict(now_s=1.0 + 0.61, speed_mps=0.1, steering_rad=0.0)
    check("maximum_prediction_s(0.6) 넘으면 invalid(prediction_horizon_exceeded)",
          not expired.valid and expired.reason == "prediction_horizon_exceeded")

    comp.observe([(0.1, 0.0), (0.2, 0.0)], stamp_s=2.0)
    behind = comp.predict(now_s=2.3, speed_mps=5.0, steering_rad=0.0)
    check("빠른 속도로 예측한 지점이 전부 차 뒤로 밀리면 -> invalid",
          not behind.valid and behind.reason == "predicted_path_is_behind_the_vehicle")

    comp.reset()
    after_reset = comp.predict(now_s=3.0, speed_mps=0.1, steering_rad=0.0)
    check("reset() 후 predict() -> 다시 no_observed_path",
          not after_reset.valid and after_reset.reason == "no_observed_path")

    malformed = LatencyCompensator()
    malformed.observe([(1.0, float("nan"))], stamp_s=1.0)
    check("비정상 좌표(NaN) observe -> 저장 거부(경로 없음 상태 유지)",
          malformed._points is None)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    print("motion_model.py는 라이브러리 모듈입니다 — "
          "python3 -m autodrive_skku_ros.motion_model --selftest")
