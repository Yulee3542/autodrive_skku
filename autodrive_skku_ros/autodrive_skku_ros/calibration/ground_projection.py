"""카메라-지면 투영: 이미지 픽셀 -> 차량 기준 미터.

선배(taegyu) auto_drive_py 프로젝트의 perception/ground_projection.py 이식
(2026-08-03). 로직은 원본과 동일 — 검증된 최소자승 호모그래피 피팅과
fail-closed 유효범위 검사를 그대로 가져왔다. lane_follow.py의 LANE_POI가
지금 쓰는 `_to_vehicle_frame()`의 단일 스칼라 px_per_m 변환은 BEV 워프가
축별로 다른 배율(세로 3.2배 vs 가로 0.5배, LANE_POI 주석 참고)로 왜곡돼
있어 원리적으로 부정확하다 — 이 모듈로 계산한 진짜 호모그래피가 그 자리를
대신한다(캘리브레이션 파일이 없으면 기존 스칼라 방식으로 폴백, 회귀 없음).

**좌표계 규약**: 후륜축을 지면에 투영한 점이 원점, x=전방, y=**좌측**,
단위는 미터(ISO 8855 / REP-103). 이미지의 u축(가로)은 오른쪽으로 증가하므로
y는 u축과 부호가 반대다.

이 변환은 **평평한 지면**을 가정하므로 세 규칙을 강제한다.
* 캘리브레이션이 실제로 덮은 지면 범위 밖으로 절대 외삽하지 않는다.
* 소실선(horizon) 너머의 픽셀은 "차량 뒤"처럼 보이는 값을 낼 수 있어
  동차좌표 부호로 걸러낸다.
* 캘리브레이션이 없거나 기준 해상도가 안 맞으면 아무 것도 내지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


COORDINATE_SPACE = "vehicle_meters"
GROUND_FRAME = "rear_axle_ground"
SCHEMA_VERSION = 1

# 캘리브레이션된 카메라가 보는 방향. 전/후방 모두 위의 단일 차량 좌표계를
# 공유한다 — x는 후륜축 기준 전방, y는 좌측. 후방 카메라는 그저 x가 음수인
# 지면을 캘리브레이션할 뿐, 별도의 거울 좌표계가 아니다. 팀 차량은 전/후방
# 카메라가 둘 다 있으므로(config.FRONT_CAMERA/REAR_CAMERA) 그대로 유용하다.
FRONT_MOUNTING = "front"
REAR_MOUNTING = "rear"
MOUNTINGS = (FRONT_MOUNTING, REAR_MOUNTING)

# 동차좌표 세 번째 성분이 이 값보다 작으면 소실선에 너무 가까워 못 믿는다 —
# 투영된 미터값이 무한대로 발산하기 직전이라는 뜻.
_MINIMUM_HOMOGENEOUS_W = 1e-9


def _finite(value) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


@dataclass(frozen=True)
class CameraIntrinsics:
    """한 기준 해상도에서의 핀홀 내부 파라미터 + plumb_bob 왜곡 계수.

    모든 값은 기준 이미지(``width`` x ``height``) 픽셀 기준이다. 다른
    크기의 프레임에서 잰 점은 먼저 :meth:`scale_to_reference`를 거쳐야 한다.

    BEV처럼 이미 원근변환으로 워프된 합성 이미지에는 렌즈 왜곡 개념이 없다
    — 이 경우 distortion=(0,0,0,0,0)인 "항등" 내부 파라미터를 만들어 쓰면
    undistort()가 사실상 그대로 통과시킨다(수학적으로 왜곡계수가 0이면
    fx/fy/cx/cy 값과 무관하게 입력=출력).
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not all(_finite(value) for value in (self.fx, self.fy, self.cx, self.cy)):
            raise ValueError("intrinsics must be finite numbers")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("focal lengths must be positive")
        for name, value in (("width", self.width), ("height", self.height)):
            if isinstance(value, bool) or not isinstance(value, int) or value < 16:
                raise ValueError(f"{name} must be an integer of at least 16")
        if not 0.0 <= self.cx < self.width or not 0.0 <= self.cy < self.height:
            raise ValueError("principal point must lie inside the reference image")
        coefficients = tuple(float(value) for value in self.distortion)
        if len(coefficients) not in (4, 5, 8, 12, 14):
            raise ValueError("plumb_bob distortion needs 4, 5, 8, 12, or 14 terms")
        if not all(math.isfinite(value) for value in coefficients):
            raise ValueError("distortion coefficients must be finite")
        object.__setattr__(self, "distortion", coefficients)

    @classmethod
    def identity(cls, width: int, height: int) -> "CameraIntrinsics":
        """왜곡 없는 항등 내부 파라미터 — BEV처럼 이미 워프된 합성 이미지용.

        fx/fy/cx/cy는 유효성 검사만 통과하면 값 자체는 결과에 영향을 주지
        않는다(distortion이 전부 0이라 undistort가 항등 사상이므로)."""
        return cls(
            fx=float(width), fy=float(width), cx=width / 2.0, cy=height / 2.0,
            width=int(width), height=int(height),
        )

    def camera_matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def distortion_vector(self) -> np.ndarray:
        return np.asarray(self.distortion, dtype=np.float64)

    def scale_to_reference(self, points_uv, image_size) -> np.ndarray:
        """리사이즈된 이미지에서 잰 점을 기준 해상도 픽셀로 되돌린다.

        ``points_uv``는 ``image_size`` = (width, height) 이미지의 픽셀이고,
        반환은 기준 이미지 픽셀이다. ``image_size=None``이면 점이 이미
        기준 스케일이라고 간주한다.
        """

        points = np.asarray(points_uv, dtype=np.float64).reshape(-1, 2)
        if image_size is None:
            return points
        width, height = image_size
        if (
            isinstance(width, bool)
            or isinstance(height, bool)
            or not isinstance(width, (int, np.integer))
            or not isinstance(height, (int, np.integer))
            or width < 16
            or height < 16
        ):
            raise ValueError("image_size must be two integers of at least 16")
        if (int(width), int(height)) == (self.width, self.height):
            return points
        scale_x = (self.width - 1) / (int(width) - 1)
        scale_y = (self.height - 1) / (int(height) - 1)
        return np.column_stack(
            (points[:, 0] * scale_x, points[:, 1] * scale_y)
        )

    def undistort(self, points_uv, image_size=None) -> np.ndarray:
        """렌즈 왜곡을 제거한 기준 해상도 픽셀을 반환. 호모그래피가 기대하는
        공간이라 모든 투영은 여기서 시작한다."""

        points = self.scale_to_reference(points_uv, image_size)
        if points.size == 0:
            return points.reshape(-1, 2)
        matrix = self.camera_matrix()
        undistorted = cv2.undistortPoints(
            points.reshape(-1, 1, 2),
            matrix,
            self.distortion_vector(),
            P=matrix,
        )
        return np.asarray(undistorted, dtype=np.float64).reshape(-1, 2)

    def distort(self, points_uv) -> np.ndarray:
        """:meth:`undistort`의 역 — 원본(왜곡된) 이미지 위에 오버레이를 그릴 때만 쓴다."""

        points = np.asarray(points_uv, dtype=np.float64).reshape(-1, 2)
        if points.size == 0:
            return points
        normalized_x = (points[:, 0] - self.cx) / self.fx
        normalized_y = (points[:, 1] - self.cy) / self.fy
        rays = np.column_stack(
            (normalized_x, normalized_y, np.ones(len(points), dtype=np.float64))
        )
        projected, _ = cv2.projectPoints(
            rays,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            self.camera_matrix(),
            self.distortion_vector(),
        )
        return np.asarray(projected, dtype=np.float64).reshape(-1, 2)

    def to_dict(self) -> dict:
        return {
            "fx": float(self.fx),
            "fy": float(self.fy),
            "cx": float(self.cx),
            "cy": float(self.cy),
            "width": int(self.width),
            "height": int(self.height),
            "distortion": [float(value) for value in self.distortion],
        }

    @classmethod
    def from_dict(cls, payload) -> "CameraIntrinsics":
        if not isinstance(payload, dict):
            raise ValueError("intrinsics payload must be a JSON object")
        return cls(
            fx=float(payload["fx"]),
            fy=float(payload["fy"]),
            cx=float(payload["cx"]),
            cy=float(payload["cy"]),
            width=int(payload["width"]),
            height=int(payload["height"]),
            distortion=tuple(float(v) for v in payload.get("distortion", ())),
        )


@dataclass(frozen=True)
class GroundProjection:
    """검증된 이미지픽셀->지면미터 평면 호모그래피 하나.

    ``homography``는 row-major 3x3, undistort된 기준해상도 픽셀을 차량
    기준 미터(x=전방, y=좌측, 후륜축 지면원점)로 매핑한다. ``valid_x_m``/
    ``valid_y_m``은 캘리브레이션 점들이 실제로 덮은 미터 범위 — 그 밖은
    외삽하지 않는다. ``revision``은 이 캘리브레이션 실행을 식별하는 이름.
    """

    homography: tuple[float, ...]
    intrinsics: CameraIntrinsics
    revision: str
    valid_x_m: tuple[float, float]
    valid_y_m: tuple[float, float]
    residual_m: float = 0.0
    maximum_residual_m: float = 0.0
    point_count: int = 0
    mounting: str = FRONT_MOUNTING

    def __post_init__(self) -> None:
        values = tuple(float(value) for value in self.homography)
        if len(values) != 9:
            raise ValueError("homography must have nine row-major elements")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("homography elements must be finite")
        matrix = np.asarray(values, dtype=np.float64).reshape(3, 3)
        if abs(float(np.linalg.det(matrix))) < 1e-12:
            raise ValueError("homography is singular")
        object.__setattr__(self, "homography", values)

        if not isinstance(self.intrinsics, CameraIntrinsics):
            raise TypeError("intrinsics must be CameraIntrinsics")
        if not str(self.revision).strip():
            raise ValueError("calibration revision must be a non-empty string")
        object.__setattr__(self, "revision", str(self.revision).strip())

        for name, bounds in (
            ("valid_x_m", self.valid_x_m),
            ("valid_y_m", self.valid_y_m),
        ):
            if not isinstance(bounds, (tuple, list)) or len(bounds) != 2:
                raise ValueError(f"{name} must be a (minimum, maximum) pair")
            low, high = (float(bounds[0]), float(bounds[1]))
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                raise ValueError(f"{name} must be a finite increasing range")
            object.__setattr__(self, name, (low, high))
        mounting = str(self.mounting).strip().lower()
        if mounting not in MOUNTINGS:
            raise ValueError(f"mounting must be one of {MOUNTINGS}")
        object.__setattr__(self, "mounting", mounting)

        if mounting == FRONT_MOUNTING and self.valid_x_m[0] <= 0.0:
            raise ValueError("valid_x_m must stay ahead of the rear axle origin")
        if mounting == REAR_MOUNTING and self.valid_x_m[1] >= 0.0:
            raise ValueError("valid_x_m must stay behind the rear axle origin")

        for name in ("residual_m", "maximum_residual_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
            object.__setattr__(self, name, value)
        if isinstance(self.point_count, bool) or not isinstance(self.point_count, int):
            raise ValueError("point_count must be an integer")
        if self.point_count < 0:
            raise ValueError("point_count must be non-negative")

    @property
    def matrix(self) -> np.ndarray:
        return np.asarray(self.homography, dtype=np.float64).reshape(3, 3)

    @classmethod
    def fit(
        cls,
        image_points,
        ground_points,
        intrinsics,
        revision,
        image_size=None,
        margin_m=0.0,
        mounting=FRONT_MOUNTING,
    ) -> tuple["GroundProjection", dict]:
        """실측 대응점으로부터 최소자승 피팅 + 사람이 읽을 잔차 리포트.

        ``image_points``는 ``image_size`` 이미지 위 픽셀(또는 None이면
        기준해상도 픽셀), ``ground_points``는 차량 기준 미터(x=전방,
        y=좌측). RANSAC이 아니라 전 점에 대한 최소자승이다 — 대응점이
        수십 개뿐이라 이상치를 조용히 버리는 것보다 잔차를 전부 보여주고
        사람이 어긋난 보드/거리 오타를 직접 찾게 하는 편이 안전하다.

        유효범위는 관측점의 바운딩박스로 자동 설정된다 — 캘리브레이션이
        덮지 않은 지면으로 외삽하는 것을 막는다.
        """

        if not isinstance(intrinsics, CameraIntrinsics):
            raise TypeError("intrinsics must be CameraIntrinsics")
        ground = np.asarray(ground_points, dtype=np.float64).reshape(-1, 2)
        undistorted = intrinsics.undistort(image_points, image_size)
        if len(ground) != len(undistorted):
            raise ValueError("image and ground point counts differ")
        if len(ground) < 4:
            raise ValueError("a homography needs at least four correspondences")
        if not np.isfinite(ground).all() or not np.isfinite(undistorted).all():
            raise ValueError("correspondences must be finite")
        margin = float(margin_m)
        if not math.isfinite(margin) or margin < 0.0:
            raise ValueError("margin_m must be finite and non-negative")

        matrix, _ = cv2.findHomography(undistorted, ground, 0)
        if matrix is None:
            raise ValueError(
                "homography fit failed; the points are probably collinear "
                "or the correspondences are mismatched"
            )
        matrix = np.asarray(matrix, dtype=np.float64)

        homogeneous = np.column_stack(
            (undistorted, np.ones(len(undistorted), dtype=np.float64))
        )
        w = homogeneous @ matrix[2]
        if np.all(w < 0.0):
            matrix = -matrix
            w = -w
        elif not np.all(w > 0.0):
            raise ValueError(
                "calibration points straddle the horizon; check that every "
                "point really lies on the ground plane ahead of the camera"
            )

        projected = homogeneous @ matrix.T
        recovered = projected[:, :2] / projected[:, 2:3]
        errors = np.linalg.norm(recovered - ground, axis=1)

        edge = float(errors.max())
        low_x, low_y = ground.min(axis=0)
        high_x, high_y = ground.max(axis=0)
        valid_x = (float(low_x) - margin - edge, float(high_x) + margin + edge)
        valid_y = (float(low_y) - margin - edge, float(high_y) + margin + edge)
        if str(mounting).strip().lower() == REAR_MOUNTING:
            if valid_x[1] >= 0.0:
                raise ValueError(
                    "calibration points reach the rear axle origin; keep every "
                    "point behind the vehicle"
                )
        elif valid_x[0] <= 0.0:
            raise ValueError(
                "calibration points reach the rear axle origin; keep every "
                "point ahead of the vehicle"
            )

        projection = cls(
            homography=tuple(matrix.reshape(-1)),
            intrinsics=intrinsics,
            revision=str(revision),
            valid_x_m=valid_x,
            valid_y_m=valid_y,
            residual_m=float(np.median(errors)),
            maximum_residual_m=float(errors.max()),
            point_count=int(len(ground)),
            mounting=mounting,
        )
        report = {
            "point_count": int(len(ground)),
            "median_residual_m": float(np.median(errors)),
            "rms_residual_m": float(np.sqrt(np.mean(errors**2))),
            "maximum_residual_m": float(errors.max()),
            "worst_index": int(np.argmax(errors)),
            "residuals_m": [float(value) for value in errors],
            "ground_points": [[float(x), float(y)] for x, y in ground],
            "valid_x_m": list(valid_x),
            "valid_y_m": list(valid_y),
        }
        return projection, report

    def image_to_ground(self, points_uv, image_size=None):
        """이미지 점을 지면 미터로 투영, 점마다 유효 마스크와 함께 반환.

        ``points_uv``는 ``image_size`` = (width, height) 이미지의 픽셀.
        반환은 ``(ground, valid)`` — ``ground``는 Nx2 차량기준 미터(0열=
        x전방, 1열=y좌측, 원점은 후륜축 지면점), ``valid``는 점별 불리언.

        ``valid``가 False인 행은 의미 없는 좌표라 호출부가 반드시 마스크를
        확인해야 한다. 탈락한 행은 0이 아니라 NaN으로 채운다 — 실수로
        그대로 쓰면 후륜축에 조용히 놓이는 대신 요란하게 티가 나도록.
        """

        undistorted = self.intrinsics.undistort(points_uv, image_size)
        count = len(undistorted)
        if count == 0:
            return np.zeros((0, 2), dtype=np.float64), np.zeros(0, dtype=bool)

        homogeneous = np.column_stack(
            (undistorted, np.ones(count, dtype=np.float64))
        )
        mapped = homogeneous @ self.matrix.T
        w = mapped[:, 2]
        finite_w = np.isfinite(w) & (w > _MINIMUM_HOMOGENEOUS_W)
        ground = np.full((count, 2), np.nan, dtype=np.float64)
        with np.errstate(divide="ignore", invalid="ignore"):
            ground[finite_w, 0] = mapped[finite_w, 0] / w[finite_w]
            ground[finite_w, 1] = mapped[finite_w, 1] / w[finite_w]

        valid = finite_w & np.isfinite(ground).all(axis=1)
        valid &= (ground[:, 0] >= self.valid_x_m[0]) & (
            ground[:, 0] <= self.valid_x_m[1]
        )
        valid &= (ground[:, 1] >= self.valid_y_m[0]) & (
            ground[:, 1] <= self.valid_y_m[1]
        )
        ground[~valid] = np.nan
        return ground, valid

    def ground_to_image(self, points_xy, image_size=None) -> np.ndarray:
        """역투영 — 원본 프레임에 미터 오버레이를 그릴 때만 쓴다.

        :meth:`image_to_ground`와 달리 ``valid_x_m``/``valid_y_m``을
        강제하지 않는다: 이건 그리기용 보조 함수이고, 캘리브레이션 범위
        밖을 그리길 거부하면 오퍼레이터가 정확히 보고 싶어하는 오버슈트를
        가려버린다. 이 결과로 미터값을 정당화하지 말 것.
        """

        points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        if points.size == 0:
            return np.zeros((0, 2), dtype=np.float64)
        homogeneous = np.column_stack(
            (points, np.ones(len(points), dtype=np.float64))
        )
        mapped = homogeneous @ np.linalg.inv(self.matrix).T
        w = mapped[:, 2]
        undistorted = np.full((len(points), 2), np.nan, dtype=np.float64)
        usable = np.isfinite(w) & (np.abs(w) > _MINIMUM_HOMOGENEOUS_W)
        with np.errstate(divide="ignore", invalid="ignore"):
            undistorted[usable, 0] = mapped[usable, 0] / w[usable]
            undistorted[usable, 1] = mapped[usable, 1] / w[usable]
        pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
        if usable.any():
            pixels[usable] = self.intrinsics.distort(undistorted[usable])
        if image_size is None:
            return pixels
        width, height = (int(image_size[0]), int(image_size[1]))
        if (width, height) == (self.intrinsics.width, self.intrinsics.height):
            return pixels
        pixels[:, 0] *= (width - 1) / (self.intrinsics.width - 1)
        pixels[:, 1] *= (height - 1) / (self.intrinsics.height - 1)
        return pixels

    def to_dict(self) -> dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "coordinate_space": COORDINATE_SPACE,
            "frame": GROUND_FRAME,
            "revision": self.revision,
            "homography": [float(value) for value in self.homography],
            "intrinsics": self.intrinsics.to_dict(),
            "valid_x_m": list(self.valid_x_m),
            "valid_y_m": list(self.valid_y_m),
            "residual_m": float(self.residual_m),
            "maximum_residual_m": float(self.maximum_residual_m),
            "point_count": int(self.point_count),
            "mounting": str(self.mounting),
        }

    @classmethod
    def from_dict(cls, payload) -> "GroundProjection":
        """저장된 캘리브레이션을 로드하되, 다른 좌표계/스키마는 거부한다.

        frame/coordinate-space 태그를 가정하지 않고 실제로 검사한다 —
        다른 규약(이미지 픽셀, 전륜축 원점, y-우측)으로 쓰인 파일이
        조용히 로드돼 차량을 좌우 반전되거나 오프셋된 채로 조향하게
        만드는 것을 막는다.
        """

        if not isinstance(payload, dict):
            raise ValueError("ground projection payload must be a JSON object")
        if payload.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("unsupported ground projection schema_version")
        if payload.get("coordinate_space") != COORDINATE_SPACE:
            raise ValueError("ground projection must use vehicle_meters")
        if payload.get("frame") != GROUND_FRAME:
            raise ValueError("ground projection must use the rear-axle ground frame")
        return cls(
            homography=tuple(float(value) for value in payload["homography"]),
            intrinsics=CameraIntrinsics.from_dict(payload["intrinsics"]),
            revision=str(payload["revision"]),
            valid_x_m=tuple(float(v) for v in payload["valid_x_m"]),
            valid_y_m=tuple(float(v) for v in payload["valid_y_m"]),
            residual_m=float(payload.get("residual_m", 0.0)),
            maximum_residual_m=float(payload.get("maximum_residual_m", 0.0)),
            point_count=int(payload.get("point_count", 0)),
            mounting=str(payload.get("mounting", FRONT_MOUNTING)),
        )


# ============================ 셀프테스트 ============================

def selftest():
    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    intr = CameraIntrinsics.identity(320, 240)
    check("identity 내부파라미터: undistort는 입력을 그대로 반환",
          np.allclose(intr.undistort([(100.0, 50.0), (10.0, 200.0)]),
                      [(100.0, 50.0), (10.0, 200.0)], atol=1e-6))

    # 4x4 격자, 정면을 보는 카메라를 흉내: 이미지 픽셀이 지면 미터에 선형
    # 대응하는 순수 스케일 관계(u -> y, v -> x, 위로 갈수록 멀어짐)로 합성.
    image_pts, ground_pts = [], []
    for row in range(4):        # row 0 = 가장 가까움(x 작음)
        for col in range(4):
            u = 40.0 + col * 60.0   # 픽셀 u
            v = 200.0 - row * 40.0  # 픽셀 v (위로 갈수록 감소 = 멀어짐)
            x = 1.0 + row * 0.3     # 전방 거리(m), 가까운 행일수록 작음
            y = (1.5 - col) * 0.25  # 좌측이 양수
            image_pts.append((u, v))
            ground_pts.append((x, y))

    projection, report = GroundProjection.fit(
        image_pts, ground_pts, intr, revision="selftest-2026-08-03")
    check("fit() 성공, revision 보존", projection.revision == "selftest-2026-08-03")
    check("fit() 잔차가 거의 0 (합성 데이터라 정확히 선형 대응)",
          report["median_residual_m"] < 1e-6)
    check("point_count == 16", projection.point_count == 16)

    ground_back, valid = projection.image_to_ground(image_pts)
    check("image_to_ground 역산 -> 원래 지면 좌표와 일치",
          np.allclose(ground_back, ground_pts, atol=1e-6))
    check("캘리브레이션 점 전부 valid_x/y_m 범위 안 -> valid=True 전부",
          bool(valid.all()))

    outside = projection.image_to_ground([(40.0 - 500.0, 200.0)])
    check("유효범위 밖(외삽 지점) -> valid=False",
          not bool(outside[1][0]))

    pixels_back = projection.ground_to_image(ground_pts)
    check("ground_to_image가 image_to_ground의 역변환과 왕복 일치",
          np.allclose(pixels_back, image_pts, atol=1e-3))

    payload = projection.to_dict()
    restored = GroundProjection.from_dict(payload)
    check("to_dict/from_dict 왕복 -> 동일 호모그래피",
          np.allclose(restored.matrix, projection.matrix))

    bad_payload = dict(payload)
    bad_payload["frame"] = "front_axle_ground"
    try:
        GroundProjection.from_dict(bad_payload)
        check("frame 태그 불일치 -> ValueError (잘못된 좌표계 로드 방지)", False)
    except ValueError:
        check("frame 태그 불일치 -> ValueError (잘못된 좌표계 로드 방지)", True)

    try:
        GroundProjection.fit(image_pts[:3], ground_pts[:3], intr, revision="too-few")
        check("대응점 4개 미만 -> ValueError", False)
    except ValueError:
        check("대응점 4개 미만 -> ValueError", True)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    print("ground_projection.py는 라이브러리 모듈입니다 — "
          "python3 -m autodrive_skku_ros.calibration.ground_projection --selftest")
