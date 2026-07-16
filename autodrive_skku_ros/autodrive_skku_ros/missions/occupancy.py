"""T주차용 고정 크기 로컬 점유 격자 — SLAM이 아니다.

오도메트리(/car/pose)가 신뢰 가능할 때, 라이다 스캔을 odom 좌표로 정합해
히트카운트 격자에 누적한다. 순간 스캔 한 장 대신 누적 맵 위에서 슬롯(주차
차량 사이 갭)을 찾으면 한두 프레임의 노이즈/가림에 훨씬 강건하다.

설계 (학생차 규모에 맞춘 최소 구성):
- 고정 크기(기본 8m x 8m, 5cm 해상도) 격자, 원점=첫 스캔 시점 pose(격자 중앙).
- 셀 값은 히트 카운트(uint8, hit_max 포화) — 로그오즈 대신 단순 카운트.
  슬롯 탐지에는 셀을 "비울" 필요가 없어(주차 차량은 안 움직임) 이걸로 충분.
- synthesize_scan(): 점유 셀을 현재 pose 기준의 원시 스캔 튜플
  [(0, raw_angle_deg, dist_mm)]로 역변환한다 — t_parking.slot_found()의
  검증된 chord 로직(filter_self → 측면 섹터 → 이웃 chord)을 코드 수정 없이
  누적 맵 위에서 그대로 재사용하기 위한 어댑터.
- 라이다 장착 오프셋(to_rear_m 75mm)은 무시한다: add_scan과 synthesize_scan이
  같은 (무시된) 변환을 쓰므로 chord 갭 판정에서는 상쇄된다.

오도메트리 미보정(pose_conf=0)이면 t_parking이 이 클래스를 아예 만들지 않고
기존 "최근 N회 스캔 deque" 동작으로 폴백한다 (fail-inert).

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.missions.occupancy --selftest
"""
import math

try:
    import numpy as np
except ImportError:
    np = None

from ..nodes.lidar_node import filter_self, vehicle_bearing_deg


def _bearing_to_raw_deg(bearing_deg, mount):
    """vehicle_bearing_deg의 역변환 — 차량 기준 bearing → 라이다 원시 각도."""
    a = bearing_deg - mount.get("yaw_offset_deg", 0.0) - 180.0
    if mount.get("invert"):
        a = -a
    return (a + 180.0) % 360.0 - 180.0


class OccupancyMap:
    """고정 크기 로컬 히트카운트 점유 격자 (odom frame)."""

    def __init__(self, size_m=8.0, res_m=0.05, hit_max=10, min_hits=2):
        assert np is not None, "occupancy 격자는 numpy 필요"
        self.size_m = float(size_m)
        self.res = float(res_m)
        self.hit_max = int(hit_max)
        self.min_hits = int(min_hits)
        self.n = int(round(size_m / res_m))
        self.grid = np.zeros((self.n, self.n), dtype=np.uint8)  # [iy, ix] 히트 수
        self.center_xy = None  # 첫 add_scan pose의 (x, y) — 격자 중앙

    # ---- 좌표 변환 ----

    def _origin_xy(self):
        """격자 (0,0) 셀 모서리의 odom 좌표 (ROS OccupancyGrid origin과 동일)."""
        cx, cy = self.center_xy
        return cx - self.size_m / 2.0, cy - self.size_m / 2.0

    def _to_cell(self, ox, oy):
        """odom 좌표 → (ix, iy). 격자 밖이면 None."""
        x0, y0 = self._origin_xy()
        ix = int((ox - x0) / self.res)
        iy = int((oy - y0) / self.res)
        if 0 <= ix < self.n and 0 <= iy < self.n:
            return ix, iy
        return None

    # ---- 누적/조회 ----

    def add_scan(self, scan, pose, mount, self_mask_deg):
        """원시 스캔 [(q, angle_deg, dist_mm)]을 pose로 odom 정합해 누적한다.
        자차 반사 제거는 lidar_node.filter_self 재사용."""
        x, y, theta = pose
        if self.center_xy is None:
            self.center_xy = (x, y)
        c, s = math.cos(theta), math.sin(theta)
        for bearing_deg, dist_mm in filter_self(scan, mount, self_mask_deg):
            d = dist_mm / 1000.0
            b = math.radians(bearing_deg)
            vx, vy = d * math.cos(b), d * math.sin(b)   # 차량 프레임 (X=전방, Y=좌)
            ox, oy = x + c * vx - s * vy, y + s * vx + c * vy
            cell = self._to_cell(ox, oy)
            if cell is not None and self.grid[cell[1], cell[0]] < self.hit_max:
                self.grid[cell[1], cell[0]] += 1

    def occupied_points(self, min_hits=None):
        """점유 확정 셀들의 odom 좌표 (N, 2) ndarray — 셀 중심 기준."""
        min_hits = self.min_hits if min_hits is None else min_hits
        iy, ix = np.where(self.grid >= min_hits)
        x0, y0 = self._origin_xy()
        return np.column_stack([(ix + 0.5) * self.res + x0,
                                (iy + 0.5) * self.res + y0])

    def synthesize_scan(self, pose, max_range_m, mount):
        """점유 셀들을 현재 pose 기준 원시 스캔 튜플로 역변환한다.

        반환 [(0, raw_angle_deg, dist_mm)] — slot_found()가 순간 스캔 대신
        이걸 넣으면 기존 filter_self/chord 로직이 누적 맵 위에서 그대로 돈다.
        같은 bearing에 여러 셀이 있으면 그대로 여러 점을 내보낸다 (chord
        판정은 정렬 후 이웃 간 거리라 중복 점은 무해)."""
        x, y, theta = pose
        c, s = math.cos(theta), math.sin(theta)
        out = []
        for ox, oy in self.occupied_points():
            dx, dy = ox - x, oy - y
            vx = c * dx + s * dy      # odom → 차량 프레임 역회전
            vy = -s * dx + c * dy
            d = math.hypot(vx, vy)
            if d < 0.05 or d > max_range_m:
                continue
            bearing = math.degrees(math.atan2(vy, vx))
            out.append((0, _bearing_to_raw_deg(bearing, mount), d * 1000.0))
        return out

    def to_ros_grid_data(self):
        """nav_msgs/OccupancyGrid용 (origin_x, origin_y, res, n, data).
        data: row-major int 리스트 — -1 미관측 / 50 히트 부족 / 100 점유."""
        x0, y0 = self._origin_xy()
        data = np.full(self.grid.shape, -1, dtype=np.int8)
        data[self.grid > 0] = 50
        data[self.grid >= self.min_hits] = 100
        return x0, y0, self.res, self.n, data.flatten().tolist()


# ========================= 오프라인 셀프테스트 =========================

def _synth_raw_scan(wall_pts, pose, mount, max_range_m=12.0):
    """odom 상의 점들을 해당 pose에서 본 원시 스캔으로 렌더 (테스트용)."""
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    out = []
    for ox, oy in wall_pts:
        dx, dy = ox - x, oy - y
        vx, vy = c * dx + s * dy, -s * dx + c * dy
        d = math.hypot(vx, vy)
        if d > max_range_m:
            continue
        bearing = math.degrees(math.atan2(vy, vx))
        out.append((15, _bearing_to_raw_deg(bearing, mount), d * 1000.0))
    return out


def selftest():
    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    if np is None:
        print("[X ] numpy 미설치 — occupancy 격자 테스트 불가")
        return 1

    mount = dict(yaw_offset_deg=0.0, invert=False, to_rear_m=0.075)
    mask = 75.0

    # 각도 역변환이 정변환의 정확한 역인지
    ok_inv = all(abs(vehicle_bearing_deg(_bearing_to_raw_deg(b, mount), mount) - b) < 1e-9
                 for b in (-179.0, -100.0, 80.0, 179.0))
    check("bearing↔raw 각도 왕복 변환 일치", ok_inv)
    m_inv = dict(mount, invert=True, yaw_offset_deg=7.0)
    ok_inv2 = all(abs(vehicle_bearing_deg(_bearing_to_raw_deg(b, m_inv), m_inv) - b) < 1e-9
                  for b in (-120.0, 100.0))
    check("invert/yaw_offset 조합에서도 왕복 일치", ok_inv2)

    # 우측 주차 차량 2대 + 갭 옆을 "지나치며" 정합: 전방 ±75도는 자차 마스크에
    # 걸리므로(실차와 동일) 차량들이 측면~후측방(75~165도)에 놓이는 구간에서만
    # 관측된다 — 서로 다른 pose에서 본 같은 벽이 같은 셀에 쌓여야 한다.
    car_a = [(0.3, -1.0), (0.5, -1.0), (0.7, -1.0)]          # 주차 차량 A
    car_b = [(1.9, -1.0), (2.1, -1.0), (2.3, -1.0)]          # 주차 차량 B (갭 1.2m)
    wall = car_a + car_b
    occ = OccupancyMap(size_m=8.0, res_m=0.05, min_hits=2)
    poses = [(x, 0.0, 0.0) for x in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0)]
    for pose in poses:
        occ.add_scan(_synth_raw_scan(wall, pose, mount), pose, mount, mask)

    pts = occ.occupied_points()
    # 셀 경계에 걸친 점은 pose에 따라 인접 셀 2개에 나뉘어 쌓일 수 있다 —
    # 정확한 개수 대신 "벽 점 수 이상, 약간의 여유 이내 + 전부 벽 근처"를 본다.
    check(f"점유 셀 수({len(pts)})가 벽 점 수({len(wall)})~+2 범위 — 다른 pose가 같은 셀로 정합",
          len(wall) <= len(pts) <= len(wall) + 2)
    if len(pts):
        err = max(min(math.hypot(px - wx, py - wy) for wx, wy in wall) for px, py in pts)
        check(f"점유 셀 중심이 실제 벽 위치와 셀 크기 내 오차 (max {err:.3f}m)",
              err <= 0.05)

    # synthesize_scan → filter_self 왕복: 마지막 pose에서 본 bearing/거리 복원
    last = poses[-1]
    synth = occ.synthesize_scan(last, max_range_m=6.0, mount=mount)
    seen = filter_self(synth, mount, mask)
    check("synthesize_scan 점이 전부 자차 마스크 밖(측면/후방)", len(seen) == len(synth))
    direct = filter_self(_synth_raw_scan(wall, last, mount), mount, mask)
    check(f"복원 점({len(seen)})이 직접 관측({len(direct)}) 이상", len(seen) >= len(direct))
    if seen and direct:
        # 복원된 각 점이 어떤 직접 관측점과든 근접해야 한다 (최근접 매칭 —
        # 인접 셀 중복 점도 원래 벽 점 근처이므로 같은 기준으로 판정 가능)
        def _nearest_err(pt):
            return min((abs(pt[0] - dp[0]), abs(pt[1] - dp[1])) for dp in direct)
        errs = [_nearest_err(pt) for pt in seen]
        b_err = max(e[0] for e in errs)
        d_err = max(e[1] for e in errs)
        check(f"최근접 bearing 오차 <= 3도 (max {b_err:.2f}), 거리 오차 <= 80mm (max {d_err:.0f})",
              b_err <= 3.0 and d_err <= 80.0)

    # 누적 맵 위 chord 갭: 이웃 점 사이 최대 chord가 실제 갭(1.2m)에 근접
    side_pts = sorted((b, d / 1000.0) for b, d in seen if -165.0 <= b <= -75.0)
    max_chord = 0.0
    for (b1, d1), (b2, d2) in zip(side_pts, side_pts[1:]):
        db = math.radians(abs(b2 - b1))
        chord = math.sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * math.cos(db))
        max_chord = max(max_chord, chord)
    check(f"우측 섹터 최대 chord({max_chord:.2f}m)가 갭 1.2m에 근접 (±0.15)",
          abs(max_chord - 1.2) <= 0.15)

    # to_ros_grid_data 형식
    x0, y0, res, n, data = occ.to_ros_grid_data()
    check("OccupancyGrid data 길이 == n*n, 값은 {-1,50,100}",
          len(data) == n * n and set(data) <= {-1, 50, 100})
    check("점유(100) 셀 존재", 100 in data)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    sys.exit(selftest())
