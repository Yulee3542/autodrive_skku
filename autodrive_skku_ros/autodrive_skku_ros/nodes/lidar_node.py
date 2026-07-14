#!/usr/bin/env python3
"""라이다 지오메트리 노드 — 공식 rplidar_ros가 발행하는 원시 /scan을 차량 기준
좌표로 보정해 재발행한다.

이 파일 상단의 함수들(vehicle_bearing_deg 등)은 순수 계산 로직(ROS 비의존,
하드웨어 불필요 — 셀프테스트 대상)이고, ros_main()의 LidarGeometryNode가 이를
얇게 감싸 ROS 토픽을 구독/발행한다. 하드웨어 I/O(시리얼 연결/스레드)는 공식
rplidar_ros 드라이버가 담당하므로 이 파일에는 없다.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.lidar_node --selftest
"""
import math

# 기본 장착 파라미터 — config.LIDAR_MOUNT 미전달 시 폴백 (2026-07-09 실측과 동일)
DEFAULT_MOUNT = dict(yaw_offset_deg=0.0, invert=False, to_rear_m=0.075)


# ---- 후방 장착 지오메트리 (순수 함수, 하드웨어 불필요 — 셀프테스트 대상) ----
#
# RP라이다는 차량 후방(후단 뒤 75mm, 지면 140mm)에 장착되고, T주차 후진용이라
# 각도 규약은 "원시 0도 = 차량 후방"이다. 아래 함수들은 원시 각도를
# 차량 전방 기준 bearing(-180..180, +가 좌측)으로 변환해 다룬다:
#   bearing = normalize(원시각도*(invert? -1:+1) + yaw_offset_deg + 180)
# 전방 |bearing| < self_mask_deg 는 자차 차체 반사(전방이 차체에 막힘)로 제거한다.

def vehicle_bearing_deg(raw_angle_deg, mount):
    """라이다 원시 각도 → 차량 전방 기준 bearing (-180..180, +좌측)."""
    a = -raw_angle_deg if mount.get("invert") else raw_angle_deg
    a = a + mount.get("yaw_offset_deg", 0.0) + 180.0
    return (a + 180.0) % 360.0 - 180.0


def filter_self(scan, mount, self_mask_deg=75.0):
    """유효 스캔점만 남긴다: [(bearing_deg, dist_mm), ...].

    - 거리 게이트 50~12000mm (노이즈/최대거리)
    - 전방 |bearing| < self_mask_deg 제거 — 자차 차체가 전방 wedge를 가려
      해당 각도의 반사는 전부 자차 자신이다.
    """
    out = []
    for _quality, angle, dist_mm in scan:
        if not (50 <= dist_mm <= 12000):
            continue
        b = vehicle_bearing_deg(angle, mount)
        if abs(b) < self_mask_deg:
            continue
        out.append((b, dist_mm))
    return out


def rear_min_m(scan, mount, sector_deg=30, self_mask_deg=75.0):
    """후방 ±sector_deg 내 최소 거리 — 뒤 범퍼 기준 m. 스캔 없으면 None.

    라이다 축이 뒤 범퍼보다 75mm 뒤에 있으므로 뒤 범퍼 기준 거리는
    라이다 거리 + to_rear_m 이다.
    """
    if not scan:
        return None
    to_rear = mount.get("to_rear_m", 0.075)
    dists = [d for b, d in filter_self(scan, mount, self_mask_deg)
             if abs(b) >= 180.0 - sector_deg]
    return (min(dists) / 1000.0 + to_rear) if dists else None


def side_clearance_m(scan, side, mount, window_deg=(75.0, 100.0), self_mask_deg=75.0,
                     min_m=0.30):
    """좌('L')/우('R') 측면 여유 거리 m. 스캔 없거나 반사 없으면 None.

    자차 실루엣을 벗어나는 abeam 창(기본 전방 기준 75~100도)만 사용 —
    이보다 전방 쪽 각도는 차체에 가려 자차 반사만 잡힌다.
    min_m: 시뮬 실측(2026-07-09)에서 자차 "코너" 반사가 bearing ~75-82도,
    0.20~0.26m에 남는 것이 확인됨 — self_mask 각도만으로는 못 거르므로
    이 근거리 게이트로 함께 제거한다 (실차 장착 후 재보정 대상).
    """
    if not scan:
        return None
    lo, hi = window_deg
    sign = 1.0 if side == "L" else -1.0
    dists = [d for b, d in filter_self(scan, mount, self_mask_deg)
             if lo <= sign * b <= hi and d / 1000.0 >= min_m]
    return (min(dists) / 1000.0) if dists else None


def laserscan_msg_to_tuples(msg):
    """sensor_msgs/LaserScan → [(quality, angle_deg, dist_mm), ...].

    filter_self/rear_min_m/side_clearance_m이 기대하는 원본 rplidar 튜플 형태로
    역변환한다 (quality는 이 함수들에서 쓰이지 않으므로 0으로 채운다).
    range_min/range_max 밖이거나 inf/nan인 레이는 "반사 없음"으로 스킵한다.
    """
    out = []
    for i, dist_m in enumerate(msg.ranges):
        if not math.isfinite(dist_m):
            continue
        if dist_m < msg.range_min or dist_m > msg.range_max:
            continue
        angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
        out.append((0, angle_deg, dist_m * 1000.0))
    return out


def scan_to_ranges(scan, mount, self_mask_deg=75.0, n_bins=360):
    """자차 반사 제거 + 차량 기준 bearing으로 재정렬한 균등 간격 LaserScan 배열.

    반환: (start_angle_rad, end_angle_rad, ranges_m) — ranges_m 길이는 n_bins,
    해당 bin에 반사가 없으면 float('nan') (Foxglove/rviz는 NaN을 무반사로 처리).
    각 bin은 -180..180도를 n_bins등분한 구간에 속하는 점 중 가장 가까운 거리를 채택한다.
    """
    start_angle = -math.pi
    end_angle = math.pi
    bin_width_deg = 360.0 / n_bins
    ranges_m = [float("nan")] * n_bins

    if not scan:
        return start_angle, end_angle, ranges_m

    for bearing_deg, dist_mm in filter_self(scan, mount, self_mask_deg):
        idx = int((bearing_deg + 180.0) / bin_width_deg) % n_bins
        dist_m = dist_mm / 1000.0
        if math.isnan(ranges_m[idx]) or dist_m < ranges_m[idx]:
            ranges_m[idx] = dist_m

    return start_angle, end_angle, ranges_m


N_BINS = 360


# ============================ ROS2 래퍼 ============================

def ros_main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32

    from .. import config

    class LidarGeometryNode(Node):
        """공식 rplidar_ros가 발행하는 /scan(원시 LaserScan)을 구독해, 2026-07-09
        후방 장착 실측 보정(config.LIDAR_MOUNT/LIDAR_SELF_MASK_DEG/LIDAR_REAR_SECTOR)을
        적용한 결과를 재발행한다.

        - /lidar/rear_min_m: 뒤 범퍼 기준 후방 섹터 최소 거리 (T주차 후진용, 없으면 NaN)
        - /lidar/scan_corrected: 자차 반사 제거 + 차량 기준 bearing으로 정렬한 LaserScan
          — 미션이 실제로 "보는" 시야를 Foxglove/rviz에서 그대로 확인 가능.

        주의: yaw_offset_deg/invert 캘리브레이션은 기존 파이썬 rplidar 라이브러리의
        각도 규약 기준으로 잡힌 값이라, rplidar_ros의 /scan 각도 규약과 정확히
        일치하는지 실차에서 재확인이 필요하다 — 다르면 config.LIDAR_MOUNT만 조정.
        """

        def __init__(self):
            super().__init__("lidar_geometry_node")

            self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
            self._rear_min_pub = self.create_publisher(Float32, "/lidar/rear_min_m", 10)
            self._corrected_pub = self.create_publisher(LaserScan, "/lidar/scan_corrected", 10)

        def _on_scan(self, msg):
            scan = laserscan_msg_to_tuples(msg)

            rear_min = rear_min_m(scan, config.LIDAR_MOUNT, config.LIDAR_REAR_SECTOR,
                                   config.LIDAR_SELF_MASK_DEG)
            self._rear_min_pub.publish(
                Float32(data=float("nan") if rear_min is None else rear_min))

            start_angle, end_angle, ranges_m = scan_to_ranges(
                scan, config.LIDAR_MOUNT, config.LIDAR_SELF_MASK_DEG, N_BINS)
            corrected = LaserScan()
            corrected.header = msg.header
            corrected.angle_min = start_angle
            corrected.angle_max = end_angle
            corrected.angle_increment = (end_angle - start_angle) / N_BINS
            corrected.time_increment = msg.time_increment
            corrected.scan_time = msg.scan_time
            corrected.range_min = 0.05
            corrected.range_max = 12.0
            corrected.ranges = ranges_m
            self._corrected_pub.publish(corrected)

    rclpy.init(args=args)
    node = LidarGeometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ========================= 오프라인 테스트 / 셀프테스트 =========================

class _FakeLaserScan:
    """sensor_msgs/LaserScan을 흉내내는 최소 더미 — rclpy 설치 없이도 테스트 가능."""

    def __init__(self, ranges, angle_min=-math.pi, angle_increment=None,
                 range_min=0.05, range_max=12.0):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = (2 * math.pi / len(ranges)) if angle_increment is None \
            else angle_increment
        self.range_min = range_min
        self.range_max = range_max


def selftest():
    """후방 장착(0도=차량 후방) 지오메트리 + LaserScan<->튜플 변환을 하드웨어/
    ROS 없이 검증한다. tools/smoke_test_missions.py의 옛 test_lidar_geometry/
    test_scan_conversion을 이 파일로 이관한 것 — 순수 함수라 소유 파일에 두는 게
    맞고, 중복 유지 비용을 없애기 위해 원본은 그쪽에서 제거했다."""
    from .. import config

    mount = config.LIDAR_MOUNT
    mask = config.LIDAR_SELF_MASK_DEG

    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    check("원시 0도 → 차량 bearing ±180 (후방→전방 기준)",
          abs(abs(vehicle_bearing_deg(0, mount)) - 180.0) < 1e-9)
    check("원시 180도 → bearing 0 (차량 전방)",
          abs(vehicle_bearing_deg(180, mount)) < 1e-9)
    check("전방 wedge(|b|<75)는 자차 반사로 제거",
          filter_self([(15, 180, 500), (15, 170, 500)], mount, mask) == [])
    check("후방/측면 반사는 유지",
          len(filter_self([(15, 0, 500), (15, 60, 800)], mount, mask)) == 2)

    r = rear_min_m([(15, 0, 250)], mount, config.LIDAR_REAR_SECTOR, mask)
    check(f"rear_min_m: 라이다 250mm → 뒤범퍼 기준 {r} == 0.325",
          r is not None and abs(r - 0.325) < 1e-9)

    left = side_clearance_m([(15, 270, 800)], "L", mount, config.LIDAR_SIDE_WINDOW_DEG, mask)
    check("side_clearance_m L (원시 270도=좌측 90도) == 0.8",
          left is not None and abs(left - 0.8) < 1e-9)
    check("반대쪽 창엔 안 잡힘",
          side_clearance_m([(15, 270, 800)], "R", mount,
                            config.LIDAR_SIDE_WINDOW_DEG, mask) is None)

    corner = (15, 256, 260)  # 시뮬 실측: 자차 코너 반사(bearing ~76도, 0.26m)
    check("자차 코너 반사(0.26m)는 측면 여유에서 제외",
          side_clearance_m([corner], "L", mount, config.LIDAR_SIDE_WINDOW_DEG, mask) is None)
    both = side_clearance_m([corner, (15, 270, 800)], "L", mount,
                             config.LIDAR_SIDE_WINDOW_DEG, mask)
    check("코너 반사 섞여도 실제 장애물(0.8m)만 반환",
          both is not None and abs(both - 0.8) < 1e-9)

    # ROS LaserScan <-> 튜플 변환 (rclpy 없이 순수 함수만 검증)
    msg = _FakeLaserScan(ranges=[1.0, float("inf"), 0.5, 100.0])
    tuples = laserscan_msg_to_tuples(msg)
    check("inf/range_max 밖 레이는 스킵 (4개 중 2개만 남음)", len(tuples) == 2)
    check("range_mm 변환 정확 (1.0m → 1000mm)",
          any(abs(dist_mm - 1000.0) < 1e-6 for _q, _a, dist_mm in tuples))
    check("angle_deg 변환 정확 (angle_min=-180도 그대로)",
          any(abs(angle_deg - (-180.0)) < 1e-6 for _q, angle_deg, _d in tuples))

    start, end, ranges_empty = scan_to_ranges([], mount, mask, n_bins=8)
    check("빈 스캔 → 전부 NaN", all(math.isnan(r) for r in ranges_empty))
    check("start/end_angle == -pi/+pi",
          abs(start + math.pi) < 1e-9 and abs(end - math.pi) < 1e-9)

    start, end, ranges_m = scan_to_ranges([(15, 180, 500), (15, 0, 800)], mount, mask, n_bins=8)
    check("전방 자차 반사는 제거되어 전방 bin이 NaN",
          math.isnan(ranges_m[len(ranges_m) // 2]))
    check("후방(bearing 0) 반사는 남음 (NaN 아닌 bin 존재)",
          any(not math.isnan(r) for r in ranges_m))

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    ros_main()
