import math

# 기본 장착 파라미터 — config.LIDAR_MOUNT 미전달 시 폴백 (2026-07-09 실측과 동일)
DEFAULT_MOUNT = dict(yaw_offset_deg=0.0, invert=False, to_rear_m=0.075)


# ---- 후방 장착 지오메트리 (순수 함수, 하드웨어 불필요 — 스모크 테스트 대상) ----
#
# RP라이다는 차량 후방(후단 뒤 75mm, 지면 140mm)에 장착되고, T주차 후진용이라
# 각도 규약은 "원시 0도 = 차량 후방"이다. 아래 함수들은 원시 각도를
# 차량 전방 기준 bearing(-180..180, +가 좌측)으로 변환해 다룬다:
#   bearing = normalize(원시각도*(invert? -1:+1) + yaw_offset_deg + 180)
# 전방 |bearing| < self_mask_deg 는 자차 차체 반사(전방이 차체에 막힘)로 제거한다.
#
# 하드웨어 I/O(시리얼 연결/스레드)는 ROS 2 전환 후 공식 rplidar_ros 드라이버
# (/scan, sensor_msgs/LaserScan)가 담당한다 — 이 파일에는 순수 함수만 남고,
# LaserScan 메시지 ↔ 이 함수들이 쓰는 [(quality, angle_deg, dist_mm), ...] 튜플
# 표현 사이의 변환 헬퍼만 추가돼 있다(laserscan_msg_to_tuples / scan_to_ranges).

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
