"""주차 슬롯 상대자세 검출 — 오도메트리 없이 매 스캔 직접 측정.

팀 저장소 HANLAB_auto(yeoeun_traffic) `mission4_parking/parking_node.py`의
`SlotDetector` 이식. 우리 t_parking은 슬롯 위치를 누적 점유격자로 잡는데,
그 격자는 오도메트리(`config.ODOMETRY.pwm_to_mps`)가 보정돼야 생성된다 —
지금은 미실측이라 fail-inert로 꺼져 있어 실질적으로 순간 스캔 폴백만 돈다.
이 모듈은 **적분을 전혀 하지 않고** 매 스캔마다 양옆 주차차량 대비 상대자세를
직접 재므로 드리프트가 원리적으로 없고, 오도메트리가 꺼져 있어도 동작한다.

측정값 SlotObs:
    e_y     [m]  슬롯 중심선 대비 횡오차 (좌+/우-)
    e_theta [rad] 슬롯축 대비 헤딩오차 (슬롯축은 180° 대칭이라 ±π/2로 접음)
    d       [m]  깊이 — 옆차들이 내 라이다보다 뒤로 뻗은 거리
    gap     [m]  두 옆차 안쪽면 간격 (검출 타당성 판정에 사용)

핵심 아이디어(원본 주석 유지): 슬롯축은 두 옆차 '안쪽면' 중심을 잇는 선의 수직
방향으로 잡는다. PCA는 보이는 면이 측면이냐 후면이냐에 따라 90° 튀는 약점이
있어 이 방식이 더 강건하다.

좌표 규약 주의: `lidar_node.vehicle_bearing_deg()`가 돌려주는 bearing은 **차량
전방 기준**이다(0°=전방, +좌측, ±180°=후방) — 라이다 원시각도 0°가 차량 후방인
것과 헷갈리기 쉬우니 주의. 여기 점군은 (x=뒤쪽+, y=좌측+) 미터로 다시 옮긴다.

ROS 비의존. 셀프테스트: python3 -m autodrive_skku_ros.missions.slot_detect --selftest
"""
import math

try:
    import numpy as np
except ImportError:
    np = None

from ..nodes.lidar_node import vehicle_bearing_deg

# 대회 규격 차량/슬롯 치수 (팀 저장소 값 — 같은 대회, 같은 유아전동차 규격)
CAR_W = 0.52
SLOT_W = 0.95

SLOT_DETECT = dict(
    roi_x=(0.05, 3.5),       # 후방 거리 범위 [m]
    roi_y=3.0,               # 횡 범위 ± [m] (시작자세에서 옆차 y≈2.1까지 포함해야 함)
    cluster_gap=0.20,        # 인접점 거리 점프가 이보다 크면 다른 클러스터 📏
    cluster_min_pts=5,
    gap_range=(0.70, 1.90),  # 두 옆차 안쪽면 간격 허용범위 [m] 📏
    gap_expect=SLOT_W + (SLOT_W - CAR_W),  # 기대 간격 ≈1.38m (쌍 선택 점수)
    face_band_m=0.10,        # 안쪽면 밴드 두께 [m]
    face_min_pts=6,
    face_min_span=0.25,      # 밴드가 이보다 짧으면 각도 신뢰 불가 [m]
    face_trim_res=0.03,      # LSQ 잔차 트리밍 임계 [m]
)


def scan_to_rear_points(scan, mount, cfg=SLOT_DETECT):
    """우리 스캔 튜플 [(quality, raw_angle_deg, dist_mm), ...] → Nx2 점군
    (x=뒤쪽+, y=좌측+, 미터). ROI 밖/무효 점은 제거. numpy 없으면 None.

    vehicle_bearing_deg는 **전방 기준**(0°=전방, +좌측)이므로 후방 프레임으로
    뒤집는다: x_back = -r·cos(b), y_left = +r·sin(b).
    (원시 0°=후방 → bearing ±180° → x_back=+r ✓ / 원시 90° → bearing -90° → y<0 우측 ✓)"""
    if np is None or not scan:
        return None
    out = []
    lo, hi = cfg["roi_x"]
    for point in scan:
        try:
            _q, raw_deg, dist_mm = point
        except (TypeError, ValueError):
            continue
        r = float(dist_mm) / 1000.0
        if r <= 0:
            continue
        b = math.radians(vehicle_bearing_deg(raw_deg, mount))
        x = -r * math.cos(b)
        y = r * math.sin(b)
        if lo < x < hi and abs(y) < cfg["roi_y"]:
            out.append((x, y))
    return np.array(out, dtype=float) if out else None


def _wrap_half(a):
    """각도를 [-π/2, π/2)로 접기 (슬롯축은 180° 대칭)."""
    while a >= math.pi / 2:
        a -= math.pi
    while a < -math.pi / 2:
        a += math.pi
    return a


def _face_angle(C, is_upper, cfg):
    """클러스터 안쪽면 밴드에 LSQ 선적합(+잔차 트리밍) → (각도, 밴드길이).

    is_upper: 회전좌표계에서 위쪽(큰 y') 클러스터면 안쪽면=min y', 아래면 max y'.
    양끝점 방식은 후면 코너 점 오염에 취약해 원본에서 폐기됨. 밴드가 짧으면 None."""
    y_in = C[:, 1].min() if is_upper else C[:, 1].max()
    band = C[np.abs(C[:, 1] - y_in) < cfg["face_band_m"]]
    if len(band) < cfg["face_min_pts"] or np.ptp(band[:, 0]) < cfg["face_min_span"]:
        return None
    k = 0.0
    for _ in range(2):
        k, b = np.polyfit(band[:, 0], band[:, 1], 1)
        res = np.abs(band[:, 1] - (k * band[:, 0] + b))
        keep = res < cfg["face_trim_res"]
        if keep.sum() < cfg["face_min_pts"] or keep.all():
            break
        band = band[keep]
    return math.atan2(k, 1.0), float(np.ptp(band[:, 0]))


class SlotObs:
    """슬롯 상대자세 한 번의 관측 (적분 없음 — 이 스캔만으로 계산된 값)."""

    __slots__ = ("e_y", "e_theta", "d", "gap")

    def __init__(self, e_y, e_theta, d, gap):
        self.e_y, self.e_theta, self.d, self.gap = e_y, e_theta, d, gap

    def __repr__(self):
        return (f"SlotObs(e_y={self.e_y:+.3f}m, e_th={math.degrees(self.e_theta):+.1f}deg, "
                f"d={self.d:.2f}m, gap={self.gap:.2f}m)")


def _cluster(pts, cfg):
    """각도순 정렬 후 인접점 거리 점프로 순차 클러스터링."""
    order = np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))
    pts = pts[order]
    clusters, cur = [], [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - cur[-1]) > cfg["cluster_gap"]:
            if len(cur) >= cfg["cluster_min_pts"]:
                clusters.append(np.array(cur))
            cur = [p]
        else:
            cur.append(p)
    if len(cur) >= cfg["cluster_min_pts"]:
        clusters.append(np.array(cur))
    return clusters


def detect_slot(points, cfg=SLOT_DETECT):
    """Nx2 후방프레임 점군 → SlotObs | None (검출 실패 시 None — 호출부는
    "이번 스캔은 못 봤다"로 처리하고 이전 상태를 유지할 것).

    좌/우(y부호) 페어링을 전제하지 않는다 — 슬롯이 차량 '옆'에 보이는 진입
    초반 자세에서도 검출되도록 임의 클러스터 쌍을 평가하고, 위/아래 구분과
    간격은 슬롯축 회전좌표계에서 판정한다(원본 리뷰#1 반영)."""
    if np is None or points is None or len(points) < cfg["cluster_min_pts"] * 2:
        return None
    pts = np.asarray(points, float)
    clusters = _cluster(pts, cfg)
    if len(clusters) < 2:
        return None

    best = None
    for i in range(len(clusters)):
        for j in range(i + 1, len(clusters)):
            A, B = clusters[i], clusters[j]
            # 1) 슬롯축 2단계 추정 — (러프) 두 클러스터 중심을 잇는 선의 수직,
            #    (정밀) 러프축 회전좌표계에서 안쪽면 밴드에 LSQ 적합 ×2
            v = A.mean(axis=0) - B.mean(axis=0)
            th = _wrap_half(math.atan2(-v[0], v[1]))
            for _ in range(2):
                ci, si = math.cos(-th), math.sin(-th)
                Rot = np.array([[ci, -si], [si, ci]])
                Ai, Bi = A @ Rot.T, B @ Rot.T
                up, lo = (Ai, Bi) if Ai[:, 1].mean() > Bi[:, 1].mean() else (Bi, Ai)
                num = den = 0.0
                for Ci, is_up in ((up, True), (lo, False)):
                    fa = _face_angle(Ci, is_up, cfg)
                    if fa is not None:
                        num += fa[0] * fa[1]
                        den += fa[1]
                if den == 0:
                    break
                th = _wrap_half(th + num / den)
            # 2) 최종 회전. 접힘(±180°) 분기 해소: 슬롯/옆차는 항상 후방(x'>0)에
            #    있어야 물리적으로 맞으므로, 아니면 좌표를 반전한다.
            ci, si = math.cos(-th), math.sin(-th)
            Rot = np.array([[ci, -si], [si, ci]])
            Ar, Br = A @ Rot.T, B @ Rot.T
            if max(Ar[:, 0].max(), Br[:, 0].max()) < 0.0:
                Ar, Br = -Ar, -Br
            up, lo = (Ar, Br) if Ar[:, 1].mean() > Br[:, 1].mean() else (Br, Ar)
            u_in = up[:, 1].min()      # 위쪽 차 안쪽면
            l_in = lo[:, 1].max()      # 아래쪽 차 안쪽면
            gap = u_in - l_in
            if not (cfg["gap_range"][0] < gap < cfg["gap_range"][1]):
                continue
            center_y = (u_in + l_in) / 2.0
            d = max(Ar[:, 0].max(), Br[:, 0].max())
            score = abs(gap - cfg["gap_expect"])   # 기대 간격에 가까운 쌍 우선
            if best is None or score < best[0]:
                best = (score, SlotObs(center_y, th, d, gap))
    return best[1] if best else None


# ---------------- 셀프테스트용 합성 스캔 ----------------

def _rect_points(cx, cy, w, h, psi=0.0, step=0.02):
    """중심(cx,cy) 크기(w,h) 회전 psi 인 직사각형 둘레 점군."""
    pts = []
    n_w, n_h = max(int(w / step), 2), max(int(h / step), 2)
    for k in range(n_w):
        u = -w / 2 + w * k / (n_w - 1)
        pts += [(u, -h / 2), (u, h / 2)]
    for k in range(n_h):
        v = -h / 2 + h * k / (n_h - 1)
        pts += [(-w / 2, v), (w / 2, v)]
    c, s = math.cos(psi), math.sin(psi)
    return [(cx + u * c - v * s, cy + u * s + v * c) for u, v in pts]


def _visible(pts, n_bins=720):
    """원점(라이다)에서 실제로 보이는 점만 남긴다 — 방위각 구간마다 가장 가까운
    점 하나. 실물 라이다는 물체 뒷면을 못 보므로 이 가림(occlusion)을 반영해야
    합성 데이터가 물리적으로 말이 된다. (안 하면 차 한 대의 앞/뒷면이 서로 다른
    클러스터로 잡혀 '슬롯'처럼 보이는 등 검출기가 실제로 겪지 않을 상황이 만들어짐)"""
    best = {}
    for x, y in pts:
        r = math.hypot(x, y)
        if r <= 0:
            continue
        b = int((math.atan2(y, x) + math.pi) / (2 * math.pi) * n_bins)
        if b not in best or r < best[b][0]:
            best[b] = (r, (x, y))
    return np.array([p for _r, p in best.values()], dtype=float)


def _synth(gap=1.38, e_y=0.0, e_theta=0.0, depth=1.0):
    """슬롯 양옆 차량 2대를 라이다가 보는 대로 합성. 차량 좌표계(x=뒤+, y=좌+)
    기준으로 e_y 만큼 옆으로, e_theta 만큼 돌아간 슬롯을 만든다.
    SlotObs.e_y 규약과 맞추려면 차량 중심을 +e_y 방향으로 옮겨야 한다
    (그래야 안쪽면 중점 center_y = +e_y)."""
    pts = []
    for side in (+1, -1):
        u, v = depth + 0.5, side * (gap / 2 + CAR_W / 2) + e_y
        c, s = math.cos(e_theta), math.sin(e_theta)
        pts += _rect_points(u * c - v * s, u * s + v * c, 1.05, CAR_W, e_theta)
    return _visible(pts)


def _selftest():
    ok = True

    def check(name, cond):
        nonlocal ok
        print(f"  [{'OK' if cond else 'X '}] {name}")
        ok = ok and bool(cond)

    print("== slot_detect ==")
    if np is None:
        print("  [OK] numpy 미설치 — 스킵")
        return True

    # (1) 정렬된 슬롯: e_y≈0, e_theta≈0
    obs = detect_slot(_synth())
    check("정렬된 슬롯 검출", obs is not None)
    if obs:
        check(f"e_y≈0 ({obs.e_y:+.3f})", abs(obs.e_y) < 0.08)
        check(f"e_theta≈0 ({math.degrees(obs.e_theta):+.1f}deg)",
              abs(obs.e_theta) < math.radians(6))
        check(f"gap이 기대치 부근 ({obs.gap:.2f} vs {SLOT_DETECT['gap_expect']:.2f})",
              abs(obs.gap - SLOT_DETECT["gap_expect"]) < 0.25)

    # (2) 횡오차: 부호와 크기가 따라와야 조향에 쓸 수 있다
    o_l = detect_slot(_synth(e_y=+0.25))
    o_r = detect_slot(_synth(e_y=-0.25))
    check("횡오차 +0.25 검출", o_l is not None and abs(o_l.e_y - 0.25) < 0.10)
    check("횡오차 -0.25 검출", o_r is not None and abs(o_r.e_y + 0.25) < 0.10)
    check("좌/우 부호가 반대", o_l is not None and o_r is not None
          and o_l.e_y > 0 > o_r.e_y)

    # (3) 헤딩오차
    o_t = detect_slot(_synth(e_theta=math.radians(15)))
    check(f"헤딩오차 15도 검출 ({math.degrees(o_t.e_theta):+.1f}deg)"
          if o_t else "헤딩오차 15도 검출",
          o_t is not None and abs(o_t.e_theta - math.radians(15)) < math.radians(7))

    # (4) 깊이가 멀어지면 d도 커진다 (단조성만 확인 — 절대값은 마운트 의존)
    d1 = detect_slot(_synth(depth=0.6))
    d2 = detect_slot(_synth(depth=1.6))
    check("깊이 증가 시 d 증가", d1 is not None and d2 is not None and d2.d > d1.d)

    # (5) 안전 반환 — 오검출보다 미검출이 안전한 쪽
    check("점군 None → None", detect_slot(None) is None)
    check("점 부족 → None", detect_slot(np.zeros((3, 2))) is None)
    check("차가 한 대뿐이면 → None (슬롯은 양옆 두 대가 있어야 성립)",
          detect_slot(_visible(_rect_points(1.0, 0.0, 1.05, CAR_W))) is None)
    far = _synth(gap=3.0)   # 간격이 규격 밖 → 슬롯 아님
    check("간격이 gap_range 밖 → None", detect_slot(far) is None)

    # (6) 스캔 튜플 변환 좌표 규약 (0°=후방, 90°=우측, 270°=좌측)
    mount = dict(yaw_offset_deg=0.0, invert=False, to_rear_m=0.0)
    pts = scan_to_rear_points([(15, 0, 1000)], mount)
    check("원시 0° → 정후방 (x>0, y≈0)",
          pts is not None and pts[0][0] > 0.9 and abs(pts[0][1]) < 0.05)
    # 원시 45°/315° = 후방 대각 (정측면 90°/270°은 x_back=0이라 후방 ROI 밖 — 정상)
    diag = scan_to_rear_points([(15, 45, 1000), (15, 315, 1000)], mount)
    check("원시 45°=후방우측(y<0) / 315°=후방좌측(y>0)",
          diag is not None and len(diag) == 2
          and any(p[1] < -0.5 for p in diag) and any(p[1] > 0.5 for p in diag)
          and all(p[0] > 0.5 for p in diag))
    check("정측면(원시 90°)은 후방 ROI 밖이라 제외",
          scan_to_rear_points([(15, 90, 1000)], mount) is None)
    return ok


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(0 if _selftest() else 1)
    print(__doc__)
