"""프로토타입: 하단 1/2 ROI를 N개의 좁은 수평 밴드로 나눠 각 밴드에서 우측
차선(중앙 점선~우측 실선) 중앙점을 찾고, 그 점들을 이어 경로(polyline)로 삼는다.
가장 가까운 밴드(들)가 즉각 조향을, 전체 점들의 기울기가 곡선 방향(앞으로 휘는
쪽)을 알려준다. vendor 코드는 건드리지 않고 독립 실행으로 시각적 튜닝만 한다.

q 키로 종료.
"""
import sys
import time

import cv2
import numpy as np

WHITE_THRESH = 170        # 그레이스케일 흰색 임계값 (조명에 따라 튜닝)
ROI_FRAC = (0.67, 0.98)   # 전체 POI: 프레임 하단 1/3
N_BANDS = 4                # ROI를 이만큼의 수평 밴드로 분할 -> 경로 점 개수 (5단 중 맨 위/가장 먼 단 제거)
CLUSTER_GAP_PX = 15        # 이 이상 컬럼이 비면 별도 클러스터로 분리
MIN_CLUSTER_MASS = 8       # 밴드가 좁아서 낮게 설정
MAX_CLUSTER_WIDTH_PX = 60  # 이보다 넓은 블롭은 배경(바닥/벽)으로 간주해 제외
MIN_ROW_SPAN_FRAC = 0.55   # 실제 차선은 밴드 세로 전체를 가로질러야 함 -- 이 비율
                            # 미만으로만 뭉쳐있으면 반사/얼룩 등 노이즈로 보고 제외
CENTER_DEADZONE_PX = 20    # 이 안쪽이면 직진(F) 유지
NEAR_WEIGHT_DECAY = 0.6    # 경로점 가중 평균 시 먼 점일수록 이 비율로 감쇠

# 사다리꼴 POI: 원근상 먼 곳일수록 차선이 화면에서 차지하는 가로폭이 좁아지므로,
# 가장 가까운 밴드(0)는 전체 폭을, 가장 먼 밴드(N_BANDS-1)는 중앙 기준 좁은 폭만
# 본다 -- 그만큼 옆 배경(바닥/벽)이 섞여 들어올 여지가 줄어든다.
TRAPEZOID_NEAR_HALF_FRAC = 0.50   # 가장 가까운 밴드: 중앙 기준 ±50% (=전체 폭)
TRAPEZOID_FAR_HALF_FRAC = 0.22    # 가장 먼 밴드: 중앙 기준 ±22%로 좁힘


def find_clusters(binary, gap_px, min_mass, max_width, min_row_span_frac):
    """binary: 밴드의 2D 이진 이미지(row x col). 컬럼 방향으로 클러스터를 묶고,
    각 클러스터가 밴드 세로를 충분히 가로지르는지(실제 선다움)까지 확인한다."""
    band_h = binary.shape[0]
    col_sum = binary.sum(axis=0) / 255.0
    cols = np.where(col_sum > 0)[0]
    if len(cols) == 0:
        return []
    clusters = []
    start = cols[0]
    prev = cols[0]
    for c in cols[1:]:
        if c - prev > gap_px:
            clusters.append((start, prev))
            start = c
        prev = c
    clusters.append((start, prev))

    out = []
    for lo, hi in clusters:
        mass = int(col_sum[lo:hi + 1].sum())
        if mass < min_mass or (hi - lo) > max_width:
            continue
        sub = binary[:, lo:hi + 1]
        rows = np.where(sub.any(axis=1))[0]
        if len(rows) == 0:
            continue
        row_span = rows[-1] - rows[0] + 1
        if row_span < min_row_span_frac * band_h:
            continue  # 세로로 충분히 이어지지 않음 -> 반사/얼룩 등 노이즈로 판단
        out.append(((lo + hi) / 2.0, mass, lo, hi))
    return out


def pick_right_lane_center(clusters):
    """3개+: 우측 실선 + 중앙 점선의 중점. 2개: 좌/우 실선만 있다고 보고 3/4 지점
    보간. 1개 이하: None."""
    cols = [c[0] for c in clusters]
    if len(cols) >= 3:
        return (cols[-1] + cols[-2]) / 2.0, "3+"
    if len(cols) == 2:
        left, right = cols[0], cols[1]
        return left + 0.75 * (right - left), "2i"
    return None, "lo"


def band_rows(h, roi_frac, n_bands, i):
    """밴드 i(0=가장 가까움/화면 아래, n_bands-1=가장 멂/위)의 (y0,y1)."""
    y_bottom = int(h * roi_frac[1])
    y_top = int(h * roi_frac[0])
    band_h = (y_bottom - y_top) / n_bands
    y1 = y_bottom - int(i * band_h)
    y0 = y_bottom - int((i + 1) * band_h)
    return max(y0, y_top), y1


def band_x_range(w, cx, n_bands, i):
    """밴드 i(0=가까움..n_bands-1=멂)의 (x_lo, x_hi) -- 사다리꼴: 멀수록 좁아짐."""
    t = i / max(n_bands - 1, 1)
    half_frac = TRAPEZOID_NEAR_HALF_FRAC + t * (TRAPEZOID_FAR_HALF_FRAC - TRAPEZOID_NEAR_HALF_FRAC)
    half_px = half_frac * w
    x_lo = max(0, int(cx - half_px))
    x_hi = min(w, int(cx + half_px))
    return x_lo, x_hi


def detect_band(frame, y0, y1, x_lo, x_hi):
    """clusters/target 컬럼은 프레임 전체 기준 절대좌표로 반환(x_lo만큼 보정)."""
    band = frame[y0:y1, x_lo:x_hi]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, WHITE_THRESH, 255, cv2.THRESH_BINARY)
    clusters = find_clusters(binary, CLUSTER_GAP_PX, MIN_CLUSTER_MASS, MAX_CLUSTER_WIDTH_PX, MIN_ROW_SPAN_FRAC)
    clusters = [(c + x_lo, m, lo + x_lo, hi + x_lo) for (c, m, lo, hi) in clusters]
    target, reason = pick_right_lane_center(clusters)
    return clusters, target, reason


def main():
    cap = None
    for attempt in range(5):
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                break
        print(f"camera open attempt {attempt + 1}/5 failed, retrying...")
        cap.release()
        time.sleep(0.5)
    else:
        print("camera open FAILED after 5 attempts (likely still held by a previous instance)")
        input("Press Enter to close...")
        sys.exit(1)

    print(f"Press q to quit. {N_BANDS} bands, band0=nearest(bottom) .. band{N_BANDS - 1}=farthest(top of ROI).")
    print("Thick red = smoothed path point used for steering. Cyan polyline = full estimated path.")
    smoothed = None
    band_colors = [(0, 255, 0), (0, 255, 255), (255, 0, 0), (255, 0, 255), (0, 165, 255), (255, 255, 0)]

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        h, w = frame.shape[:2]
        cx = w / 2.0

        path_points = []   # [(row_y_mid, target_col, band_idx), ...] 가까운 순
        vis = frame.copy()
        band_dump = []
        for i in range(N_BANDS):
            y0, y1 = band_rows(h, ROI_FRAC, N_BANDS, i)
            x_lo, x_hi = band_x_range(w, cx, N_BANDS, i)
            clusters, target, reason = detect_band(frame, y0, y1, x_lo, x_hi)
            col = band_colors[min(i, len(band_colors) - 1)]
            cv2.rectangle(vis, (x_lo, y0), (x_hi - 1, y1), (90, 90, 90), 1)
            for (center, mass, lo, hi) in clusters:
                cv2.line(vis, (int(lo), y0), (int(lo), y1), col, 1)
                cv2.line(vis, (int(hi), y0), (int(hi), y1), col, 1)
            if target is not None:
                y_mid = (y0 + y1) // 2
                cv2.circle(vis, (int(target), y_mid), 4, (0, 0, 255), -1)
                path_points.append((y_mid, target, i))
            band_dump.append(f"b{i}:{'-' if target is None else f'{target:.0f}'}({reason})")

        # 경로 폴리라인 (가까운 -> 먼 순으로 정렬해서 그림)
        path_points.sort(key=lambda p: -p[0])  # y 큰(아래/가까움) -> 작은(위/멂)
        for a, b in zip(path_points, path_points[1:]):
            cv2.line(vis, (int(a[1]), a[0]), (int(b[1]), b[0]), (255, 255, 0), 2)

        # 사다리꼴 외곽선 (가장 가까운 밴드 하단 <-> 가장 먼 밴드 상단, 좌/우 각각)
        near_y0, near_y1 = band_rows(h, ROI_FRAC, N_BANDS, 0)
        far_y0, far_y1 = band_rows(h, ROI_FRAC, N_BANDS, N_BANDS - 1)
        near_xlo, near_xhi = band_x_range(w, cx, N_BANDS, 0)
        far_xlo, far_xhi = band_x_range(w, cx, N_BANDS, N_BANDS - 1)
        cv2.line(vis, (near_xlo, near_y1), (far_xlo, far_y0), (0, 100, 255), 1)
        cv2.line(vis, (near_xhi, near_y1), (far_xhi, far_y0), (0, 100, 255), 1)

        cv2.line(vis, (int(cx), 0), (int(cx), h), (180, 180, 180), 1)

        # 가장 가까운 몇 개 점을 거리 가중 평균해서 조향 목표로 사용
        # (곡선 진입 시 먼 점이 안쪽으로 휘어들어와도 즉각 반응은 가까운 점이 지배)
        raw_target = None
        if path_points:
            weights_sum = 0.0
            weighted = 0.0
            for rank, (_y, col, _i) in enumerate(path_points):
                weight = NEAR_WEIGHT_DECAY ** rank
                weighted += col * weight
                weights_sum += weight
            raw_target = weighted / weights_sum

        if raw_target is not None:
            smoothed = raw_target if smoothed is None else (0.7 * smoothed + 0.3 * raw_target)

        direction = None
        if smoothed is not None:
            cv2.line(vis, (int(smoothed), int(h * ROI_FRAC[0])), (int(smoothed), h), (0, 0, 255), 3)
            offset = smoothed - cx
            if offset > CENTER_DEADZONE_PX:
                direction = "R"
            elif offset < -CENTER_DEADZONE_PX:
                direction = "L"
            else:
                direction = "F"
            cv2.putText(vis, f"offset={offset:+.0f}px dir={direction} pts={len(path_points)}/{N_BANDS}",
                        (5, int(h * ROI_FRAC[0]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            raw_str = "held" if raw_target is None else f"{raw_target:.0f}"
            print(f"{' '.join(band_dump)} -> weighted={raw_str} smoothed={smoothed:.0f} offset={offset:+.0f} dir={direction} pts={len(path_points)}/{N_BANDS}")
        else:
            cv2.putText(vis, "no path points found", (5, int(h * ROI_FRAC[0]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            print(f"{' '.join(band_dump)} -> NO PATH")

        cv2.imshow("lane path (multi-band, q=quit)", vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        input("Press Enter to close...")
