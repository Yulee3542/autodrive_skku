"""감지기 분석 결과를 프레임에 그려주는 디버그 오버레이 (Foxglove /debug/* 용).

prototypes/lane_center_poi_windows_test.py의 시각화(밴드 사각형/클러스터 경계선/
목표점/경로 폴리라인/사다리꼴 외곽/스무딩 라인)를 ROS 포팅판에 맞게 옮긴 것.
mission_node의 오버레이 타이머가 Mission.debug 스크래치를 읽어 여기 함수들로
그린 뒤 CompressedImage로 발행한다.

전부 순수 그리기 함수: cv2.imshow 없이 vis 프레임만 반환한다(headless 안전).
dbg가 비었거나 None이면 원본 사본에 'no data' 표시만 하고 돌려준다 — 오버레이
경로에서 예외가 나도 미션 루프에는 영향이 없어야 한다(mission_node가 try로 감쌈).
"""
try:
    import cv2
except ImportError:
    cv2 = None

_BAND_COLORS = [(0, 255, 0), (0, 255, 255), (255, 0, 0), (255, 0, 255),
                (0, 165, 255), (255, 255, 0)]
_RED = (0, 0, 255)
_CYAN = (255, 255, 0)
_GRAY = (150, 150, 150)
_GREEN = (0, 255, 0)
_ORANGE = (0, 100, 255)


def _no_data(frame, label):
    vis = frame.copy()
    cv2.putText(vis, f"{label}: no data", (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, _RED, 1)
    return vis


def draw_lane_poi(frame, details):
    """follow_lane_poi가 반환한 details를 그린다 — 프로토타입 오버레이 포팅."""
    if not details:
        return _no_data(frame, "lane_poi")
    vis = frame.copy()
    h = details["h"]
    cx = details["cx"]
    bands = details["bands"]

    for i, band in enumerate(bands):
        col = _BAND_COLORS[min(i, len(_BAND_COLORS) - 1)]
        cv2.rectangle(vis, (band["x_lo"], band["y0"]),
                      (band["x_hi"] - 1, band["y1"]), (90, 90, 90), 1)
        for (_center, _mass, lo, hi) in band["clusters"]:
            cv2.line(vis, (int(lo), band["y0"]), (int(lo), band["y1"]), col, 1)
            cv2.line(vis, (int(hi), band["y0"]), (int(hi), band["y1"]), col, 1)
        if band["target"] is not None:
            y_mid = (band["y0"] + band["y1"]) // 2
            cv2.circle(vis, (int(band["target"]), y_mid), 4, _RED, -1)

    # 경로 폴리라인 (가까운 → 먼)
    pts = details["path_points"]
    for a, b in zip(pts, pts[1:]):
        cv2.line(vis, (int(a[1]), a[0]), (int(b[1]), b[0]), _CYAN, 2)

    # 사다리꼴 외곽선 (가장 가까운 밴드 하단 ↔ 가장 먼 밴드 상단)
    if bands:
        near, far = bands[0], bands[-1]
        cv2.line(vis, (near["x_lo"], near["y1"]), (far["x_lo"], far["y0"]), _ORANGE, 1)
        cv2.line(vis, (near["x_hi"], near["y1"]), (far["x_hi"], far["y0"]), _ORANGE, 1)

    # 프레임 중앙선 + deadzone 밴드
    cv2.line(vis, (int(cx), 0), (int(cx), h), (180, 180, 180), 1)
    deadzone = details.get("deadzone")
    if deadzone:
        roi_y = int(h * details.get("roi_frac", (0.67, 0.98))[0])
        cv2.line(vis, (int(cx - deadzone), roi_y), (int(cx - deadzone), h), (100, 100, 100), 1)
        cv2.line(vis, (int(cx + deadzone), roi_y), (int(cx + deadzone), h), (100, 100, 100), 1)

    # 스무딩된 조향 목표(굵은 빨간 세로선) + 상태 텍스트
    smoothed = details.get("smoothed")
    roi_top_y = int(h * details.get("roi_frac", (0.67, 0.98))[0])
    if smoothed is not None:
        cv2.line(vis, (int(smoothed), roi_top_y), (int(smoothed), h), _RED, 3)
        offset = details.get("offset")
        direction = details.get("direction")
        text = (f"offset={offset:+.0f}px dir={direction} " if offset is not None
                else "held ") + f"pts={len(pts)}/{len(bands)}"
        cv2.putText(vis, text, (5, max(roi_top_y - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, _RED, 1)
    else:
        cv2.putText(vis, "no path points found", (5, max(roi_top_y - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, _RED, 1)
    return vis


def draw_obstacle(frame, dbg):
    """detect_obstacle_ahead의 debug dict를 그린다 — ROI + 블롭 bbox(합격=빨강/탈락=회색)."""
    if not dbg or "roi" not in dbg:
        return _no_data(frame, "obstacle")
    vis = frame.copy()
    x0, y0, x1, y1 = dbg["roi"]
    cv2.rectangle(vis, (x0, y0), (x1 - 1, y1 - 1), _GRAY, 1)
    for (bx, by, bw, bh, passed) in dbg.get("blobs", []):
        cv2.rectangle(vis, (bx, by), (bx + bw, by + bh),
                      _RED if passed else _GRAY, 2 if passed else 1)
    verdict = "OBSTACLE!" if dbg.get("result") else "clear"
    cv2.putText(vis, verdict, (x0 + 3, max(y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, _RED if dbg.get("result") else _GREEN, 2)
    return vis


def draw_stop_line(frame, dbg):
    """stop_line_detected의 debug dict를 그린다 — ROI 경계 + 행 채움비 바그래프."""
    if not dbg or "row_frac" not in dbg:
        return _no_data(frame, "stop_line")
    vis = frame.copy()
    h, w = vis.shape[:2]
    roi_y0 = dbg["roi_y0"]
    cv2.line(vis, (0, roi_y0), (w, roi_y0), _GRAY, 1)
    # 각 행의 흰 픽셀 채움비를 왼쪽에서부터의 가로 바로 그림 + row_fill 임계 세로선
    bar_w = int(w * 0.25)  # 바그래프 최대 폭 (프레임 좌측 1/4)
    thr_x = int(bar_w * dbg["row_fill"])
    for i, f in enumerate(dbg["row_frac"]):
        y = roi_y0 + i
        if y >= h:
            break
        filled = int(bar_w * min(float(f), 1.0))
        if filled > 0:
            hit = f >= dbg["row_fill"]
            cv2.line(vis, (0, y), (filled, y), _RED if hit else _GREEN, 1)
    cv2.line(vis, (thr_x, roi_y0), (thr_x, h), _ORANGE, 1)
    verdict = "STOP LINE!" if dbg.get("result") else f"fill<{dbg['row_fill']}"
    cv2.putText(vis, verdict, (5, max(roi_y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, _RED if dbg.get("result") else _GREEN, 2)
    return vis


def draw_traffic_light(frame, dbg):
    """detect_light_color의 debug dict를 그린다 — 픽셀 카운트/임계/판정."""
    if not dbg or "red" not in dbg:
        return _no_data(frame, "traffic_light")
    vis = frame.copy()
    result = dbg.get("result")
    color = _RED if result == "red" else _GREEN if result == "green" else _GRAY
    cv2.putText(vis, f"light={result}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(vis, f"red={dbg['red']} green={dbg['green']} min={dbg['min_pixels']}",
                (5, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return vis


def draw_parking_line(frame, dbg):
    """reverse_lane_steer의 debug dict를 그린다 — 주차선 클러스터/중점/허용대."""
    if not dbg or "roi_y0" not in dbg:
        return _no_data(frame, "parking_line")
    vis = frame.copy()
    h, w = vis.shape[:2]
    roi_y0 = dbg["roi_y0"]
    cx = w / 2.0
    cv2.line(vis, (0, roi_y0), (w, roi_y0), _GRAY, 1)
    cv2.line(vis, (int(cx), roi_y0), (int(cx), h), (180, 180, 180), 1)
    tol = dbg.get("tol")
    if tol:
        cv2.line(vis, (int(cx - tol), roi_y0), (int(cx - tol), h), (100, 100, 100), 1)
        cv2.line(vis, (int(cx + tol), roi_y0), (int(cx + tol), h), (100, 100, 100), 1)
    for c in dbg.get("clusters", []):
        cv2.line(vis, (int(c), roi_y0), (int(c), h), _GREEN, 2)
    mid = dbg.get("mid")
    if mid is not None:
        cv2.line(vis, (int(mid), roi_y0), (int(mid), h), _RED, 3)
        cv2.putText(vis, f"err={dbg['err']:+.0f}px steer={dbg['steer']}",
                    (5, max(roi_y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _RED, 1)
    else:
        cv2.putText(vis, f"lines={len(dbg.get('clusters', []))}<2 (hold)",
                    (5, max(roi_y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _ORANGE, 1)
    return vis
