"""감지기 분석 결과를 프레임에 그려주는 디버그 오버레이 (Foxglove /debug/* 용).

lane_poi/parking_line 오버레이는 raw 측정(얇은 주황선)과 칼만필터로 스무딩된
추정(굵은 빨간선)을 함께 그린다 — sigma(sqrt(variance))도 텍스트로 표시해
실차 튜닝 중 필터가 얼마나 흔들리는지/수렴했는지 한눈에 보이게 한다.

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
    """follow_lane_poi가 반환한 details를 그린다 — BEV(원근변환된) 캔버스 위에
    그린다(2026-07-23, BEV 도입) — 원본 bottom 프레임이 아니라 details["bev"]가
    캔버스다(밴드/클러스터/목표점이 전부 BEV 픽셀좌표라 원본 프레임에 그리면
    좌표가 안 맞음). details가 없을 때만(no-data) 인자로 받은 raw frame을 쓴다."""
    if not details or "bev" not in details:
        return _no_data(frame, "lane_poi")
    vis = details["bev"].copy()
    h = details["h"]
    w = details["w"]
    cx = details["cx"]
    bands = details["bands"]

    for i, band in enumerate(bands):
        col = _BAND_COLORS[min(i, len(_BAND_COLORS) - 1)]
        cv2.line(vis, (0, band["y0"]), (w, band["y0"]), (90, 90, 90), 1)
        for (_center, _mass, lo, hi) in band["clusters"]:
            cv2.line(vis, (int(lo), band["y0"]), (int(lo), band["y1"]), col, 1)
            cv2.line(vis, (int(hi), band["y0"]), (int(hi), band["y1"]), col, 1)
        if band["target"] is not None:
            y_mid = (band["y0"] + band["y1"]) // 2
            cv2.circle(vis, (int(band["target"]), y_mid), 4, _RED, -1)

    # 경로 폴리라인 (가까운 → 먼) + raw_target(얇은 주황 세로선)
    pts = details["path_points"]
    for a, b in zip(pts, pts[1:]):
        cv2.line(vis, (int(a[1]), a[0]), (int(b[1]), b[0]), _CYAN, 2)
    raw_target = details.get("raw_target")
    if raw_target is not None:
        cv2.line(vis, (int(raw_target), 0), (int(raw_target), h), _ORANGE, 1)

    # 코리도어 락(직전 프레임 SOLID 경계) — 검색에서 제외된 영역을 어둡게 틴트
    corridor = details.get("corridor") or {}
    if corridor.get("left") is not None:
        lx = int(corridor["left"])
        vis[:, :max(0, lx)] = (vis[:, :max(0, lx)] * 0.5).astype(vis.dtype)
        cv2.line(vis, (lx, 0), (lx, h), _ORANGE, 1)
    if corridor.get("right") is not None:
        rx = int(corridor["right"])
        vis[:, min(w, rx):] = (vis[:, min(w, rx):] * 0.5).astype(vis.dtype)
        cv2.line(vis, (rx, 0), (rx, h), _ORANGE, 1)

    # 프레임 중앙선(차량 중심선)
    cv2.line(vis, (int(cx), 0), (int(cx), h), (180, 180, 180), 1)

    # Pure-Pursuit lookahead 지점 + 조향각/속도 상태 텍스트
    lookahead_px = details.get("lookahead_px")
    if lookahead_px is not None:
        lx, ly = lookahead_px
        ccx, ccy = details.get("car_center_px", (cx, h - 1))
        cv2.circle(vis, (int(lx), int(ly)), 6, _CYAN, 2)
        cv2.line(vis, (int(ccx), int(ccy)), (int(lx), int(ly)), _CYAN, 1)

    smoothed = details.get("smoothed")
    direction = details.get("direction")
    variance = details.get("variance")
    if smoothed is not None:
        sigma = f" sigma={variance ** 0.5:.1f}deg" if variance is not None else ""
        text = f"delta={smoothed:+.1f}deg dir={direction} pts={len(pts)}/{len(bands)}{sigma}"
        cv2.putText(vis, text, (5, max(h - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, _RED, 1)
    else:
        cv2.putText(vis, "no path points found", (5, max(h - 8, 12)),
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
    """TrafficMission._detect_light_color의 debug dict를 그린다 — YOLO(bbox+
    confidence) 또는 HSV(픽셀 카운트/임계) 판정, 어느 쪽이 쓰였는지(source)."""
    if not dbg or "result" not in dbg:
        return _no_data(frame, "traffic_light")
    vis = frame.copy()
    result = dbg.get("result")
    color = _RED if result == "red" else _GREEN if result == "green" else _GRAY
    cv2.putText(vis, f"light={result} ({dbg.get('source', '?')})",
                (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    if dbg.get("source") == "yolo":
        bbox = dbg.get("bbox")
        if bbox is not None:
            x1, y1, x2, y2 = (int(v) for v in bbox)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        cv2.putText(vis, f"class={dbg.get('class_name')} conf={dbg.get('confidence')}",
                    (5, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    else:
        cv2.putText(vis, f"red={dbg.get('red')} green={dbg.get('green')} min={dbg.get('min_pixels')}",
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
    # raw_err 위치(얇은 주황선)와 칼만필터로 스무딩된 err(굵은 빨간선)를 함께 표시
    raw_err = dbg.get("raw_err")
    if raw_err is not None:
        cv2.line(vis, (int(cx + raw_err), roi_y0), (int(cx + raw_err), h), _ORANGE, 1)
    mid = dbg.get("mid")
    if mid is not None:
        cv2.line(vis, (int(mid), roi_y0), (int(mid), h), _RED, 3)
        variance = dbg.get("variance")
        sigma = f" sigma={variance ** 0.5:.1f}px" if variance is not None else ""
        cv2.putText(vis, f"err={dbg['err']:+.0f}px steer={dbg['steer']}{sigma}",
                    (5, max(roi_y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _RED, 1)
    else:
        cv2.putText(vis, f"lines={len(dbg.get('clusters', []))}<2 (hold)",
                    (5, max(roi_y0 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _ORANGE, 1)
    return vis
