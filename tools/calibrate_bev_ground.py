#!/usr/bin/env python3
"""BEV 좌표계 지면 호모그래피 캘리브레이션 도구.

선배(taegyu) auto_drive_py 프로젝트의 ground_calibration.py를 이 팀 저장소의
BEV 파이프라인에 맞게 단순화해서 이식했다(2026-08-03) — TOML 플랜/웹 세션
같은 원본의 부가 기능은 빼고, 체커보드 검출 + 호모그래피 피팅 핵심만 가져왔다.

lane_follow.py의 LANE_POI는 지금 raw 프레임 -> ROI 크롭 -> src_frac 사다리꼴
원근변환(BEV)까지 거친 뒤, 그 BEV 픽셀을 **단일 스칼라 px_per_m**으로 미터
환산한다. LANE_POI 주석에 적혀 있듯 이 BEV 워프는 세로 3.2배 확대/가로 0.5배
축소로 축별 배율이 6.4배 어긋나 있어(무왜곡 기본 src_frac이라 진짜 사다리꼴이
아님), 스칼라 하나로는 원리적으로 두 축을 동시에 맞출 수 없다.

이 도구는 **BEV 워프 이후** 이미지에서 체커보드를 찍어 그 BEV 픽셀 ->
실제 지면 미터의 진짜 호모그래피를 피팅한다. BEV 워프 자체(클러스터링이
튜닝된 좌표계)는 건드리지 않는다 — 마지막 스칼라 환산 단계만 교체한다.

사용법 (오프라인, 이미 찍어둔 사진들로):
    python tools/calibrate_bev_ground.py \\
        --rows 3 --cols 4 --square-m 0.025 \\
        --shots calibration/shots.json \\
        --out calibration/ground_projection_front.json

--shots는 JSON 배열: [{"image": "shot1.jpg", "x_m": 0.5, "y_m": 0.0}, ...]
(square_m은 --square-m 하나로 전체 샷에 공통 적용)
image는 --shots 파일 기준 상대경로 가능. x_m/y_m은 보드에서 차량에 가장
가까운 안쪽 교점 행(row 0)의 **중앙**을 차량 좌표계(x=전방, y=좌측, 후륜축
원점)로 잰 값이다 — square_m 격자 간격으로 나머지 교점 좌표를 계산한다.

라이브 캡처 모드(카메라에서 즉석으로): --camera <index> --live
매 샷마다 보드를 옮기고 Enter, x_m/y_m을 입력하면 그 자리에서 프레임을
잡아 저장한다.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                 "autodrive_skku_ros"))

try:
    import cv2
    import numpy as np
except ImportError as e:
    print(f"[X] cv2/numpy 미설치: {e}")
    print("    pip install opencv-python-headless numpy 로 설치 후 재실행")
    sys.exit(1)

from autodrive_skku_ros.missions.lane_follow import _bev_warp, LANE_POI
from autodrive_skku_ros.calibration.ground_projection import (
    CameraIntrinsics, GroundProjection)


def detect_corners(bev_bgr, rows, cols):
    """BEV 이미지에서 체커보드 내부교점 (rows,cols,2) 픽셀 그리드를 찾는다.

    canonical_orientation: row 0 = 차량에 가장 가까움(이미지 아래쪽, v가 큼),
    col 0 = 이미지 왼쪽(=차량 좌측, +y) — board_ground_points()가 이 순서를
    그대로 가정하므로 반드시 이 정렬을 지켜야 한다."""
    grey = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2GRAY)
    found, detected = cv2.findChessboardCorners(
        grey, (cols, rows),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    corners = cv2.cornerSubPix(grey, detected, (11, 11), (-1, -1), criteria)
    grid = np.asarray(corners, dtype=np.float64).reshape(rows, cols, 2)

    # 행/열이 뒤바뀌어 검출됐으면 전치(행 안에서는 u가 더 크게 변하고,
    # 열을 따라서는 v가 더 크게 변해야 정상).
    if (float(np.mean(np.ptp(grid[:, :, 0], axis=1)))
            < float(np.mean(np.ptp(grid[:, :, 1], axis=1)))
            and float(np.mean(np.ptp(grid[:, :, 1], axis=0)))
            < float(np.mean(np.ptp(grid[:, :, 0], axis=0)))):
        grid = np.transpose(grid, (1, 0, 2))
    # row 0을 차량에 가장 가까운 쪽(v가 큰 쪽)으로.
    if float(np.mean(grid[0, :, 1])) < float(np.mean(grid[-1, :, 1])):
        grid = grid[::-1]
    # col 0을 이미지 왼쪽으로.
    if float(np.mean(grid[:, 0, 0])) > float(np.mean(grid[:, -1, 0])):
        grid = grid[:, ::-1]
    return grid


def board_ground_points(rows, cols, square_m, near_x_m, centre_y_m):
    """보드 놓인 자리(row 0 중앙의 x, 보드 전체의 y중심)로부터 내부교점
    (rows,cols,2) 그리드를 차량 기준 미터로 계산한다.

    row가 커질수록(=체커보드 안쪽으로 갈수록) 차량에서 더 멀어지므로 x가
    커진다. col 0은 차량 좌측(+y)이므로 col이 커질수록 y가 감소한다 —
    detect_corners()의 정렬(row 0=가까움, col 0=왼쪽)과 반드시 짝이 맞아야
    한다."""
    points = np.empty((rows, cols, 2), dtype=np.float64)
    for row in range(rows):
        for col in range(cols):
            points[row, col, 0] = near_x_m + square_m * row
            points[row, col, 1] = centre_y_m + square_m * ((cols - 1) / 2.0 - col)
    return points


def collect_from_files(shots_path, rows, cols, square_m):
    with open(shots_path, "r", encoding="utf-8") as f:
        shots = json.load(f)
    base_dir = os.path.dirname(os.path.abspath(shots_path))

    image_pts, ground_pts = [], []
    for i, shot in enumerate(shots):
        image_path = shot["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(base_dir, image_path)
        frame = cv2.imread(image_path)
        if frame is None:
            print(f"[X] {i}: 이미지를 읽지 못함 - {image_path}")
            continue
        bev, _M = _bev_warp(frame, LANE_POI)
        grid = detect_corners(bev, rows, cols)
        if grid is None:
            print(f"[X] {i}: 체커보드 검출 실패 - {image_path}")
            continue
        ground = board_ground_points(rows, cols, square_m, shot["x_m"], shot["y_m"])
        image_pts.extend(grid.reshape(-1, 2).tolist())
        ground_pts.extend(ground.reshape(-1, 2).tolist())
        print(f"[OK] {i}: {os.path.basename(image_path)} - {rows*cols}개 교점 검출")
    return image_pts, ground_pts


def collect_live(camera_index, rows, cols, square_m):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[X] 카메라 {camera_index}를 열지 못함")
        sys.exit(1)

    image_pts, ground_pts = [], []
    shot_i = 0
    print("보드를 놓고 Enter (그만하려면 q + Enter):")
    while True:
        cmd = input(f"[샷 {shot_i}] 계속하려면 Enter, 종료하려면 q: ").strip()
        if cmd.lower() == "q":
            break
        ok, frame = cap.read()
        if not ok:
            print("[X] 프레임을 못 읽음")
            continue
        bev, _M = _bev_warp(frame, LANE_POI)
        grid = detect_corners(bev, rows, cols)
        if grid is None:
            print("[X] 체커보드를 못 찾음 — 각도/거리를 조정하고 다시 시도")
            continue
        try:
            x_m = float(input("  row0(가까운 쪽) 중앙까지 전방거리 x_m: "))
            y_m = float(input("  보드 전체 좌우중심 y_m (좌측=+): "))
        except ValueError:
            print("[X] 숫자가 아님 — 이 샷은 건너뜀")
            continue
        ground = board_ground_points(rows, cols, square_m, x_m, y_m)
        image_pts.extend(grid.reshape(-1, 2).tolist())
        ground_pts.extend(ground.reshape(-1, 2).tolist())
        print(f"[OK] 샷 {shot_i} 기록됨 ({rows*cols}개 교점)")
        shot_i += 1
    cap.release()
    return image_pts, ground_pts


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rows", type=int, required=True, help="체커보드 내부교점 행 수")
    parser.add_argument("--cols", type=int, required=True, help="체커보드 내부교점 열 수")
    parser.add_argument("--square-m", type=float, required=True, help="칸 한 변 길이(m)")
    parser.add_argument("--shots", help="오프라인 모드: 샷 목록 JSON 경로")
    parser.add_argument("--camera", type=int, help="라이브 모드: 카메라 인덱스")
    parser.add_argument("--live", action="store_true", help="라이브 캡처 모드 사용")
    parser.add_argument("--revision", default=None,
                         help="캘리브레이션 식별자 (기본: 오늘 날짜)")
    parser.add_argument("--mounting", default="front", choices=("front", "rear"))
    parser.add_argument("--out", default="calibration/ground_projection_front.json")
    args = parser.parse_args()

    if args.rows == args.cols:
        print("[X] rows == cols 인 정사각 패턴은 90도 회전을 구분 못 함 - "
              "서로 다른 값을 쓰세요 (예: 3x4)")
        sys.exit(1)

    if args.live:
        if args.camera is None:
            print("[X] --live에는 --camera가 필요합니다")
            sys.exit(1)
        image_pts, ground_pts = collect_live(args.camera, args.rows, args.cols, args.square_m)
    elif args.shots:
        image_pts, ground_pts = collect_from_files(args.shots, args.rows, args.cols, args.square_m)
    else:
        print("[X] --shots 또는 --live 중 하나가 필요합니다")
        sys.exit(1)

    if len(image_pts) < 4:
        print(f"[X] 유효한 대응점이 {len(image_pts)}개뿐 — 최소 4개 필요 "
              "(체커보드 하나당 rows*cols개이므로 실질적으로 최소 1장 성공해야 함)")
        sys.exit(1)

    bev_w, bev_h = LANE_POI["bev_w"], LANE_POI["bev_h"]
    intrinsics = CameraIntrinsics.identity(bev_w, bev_h)
    revision = args.revision or f"bev-{__import__('datetime').date.today().isoformat()}"

    try:
        projection, report = GroundProjection.fit(
            image_pts, ground_pts, intrinsics, revision=revision, mounting=args.mounting)
    except ValueError as e:
        print(f"[X] 호모그래피 피팅 실패: {e}")
        sys.exit(1)

    print(f"\n== 피팅 결과 ({report['point_count']}점) ==")
    print(f"  중앙값 잔차: {report['median_residual_m']*1000:.2f}mm")
    print(f"  RMS 잔차:   {report['rms_residual_m']*1000:.2f}mm")
    print(f"  최대 잔차:   {report['maximum_residual_m']*1000:.2f}mm "
          f"(점 #{report['worst_index']})")
    print(f"  유효 전방:   {projection.valid_x_m[0]:.3f} ~ {projection.valid_x_m[1]:.3f} m")
    print(f"  유효 좌우:   {projection.valid_y_m[0]:.3f} ~ {projection.valid_y_m[1]:.3f} m")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(projection.to_dict(), f, indent=2, ensure_ascii=False)
    print(f"\n[OK] 저장됨: {args.out}")
    print("     lane_follow.py의 LANE_POI['ground_projection_path']가 이 경로를 "
          "가리키면 다음 기동부터 자동으로 이 호모그래피를 쓴다.")


if __name__ == "__main__":
    main()
