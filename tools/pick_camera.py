#!/usr/bin/env python3
"""어느 카메라가 전방인지 사람이 한 번 확인해서 저장하는 도구.

왜 필요한가: `nodes/ports.py::autodetect_cameras()`는 이름(C920)으로 우리
카메라를 골라내지만, 같은 모델이 두 대(전방/후방)면 **어느 쪽이 전방인지는
영상 없이 알 수 없어** 찾은 순서대로 배정한다. 게다가 /dev/videoN 번호는
재부팅/재연결마다 바뀌고, 노트북 내장캠이 video0을 먼저 가져가는 사례가 실차에서
반복 확인됐다. 그래서 사람이 한 번 눈으로 확인한 결과를 **번호가 아니라 안정
식별자**(/dev/v4l/by-id 이름 — 커널이 USB 벤더/모델/시리얼로 만들어 재연결해도
동일)로 저장해 둔다. bringup.launch.py가 이 파일을 자동으로 우선 사용한다.

사용법:
    python3 tools/pick_camera.py            # 각 카메라 한 장씩 저장 후 선택
    python3 tools/pick_camera.py --show     # 디스플레이가 있으면 창으로도 표시
    python3 tools/pick_camera.py --list     # 감지 결과만 출력하고 종료

디스플레이가 없는 실차(SSH)에서도 쓸 수 있게, 기본 동작은 창을 띄우는 게 아니라
프레임을 파일로 저장하고 경로를 알려주는 것이다 — scp로 받아 보거나 Foxglove로
확인한 뒤 번호를 고르면 된다.
"""
import argparse
import glob
import json
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_TOOLS), "autodrive_skku_ros"))

try:
    import cv2
except ImportError:
    print("[X ] cv2 미설치 — pip install opencv-python 후 재실행")
    sys.exit(1)

from autodrive_skku_ros import config
from autodrive_skku_ros.nodes import ports


def list_video_indices():
    idx = []
    for path in sorted(glob.glob("/dev/video*")):
        try:
            idx.append(int(path.rsplit("video", 1)[1]))
        except (IndexError, ValueError):
            continue
    return sorted(set(idx))


def device_name(index):
    try:
        with open(f"/sys/class/video4linux/video{index}/name", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return "(이름 알 수 없음)"


def probe(index, out_dir):
    """카메라 한 장 캡처 → (성공 여부, 저장 경로 또는 사유)."""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        cap.release()
        return False, "열기 실패 (다른 프로세스가 점유 중이거나 캡처 장치가 아님)"
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        return False, "프레임 캡처 실패"
    path = os.path.join(out_dir, f"camera_video{index}.jpg")
    cv2.imwrite(path, frame)
    return True, path


def main():
    ap = argparse.ArgumentParser(description="전방/후방 카메라 확인 후 저장")
    ap.add_argument("--show", action="store_true", help="창으로도 표시(디스플레이 필요)")
    ap.add_argument("--list", action="store_true", help="감지 결과만 출력")
    ap.add_argument("--out-dir", default=None, help="프레임 저장 폴더(기본: 임시폴더)")
    args = ap.parse_args()

    if not sys.platform.startswith("linux"):
        print("[! ] 이 도구는 /dev/video* + /dev/v4l/by-id 가 있는 Linux(실차)용입니다.")
        print("     개발용 Windows에서는 저장할 안정 식별자가 없어 의미가 없습니다.")
        return 1

    indices = list_video_indices()
    if not indices:
        print("[X ] /dev/video* 장치가 없습니다 — 카메라 연결 / WSL2면 usbipd attach 확인")
        return 1

    out_dir = args.out_dir or os.path.join(os.path.expanduser("~"), "camera_check")
    os.makedirs(out_dir, exist_ok=True)

    print("== 감지된 카메라 ==")
    usable = []
    for i in indices:
        sid = ports.camera_stable_id(i)
        name = device_name(i)
        ok, info = probe(i, out_dir)
        mark = "OK" if ok else "X "
        print(f"  [{mark}] video{i:<2} {name}")
        print(f"        안정 식별자: {sid or '(by-id 없음 — 이 장치는 저장 불가)'}")
        print(f"        {'프레임: ' + info if ok else '사유: ' + info}")
        if ok and sid:
            usable.append((i, sid, info))
            if args.show:
                img = cv2.imread(info)
                if img is not None:
                    cv2.imshow(f"video{i}", img)

    if args.show and usable:
        print("\n아무 키나 누르면 창이 닫힙니다...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    if args.list:
        return 0
    if not usable:
        print("\n[X ] 저장 가능한 카메라가 없습니다 "
              "(by-id 식별자가 없거나 전부 캡처 실패).")
        return 1

    print(f"\n저장된 프레임을 열어보고 어느 것이 **전방**인지 고르세요 ({out_dir})")
    print("  (SSH라면: scp 로 받아서 보거나, --show 로 창 표시)")
    print("  선택 가능:", ", ".join(str(i) for i, _s, _p in usable))

    def ask(prompt, allow_blank=False):
        while True:
            try:
                raw = input(prompt).strip()
            except EOFError:
                print("\n[X ] 입력을 받을 수 없습니다(비대화형). 대화형 터미널에서 실행하세요.")
                sys.exit(1)
            if allow_blank and raw == "":
                return None
            if raw.isdigit() and int(raw) in [i for i, _s, _p in usable]:
                return int(raw)
            print("  목록에 있는 번호를 입력하세요" + (" (건너뛰려면 빈 줄)" if allow_blank else ""))

    front = ask("전방 카메라 번호: ")
    rear = ask("후방 카메라 번호 (없으면 빈 줄): ", allow_blank=True)

    sid = {i: s for i, s, _p in usable}
    mapping = {"front": sid[front]}
    if rear is not None:
        mapping["rear"] = sid[rear]

    path = config.CAMERA_MAP_PATH
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(mapping, f, ensure_ascii=False, indent=2)
        f.write("\n")

    print(f"\n[OK] 저장: {path}")
    for k, v in mapping.items():
        print(f"     {k}: {v}")
    print("\n이제 bringup.launch.py가 이 매핑을 자동으로 우선 사용합니다.")
    print("되돌리려면 이 파일을 지우면 기존 autodetect 동작으로 돌아갑니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
