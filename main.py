import argparse
import time

try:
    import cv2
except ImportError:
    cv2 = None

import config
from src.missions import MISSIONS
from src.nodes.arduino_node import ArduinoNode
from src.nodes.camera_node import CameraNode
from src.nodes.lidar_node import LidarNode
from src.nodes.ports import autodetect_ports

MISSION_DESC = {
    "road": "도로 주행 — 차선 인식/차선 변경/장애물 회피",
    "traffic": "신호등 주행 — 정지선/신호등 인식",
    "t_parking": "T 주차 — 라이다 맵/후방캠 주차선",
}


def pick_mission():
    names = list(MISSIONS)
    print("\n미션 선택:")
    for i, name in enumerate(names, 1):
        print(f"  {i}. {name:<10} {MISSION_DESC.get(name, '')}")
    while True:
        choice = input("번호 또는 이름 입력 > ").strip().lower()
        if choice in MISSIONS:
            return choice
        if choice.isdigit() and 1 <= int(choice) <= len(names):
            return names[int(choice) - 1]
        print("잘못된 입력입니다.")


def show_frames(top, bottom, rear):
    if cv2 is None:
        return True
    if top is not None:
        cv2.imshow("top (traffic light)", top)
    if bottom is not None:
        cv2.imshow("bottom (lane)", bottom)
    if rear is not None:
        cv2.imshow("rear (parking)", rear)
    return (cv2.waitKey(1) & 0xFF) != ord("q")


def main():
    parser = argparse.ArgumentParser(description="autodrive_skku 미션 런처")
    parser.add_argument("--mission", choices=list(MISSIONS), help="생략하면 메뉴에서 선택")
    parser.add_argument("--arduino", default=config.ARDUINO_PORT, help="아두이노 시리얼 포트")
    parser.add_argument("--lidar", default=config.LIDAR_PORT, help="라이다 시리얼 포트")
    parser.add_argument("--front-camera", type=int, default=config.FRONT_CAMERA)
    parser.add_argument("--rear-camera", type=int, default=config.REAR_CAMERA,
                        help="T주차용 후방 카메라 인덱스 (기본: 미사용)")
    parser.add_argument("--no-split", action="store_true",
                        help="전방 프레임 상/하 분할 비활성화")
    parser.add_argument("--show", action="store_true", help="카메라 창 표시 (q로 종료)")
    args = parser.parse_args()

    mission_name = args.mission or pick_mission()
    mission = MISSIONS[mission_name]()

    arduino_port, lidar_port = args.arduino, args.lidar
    if arduino_port is None or lidar_port is None:
        auto_arduino, auto_lidar = autodetect_ports()
        arduino_port = arduino_port or auto_arduino
        lidar_port = lidar_port or auto_lidar

    print(f"[main] mission={mission_name} arduino={arduino_port} lidar={lidar_port}")

    car = ArduinoNode(arduino_port, config.ARDUINO_BAUD)
    cameras = CameraNode(args.front_camera, args.rear_camera,
                         split=config.CAMERA_SPLIT and not args.no_split,
                         width=config.FRAME_WIDTH, height=config.FRAME_HEIGHT,
                         rotate=config.FRONT_CAMERA_ROTATE)
    lidar = LidarNode(lidar_port, config.LIDAR_BAUD,
                      mount=config.LIDAR_MOUNT,
                      self_mask_deg=config.LIDAR_SELF_MASK_DEG)

    period = 1.0 / config.LOOP_HZ
    mission.on_start(car, config)
    print("[main] 실행 중 — Ctrl+C 로 종료")
    try:
        while True:
            top, bottom = cameras.latest()
            rear = cameras.rear()
            sensors = {
                "top": top,
                "bottom": bottom,
                "rear": rear,
                "lidar_min_m": lidar.min_distance_m(config.LIDAR_REAR_SECTOR),
                "lidar_scan": lidar.scan,
                "state": car.state,
            }
            mission.step(sensors, car)
            if args.show and not show_frames(top, bottom, rear):
                break
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[main] 종료 — 차량 정지")
        mission.on_stop(car)
        car.close()
        cameras.close()
        lidar.close()


if __name__ == "__main__":
    main()
