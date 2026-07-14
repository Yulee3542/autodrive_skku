#!/usr/bin/env python3
"""카메라 캡처/발행 노드 — 전방(+선택 후방) C920을 읽어 ROS 토픽으로 발행한다.

CameraNode: 백그라운드 스레드로 카메라를 계속 읽는 순수 파이썬 클래스 (ROS
비의존). ros_main()의 CameraPublisherNode가 이 클래스를 얇게 감싸 타이머로
최신 프레임을 /camera/top, /camera/bottom, /camera/rear에 CompressedImage(jpeg)로
발행한다. cv_bridge 불필요 — CompressedImage는 순수 bytes라 cv2.imencode 결과를
그대로 담는다.

오프라인 셀프테스트 (ROS 불필요): python3 -m autodrive_skku_ros.nodes.camera_node --selftest
"""
import sys
import threading

try:
    import cv2
except ImportError:
    cv2 = None

WSL2_HINT = "WSL2라면 Windows에서 usbipd attach 필요 (README 'WSL2에서 실행' 절 참고)"

_ROTATE_MAP = {}
if cv2 is not None:
    _ROTATE_MAP = {
        "CW": cv2.ROTATE_90_CLOCKWISE,
        "CCW": cv2.ROTATE_90_COUNTERCLOCKWISE,
        "180": cv2.ROTATE_180,
    }


class CameraNode:
    """전방 C920(+선택 후방) 카메라를 백그라운드 스레드로 계속 읽는다.

    split=True(기본)면 전방 프레임을 상/하 절반으로 나눠 latest()가
    (top=신호등용, bottom=차선용)을 반환한다. 후방 카메라는 rear()로 접근.

    rotate: 전방 카메라가 portrait(세로)로 물리 마운트된 경우의 회전 보정.
    None|"CW"|"CCW"|"180" — 후방 카메라에는 적용하지 않는다.
    """

    def __init__(self, front_index, rear_index=None, split=True,
                 width=640, height=480, rotate=None):
        self._split = split
        self._width = width
        self._height = height
        self._rotate = _ROTATE_MAP.get(rotate)
        self._lock = threading.Lock()
        self._front_frame = None
        self._rear_frame = None
        self._front = self._open(front_index, "front")
        self._rear = self._open(rear_index, "rear") if rear_index is not None else None
        self._running = self._front is not None or self._rear is not None
        if self._running:
            threading.Thread(target=self._loop, daemon=True).start()

    def _open(self, index, name):
        if cv2 is None:
            print("[camera] opencv 미설치 — 카메라 없이 실행")
            return None
        if sys.platform.startswith("linux"):
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            print(f"[camera] {name}({index}) 열기 실패 — {WSL2_HINT}")
            return None
        # USB 웹캠은 MJPG로 열어야 640x480@30fps가 안정적으로 나온다
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        print(f"[camera] {name}({index}) 연결됨")
        return cap

    def _loop(self):
        while self._running:
            if self._front is not None:
                ok, frame = self._front.read()
                if ok:
                    if self._rotate is not None:
                        frame = cv2.rotate(frame, self._rotate)
                    with self._lock:
                        self._front_frame = frame
            if self._rear is not None:
                ok, frame = self._rear.read()
                if ok:
                    with self._lock:
                        self._rear_frame = frame

    def latest(self):
        """(top_frame, bottom_frame). split이면 전방 프레임의 상/하 절반."""
        with self._lock:
            frame = self._front_frame
        if frame is None:
            return None, None
        if self._split:
            h = frame.shape[0]
            return frame[:h // 2, :], frame[h // 2:, :]
        return frame, frame

    def rear(self):
        with self._lock:
            return self._rear_frame

    def close(self):
        self._running = False
        for cap in (self._front, self._rear):
            if cap is not None:
                cap.release()
        if cv2 is not None:
            cv2.destroyAllWindows()


# rear_camera_index 파라미터가 이 값이면 "후방 카메라 미사용"(config.REAR_CAMERA=None과 동일).
NO_REAR_CAMERA = -1


# ============================ ROS2 래퍼 ============================

def ros_main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage

    from .. import config

    class CameraPublisherNode(Node):
        """CameraNode(캡처/상하분할/회전 보정)를 그대로 소유하고, 타이머로 최신
        프레임을 /camera/top, /camera/bottom, /camera/rear에 CompressedImage(jpeg)로
        발행한다.
        """

        def __init__(self):
            super().__init__("camera_publisher_node")

            self.declare_parameter("front_camera_index", config.FRONT_CAMERA)
            self.declare_parameter("rear_camera_index", NO_REAR_CAMERA
                                    if config.REAR_CAMERA is None else config.REAR_CAMERA)
            self.declare_parameter("split", config.CAMERA_SPLIT)
            self.declare_parameter("rotate", config.FRONT_CAMERA_ROTATE or "")
            self.declare_parameter("jpeg_quality", 80)

            rear_index = self.get_parameter("rear_camera_index").value
            rear_index = None if rear_index == NO_REAR_CAMERA else rear_index
            rotate = self.get_parameter("rotate").value or None
            self._jpeg_quality = self.get_parameter("jpeg_quality").value

            self._cameras = CameraNode(
                self.get_parameter("front_camera_index").value,
                rear_index,
                split=self.get_parameter("split").value,
                width=config.FRAME_WIDTH,
                height=config.FRAME_HEIGHT,
                rotate=rotate,
            )

            self._top_pub = self.create_publisher(CompressedImage, "/camera/top", 10)
            self._bottom_pub = self.create_publisher(CompressedImage, "/camera/bottom", 10)
            self._rear_pub = self.create_publisher(CompressedImage, "/camera/rear", 10)

            self.create_timer(1.0 / config.LOOP_HZ, self._tick)

        def _publish_frame(self, publisher, frame):
            if cv2 is None or frame is None:
                return
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
            if not ok:
                return
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = buf.tobytes()
            publisher.publish(msg)

        def _tick(self):
            top, bottom = self._cameras.latest()
            rear = self._cameras.rear()
            self._publish_frame(self._top_pub, top)
            self._publish_frame(self._bottom_pub, bottom)
            self._publish_frame(self._rear_pub, rear)

        def destroy_node(self):
            self._cameras.close()
            super().destroy_node()

    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ========================= 오프라인 테스트 / 셀프테스트 =========================

def selftest():
    """실제 카메라를 열지 않고 CameraNode.latest()의 상하 분할 로직과 portrait
    회전 후 shape만 검증한다 (tools/smoke_test_lane_follow.py::test_portrait_
    rotation_shapes와 동일한 관심사를 이 파일 로컬로도 확인)."""
    if cv2 is None:
        print("[X ] opencv 미설치 — 카메라 셀프테스트 불가")
        return 1
    import numpy as np

    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    # __init__을 거치지 않고(=실제 카메라를 열지 않고) latest()의 순수 분할 로직만 검증
    cam = CameraNode.__new__(CameraNode)
    cam._split = True
    cam._lock = threading.Lock()
    cam._front_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cam._rear_frame = None

    top, bottom = cam.latest()
    check("split=True: top/bottom 모두 (240, 640, 3)",
          top.shape == (240, 640, 3) and bottom.shape == (240, 640, 3))

    cam._split = False
    top, bottom = cam.latest()
    check("split=False: top == bottom == 원본 프레임", top is bottom is cam._front_frame)

    cam._front_frame = None
    top, bottom = cam.latest()
    check("프레임 없으면 (None, None)", top is None and bottom is None)

    rotated = cv2.rotate(np.zeros((480, 640, 3), dtype=np.uint8), cv2.ROTATE_90_CLOCKWISE)
    check("portrait 마운트(CW 회전) 후 shape (640, 480, 3)", rotated.shape == (640, 480, 3))

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    ros_main()
