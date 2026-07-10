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
