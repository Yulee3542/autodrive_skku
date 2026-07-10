import time

try:
    import cv2
except ImportError:
    cv2 = None

from .base import Mission
from .lane_follow import follow_lane

try:
    from ..vendor import Function_Library as fl
    from ..vendor.Function_Library import HUE_THRESHOLD, SATURATION, RED, GREEN
except ImportError:  # 패키지 미설치 개발 환경 — 검증된 상수값만 복사해 사용
    fl = None
    RED, GREEN = 0, 1
    HUE_THRESHOLD = ([4, 176], [40, 80])
    SATURATION = 150


def detect_light_color(frame, min_ratio=0.005):
    """상단 프레임에서 빨강/초록 픽셀 비율로 신호등 판정. 'red'/'green'/None.

    검증된 HUE_THRESHOLD/SATURATION 값을 그대로 사용한다. 디스플레이가 있는
    환경에서는 fl.libCAMERA().object_detection(원 검출 방식)으로 교체 가능.
    """
    if cv2 is None or frame is None:
        return None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, _v = cv2.split(hsv)
    s_cond = s > SATURATION

    red = (((h < HUE_THRESHOLD[RED][0]) | (h > HUE_THRESHOLD[RED][1])) & s_cond).sum()
    green = (((h > HUE_THRESHOLD[GREEN][0]) & (h < HUE_THRESHOLD[GREEN][1])) & s_cond).sum()

    min_pixels = frame.shape[0] * frame.shape[1] * min_ratio
    if red >= min_pixels and red > green * 2:
        return "red"
    if green >= min_pixels and green > red * 2:
        return "green"
    return None


class TrafficMission(Mission):
    """2. 신호등 주행

    목표:
      (1) 정지선 인식      — 테스트 구현 (흰색 가로 밴드, 행 채움비 판정)
      (2) 신호등 라이트 인식 — 동작 (HSV 픽셀 비율 판정)

    동작: 차선 추종 주행 중 정지선을 만나면 정지, 초록불이면 다시 출발.
    빨간불은 언제든 즉시 정지 (main3 검증 로직과 동일).

    대기 상태 구분 (wait):
      "red"  — 빨간불을 본 대기. 초록불이 뜰 때까지 무기한 대기
               (빨간불 출발은 규정 감점이라 타임아웃 없음).
      "line" — 정지선 대기. 초록불이면 출발, 신호등이 아예 안 보이면
               wait_max_s 후 경고 출력 후 재출발 (교착 방지 가드 —
               규정상 감점 여지가 있어 wait_max_s는 팀 확인 필요).
    재출발 직후에는 cooldown_s 동안 같은 정지선에 재정지하지 않는다.
    """

    name = "traffic"

    def on_start(self, car, config):
        assert set(config.LANE_EDGE) == {"width", "height", "gap", "threshold"}, \
            f"config.LANE_EDGE 키가 예상과 다름: {set(config.LANE_EDGE)}"
        self.config = config
        self.env = fl.libCAMERA() if fl is not None else None
        self._now = time.monotonic  # 테스트에서 가짜 시계 주입 지점
        self.wait = None            # None | "line" | "red"
        self.wait_t0 = 0.0
        self.cooldown_until = 0.0
        car.go()
        car.drive(config.DRIVE_SPEED)

    def step(self, sensors, car):
        now = self._now()
        color = detect_light_color(sensors["top"], self.config.TRAFFIC_PIXEL_RATIO)

        if color == "red" and self.wait != "red":
            self.wait = "red"
            self.wait_t0 = now

        if self.wait is not None:
            car.drive(0)
            if color == "green":
                self._resume(car, now)
            elif self.wait == "line" and color is None and \
                    now - self.wait_t0 >= self.config.STOP_LINE["wait_max_s"]:
                print("[traffic] 신호등 미검출 — 대기 시간 초과, 재출발")
                self._resume(car, now)
            return

        if now >= self.cooldown_until and self.stop_line_detected(sensors["bottom"]):
            self.wait = "line"
            self.wait_t0 = now
            car.drive(0)
            return

        car.drive(self.config.DRIVE_SPEED)
        follow_lane(self.env, car, sensors["bottom"], self.config.LANE_EDGE)

    def _resume(self, car, now):
        self.wait = None
        self.cooldown_until = now + self.config.STOP_LINE["cooldown_s"]
        car.drive(self.config.DRIVE_SPEED)

    def stop_line_detected(self, bottom_frame):
        """(1단계) 하단 ROI에서 가로로 긴 흰색 밴드 검출.

        흰색(저채도·고명도) 마스크 → 행별 흰 픽셀 비율 → row_fill 이상인
        행이 min_rows 연속이면 정지선. 세로 차선은 행 채움비가 낮고,
        횡단보도(진행방향 줄무늬)는 폭 점유가 ~60%라 row_fill 0.7을 못 넘는다.
        """
        if cv2 is None or bottom_frame is None:
            return False
        try:
            sl = self.config.STOP_LINE
            h, w = bottom_frame.shape[:2]
            roi = bottom_frame[int(h * sl["roi_top"]):, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 0, sl["v_min"]), (179, sl["s_max"], 255))
            row_frac = mask.sum(axis=1) / (255.0 * w)
            run = 0
            for f in row_frac:
                run = run + 1 if f >= sl["row_fill"] else 0
                if run >= sl["min_rows"]:
                    return True
            return False
        except Exception as e:
            print(f"[traffic] 정지선 감지 실패, 이번 프레임 스킵: {e}")
            return False
