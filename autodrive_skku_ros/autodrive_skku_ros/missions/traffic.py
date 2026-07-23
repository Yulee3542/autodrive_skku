import time

try:
    import cv2
except ImportError:
    cv2 = None

from .base import Mission
from .lane_follow import LaneCenterTracker, follow_lane_poi, LANE_POI
from . import traffic_yolo
from .. import config as _config

# vendor 미설치 개발 환경용 폴백 상수 복사본. vendor 파일은 수정 금지(대회 규정)라
# 자동 동기화 불가 — tools/smoke_test_missions.py의 test_vendor_fallback_sync가
# vendor import 가능 환경에서 이 값과 vendor 값의 일치를 검증한다.
_FALLBACK = dict(RED=0, GREEN=1, HUE_THRESHOLD=([4, 176], [40, 80]), SATURATION=150)

try:
    from ..vendor import Function_Library as fl
    from ..vendor.Function_Library import HUE_THRESHOLD, SATURATION, RED, GREEN
except ImportError:  # 패키지 미설치 개발 환경 — 검증된 상수값만 복사해 사용
    fl = None
    RED, GREEN = _FALLBACK["RED"], _FALLBACK["GREEN"]
    HUE_THRESHOLD = _FALLBACK["HUE_THRESHOLD"]
    SATURATION = _FALLBACK["SATURATION"]


# ---------------- 튜닝 파라미터 ----------------
# 신호등 판정: 상단 프레임에서 해당 색 픽셀이 이 비율을 넘어야 인식
TRAFFIC_PIXEL_RATIO = 0.005

# 정지선(흰색) 인식 (traffic 미션 ①). row_fill 0.7: 횡단보도(진행방향 줄무늬,
# 폭 점유 ~60%)와 세로 차선이 행 채움비를 못 넘게 하는 값.
# 흰색 임계는 config.WHITE_HSV(단일 소스) 사용 — 정지선만 다르게 찍히면
# white_s_max/white_v_min override로 개별 조정.
STOP_LINE = dict(
    white_s_max=None, white_v_min=None,  # None=config.WHITE_HSV 공유값 사용
    roi_top=0.55,          # bottom 프레임에서 이 비율 아래 행만 검사 (가까운 노면)
    row_fill=0.7,          # 행 폭 대비 흰 픽셀 비율 임계
    min_rows=6,            # 연속으로 임계를 넘어야 하는 행 수
    wait_max_s=6.0,        # 정지선 대기 중 신호등 미검출 시 재출발까지 시간 (교착 방지)
    cooldown_s=3.0,        # 재출발 후 같은 정지선 재트리거 억제
)

# 신호등 색 판정: YOLO(1순위) + HSV(폴백, detect_light_color) 이중 검출 +
# 프레임 연속 판정(debounce). red/green 비대칭 — 빨간불 오인은 손해가 없지만
# 초록불 오인(=빨간불 출발)은 규정 감점이라, 초록 확정에 더 많은 연속 프레임을
# 요구한다(팀 저장소 ParkGaYoung/Haebin/yeoeun_traffic 구현 공통 설계).
TRAFFIC_LIGHT = dict(
    use_yolo=True,                              # False면 HSV(detect_light_color)만 사용
    model_path=traffic_yolo.DEFAULT_MODEL_PATH,  # 문자열 기본값 — tuning.py의 None
                                                  # 슬롯(NONE_SENTINEL/float 전용)과
                                                  # 섞이면 안 되므로 실제 경로를 기본값으로 둔다
    conf=0.35,                   # YOLO confidence 임계
    red_confirm_n=2,             # 연속 몇 프레임 'red'여야 정지 상태 진입
    green_confirm_n=3,           # 연속 몇 프레임 'green'이어야 재출발 (비대칭: red보다 엄격)
)


def detect_light_color(frame, min_ratio=0.005, debug=None):
    """상단 프레임에서 빨강/초록 픽셀 비율로 신호등 판정. 'red'/'green'/None.

    검증된 HUE_THRESHOLD/SATURATION 값을 그대로 사용한다. 디스플레이가 있는
    환경에서는 fl.libCAMERA().object_detection(원 검출 방식)으로 교체 가능.

    debug: dict를 넘기면 픽셀 카운트/임계/판정을 채운다
    (debug_viz.draw_traffic_light 오버레이용). 반환값/판정 로직은 불변.
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
        result = "red"
    elif green >= min_pixels and green > red * 2:
        result = "green"
    else:
        result = None
    if debug is not None:
        debug.update(red=int(red), green=int(green),
                     min_pixels=int(min_pixels), result=result)
    return result


class TrafficMission(Mission):
    """2. 신호등 주행

    목표:
      (1) 정지선 인식      — 테스트 구현 (흰색 가로 밴드, 행 채움비 판정)
      (2) 신호등 라이트 인식 — 동작 (YOLO 1순위 + HSV 픽셀 비율 폴백)

    동작: 차선 추종 주행 중 정지선을 만나면 정지, 초록불이면 다시 출발.
    빨간불은 언제든 정지 (main3 검증 로직과 동일).

    신호등 색 판정 (TRAFFIC_LIGHT 튜닝값, `_detect_light_color` 참고):
      YOLO(1순위, `traffic_yolo.detect_light_color_yolo`)가 이번 프레임에 확정
      결과(red/green)를 못 주면(미검출/모델 없음/추론 실패) HSV(`detect_light_color`)
      로 그 프레임만 폴백한다. 팀 저장소(yeoeun_traffic/ParkGaYoung/Haebin)
      구현들과 동일하게 이중 검출 구조.
      또한 단일 프레임 오검출을 막기 위해 연속 프레임 판정(debounce)을 쓴다 —
      `red_confirm_n`번 연속 red여야 정지 상태에 진입하고, `green_confirm_n`번
      연속 green이어야(red보다 엄격 — 비대칭) 재출발한다. 빨간불에서 잘못
      출발하면 규정 감점이지만 초록불에서 늦게 출발하는 건 손해가 없기 때문.

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
        self.config = config
        self.debug = {}
        self._lane_tracker = LaneCenterTracker()
        self._now = time.monotonic  # 테스트에서 가짜 시계 주입 지점
        self.wait = None            # None | "line" | "red"
        self.wait_t0 = 0.0
        self.cooldown_until = 0.0
        self._red_streak = 0
        self._green_streak = 0
        self._yolo_model = (traffic_yolo.load_model(TRAFFIC_LIGHT["model_path"])
                            if TRAFFIC_LIGHT["use_yolo"] else None)
        car.go()
        car.drive(config.DRIVE_SPEED)

    def _detect_light_color(self, frame, debug=None):
        """YOLO(1순위) → 확정 결과 없으면 HSV(폴백)로 그 프레임만 재판정."""
        if self._yolo_model is not None:
            yolo_dbg = {}
            result = traffic_yolo.detect_light_color_yolo(
                self._yolo_model, frame, conf=TRAFFIC_LIGHT["conf"], debug=yolo_dbg)
            if result is not None:
                if debug is not None:
                    debug.update(yolo_dbg)
                return result
        hsv_dbg = {}
        result = detect_light_color(frame, TRAFFIC_PIXEL_RATIO, debug=hsv_dbg)
        hsv_dbg["source"] = "hsv"
        if debug is not None:
            debug.update(hsv_dbg)
        return result

    def step(self, sensors, car):
        now = self._now()
        light_dbg = {}
        raw_color = self._detect_light_color(sensors.get("top"), debug=light_dbg)
        self.debug["traffic_light"] = light_dbg

        self._red_streak = self._red_streak + 1 if raw_color == "red" else 0
        self._green_streak = self._green_streak + 1 if raw_color == "green" else 0
        color = "red" if self._red_streak >= TRAFFIC_LIGHT["red_confirm_n"] else \
            "green" if self._green_streak >= TRAFFIC_LIGHT["green_confirm_n"] else None

        if color == "red" and self.wait != "red":
            self.wait = "red"
            self.wait_t0 = now

        if self.wait is not None:
            car.drive(0)
            if color == "green":
                self._resume(car, now)
            elif self.wait == "line" and color is None and \
                    now - self.wait_t0 >= STOP_LINE["wait_max_s"]:
                print("[traffic] 신호등 미검출 — 대기 시간 초과, 재출발")
                self._resume(car, now)
            return

        stop_dbg = {}
        stop_hit = self.stop_line_detected(sensors.get("bottom"), debug=stop_dbg)
        self.debug["stop_line"] = stop_dbg
        if now >= self.cooldown_until and stop_hit:
            self.wait = "line"
            self.wait_t0 = now
            car.drive(0)
            return

        # follow_lane_poi가 곡률에 따라 속도를 다시 내리므로(감속) 여기 drive는
        # "프레임이 없어 follow_lane_poi가 조기 반환할 때"의 기본값 역할이다.
        car.drive(self.config.DRIVE_SPEED)
        self.debug["lane_poi"] = follow_lane_poi(
            self._lane_tracker, car, sensors.get("bottom"), LANE_POI, now=self._now)

    def _resume(self, car, now):
        self.wait = None
        self.cooldown_until = now + STOP_LINE["cooldown_s"]
        car.drive(self.config.DRIVE_SPEED)

    def stop_line_detected(self, bottom_frame, debug=None):
        """(1단계) 하단 ROI에서 가로로 긴 흰색 밴드 검출.

        흰색(저채도·고명도) 마스크 → 행별 흰 픽셀 비율 → row_fill 이상인
        행이 min_rows 연속이면 정지선. 세로 차선은 행 채움비가 낮고,
        횡단보도(진행방향 줄무늬)는 폭 점유가 ~60%라 row_fill 0.7을 못 넘는다.

        debug: dict를 넘기면 행 채움비/ROI/판정을 채운다
        (debug_viz.draw_stop_line 오버레이용). 반환값/판정 로직은 불변.
        """
        if cv2 is None or bottom_frame is None:
            return False
        try:
            sl = STOP_LINE
            s_max, v_min = _config.white_hsv(sl)
            h, w = bottom_frame.shape[:2]
            roi_y0 = int(h * sl["roi_top"])
            roi = bottom_frame[roi_y0:, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 0, v_min), (179, s_max, 255))
            row_frac = mask.sum(axis=1) / (255.0 * w)
            found = False
            run = 0
            for f in row_frac:
                run = run + 1 if f >= sl["row_fill"] else 0
                if run >= sl["min_rows"]:
                    found = True
                    if debug is None:
                        return True  # 오버레이 불필요 시 기존처럼 조기 종료
                    break
            if debug is not None:
                debug.update(roi_y0=roi_y0, row_frac=row_frac,
                             row_fill=sl["row_fill"], min_rows=sl["min_rows"],
                             result=found)
            return found
        except Exception as e:
            print(f"[traffic] 정지선 감지 실패, 이번 프레임 스킵: {e}")
            return False
