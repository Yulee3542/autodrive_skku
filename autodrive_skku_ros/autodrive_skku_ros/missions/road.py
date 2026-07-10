import time

try:
    import cv2
except ImportError:
    cv2 = None

from .base import Mission
from .lane_follow import follow_lane
from ..nodes.lidar_node import side_clearance_m

try:
    from ..vendor import Function_Library as fl
except ImportError:  # 패키지 미설치 개발 환경 — 차선 인식 없이 골격만 동작
    fl = None


def detect_obstacle_ahead(frame, cam_cfg):
    """bottom 프레임 중앙 ROI에서 흰색 장애물 차량 블롭 감지. True/False.

    대회 규격상 장애물 차량·정지선·차선이 전부 흰색이라 색이 아닌 "형태"로
    구분한다: 차선은 가늘고(폭 작음) 세로로 길며, 정지선은 가로로 얇은 밴드
    (높이 작음), 장애물 차량은 폭·높이가 모두 크고 bbox를 촘촘히 채우는
    블롭이다. 대각선으로 걸친 차선은 bbox가 커도 채움비(min_fill)에서 탈락.
    """
    if cv2 is None or frame is None:
        return False
    try:
        h, w = frame.shape[:2]
        y0, y1 = int(h * cam_cfg["roi_top"]), int(h * cam_cfg["roi_bottom"])
        x0, x1 = int(w * cam_cfg["col_lo"]), int(w * cam_cfg["col_hi"])
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, cam_cfg["v_min"]), (179, cam_cfg["s_max"], 255))
        rh, rw = mask.shape[:2]
        num, _labels, stats, _cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for i in range(1, num):
            _bx, _by, bw, bh, area = stats[i]
            if bw < cam_cfg["min_w_ratio"] * rw or bh < cam_cfg["min_h_ratio"] * rh:
                continue  # 가늘거나(차선) 낮음(정지선)
            if area < cam_cfg["min_area_ratio"] * rh * rw:
                continue
            if area / float(bw * bh) < cam_cfg["min_fill"]:
                continue  # bbox가 희박 — 대각선 차선
            return True
        return False
    except Exception as e:
        print(f"[road] 장애물 감지 실패, 이번 프레임 스킵: {e}")
        return False


class RoadMission(Mission):
    """1. 도로 주행

    단계별 목표:
      (1) 직진, 스티어링          — 동작
      (2) 차선 인식 도로 주행      — 동작 (팀 검증 edge_detection 사용, lane_follow.py 공유)
      (3) 차선 변경하며 도로 주행  — 테스트 구현 (논블로킹 페이즈 머신, 타이밍은 실차 튜닝 대상)
      (4) 장애물 피해 차선 변경    — 테스트 구현 (전방=카메라 흰 블롭, 방향=라이다 측면 여유)

    라이다가 후방 장착(0도=차량 후방)이라 전방은 자차 차체에 막힘 —
    전방 장애물은 detect_obstacle_ahead(카메라)로 감지하고, 회피 방향만
    라이다 측면(abeam) 여유 비교로 정한다.
    """

    name = "road"

    def on_start(self, car, config):
        assert set(config.LANE_EDGE) == {"width", "height", "gap", "threshold"}, \
            f"config.LANE_EDGE 키가 예상과 다름: {set(config.LANE_EDGE)}"
        self.config = config
        self.env = fl.libCAMERA() if fl is not None else None
        self._now = time.monotonic  # 테스트에서 가짜 시계 주입 지점
        self._lc_phase = None       # None이면 기동 중 아님
        self._lc_dir = "L"
        self._lc_t0 = 0.0
        self._lc_pulses = 0
        self._lc_last_pulse = 0.0
        self._cooldown_until = 0.0
        car.go()
        car.drive(config.DRIVE_SPEED)

    def step(self, sensors, car):
        now = self._now()

        # (3) 차선 변경 기동 중이면 페이즈 머신만 진행
        if self._lc_phase is not None:
            self._lane_change_tick(car, now)
            return

        # (4) 카메라로 전방 흰색 장애물 감지 → 라이다 측면 여유로 방향 결정
        if now >= self._cooldown_until and \
                detect_obstacle_ahead(sensors["bottom"], self.config.OBSTACLE_CAM):
            self.lane_change(car, self.pick_avoid_direction(sensors["lidar_scan"]))
            self._lane_change_tick(car, now)
            return

        # (2) 차선 인식 주행 — 검증된 파라미터는 config.LANE_EDGE
        follow_lane(self.env, car, sensors["bottom"], self.config.LANE_EDGE)

    def pick_avoid_direction(self, scan):
        """라이다 측면 여유 비교로 회피 방향 결정. 반사 없음(None)=완전히 빈 쪽."""
        cfg = self.config
        left = side_clearance_m(scan, "L", cfg.LIDAR_MOUNT,
                                cfg.LIDAR_SIDE_WINDOW_DEG, cfg.LIDAR_SELF_MASK_DEG)
        right = side_clearance_m(scan, "R", cfg.LIDAR_MOUNT,
                                 cfg.LIDAR_SIDE_WINDOW_DEG, cfg.LIDAR_SELF_MASK_DEG)
        if left is None:
            return "L"  # 스캔 자체가 없거나 좌측이 완전히 빔 → 기본 좌측
        if right is None:
            return "R"
        return "L" if left >= right else "R"

    def lane_change(self, car, direction):
        """(3단계) 차선 변경 기동 시작 — 이후 step()이 틱마다 진행한다.

        블로킹 sleep 없이 페이즈 머신으로 구현: 메인 루프가 계속 돌아야
        센서/아두이노 keepalive가 유지된다.
          OUT      : direction 쪽 조향 펄스 × pulses, out_s 동안 옆 차선 진입
          BACK     : 반대 조향 펄스 × 2·pulses, back_s 동안 차선 정렬
          STRAIGHT : direction 쪽 펄스 × pulses로 중립 복귀, straight_s 후 종료
        """
        lc = self.config.LANE_CHANGE
        self._lc_dir = direction
        self._lc_phase = "OUT"
        self._lc_t0 = self._now()
        self._lc_pulses = 0
        self._lc_last_pulse = 0.0
        car.drive(lc["speed"])

    def _lc_enter(self, phase, now):
        self._lc_phase = phase
        self._lc_t0 = now
        self._lc_pulses = 0
        self._lc_last_pulse = 0.0

    def _lc_pulse(self, car, direction, target, now):
        lc = self.config.LANE_CHANGE
        if self._lc_pulses < target and now - self._lc_last_pulse >= lc["pulse_gap_s"]:
            car.steer_pulse(direction)
            self._lc_pulses += 1
            self._lc_last_pulse = now

    def _lane_change_tick(self, car, now):
        lc = self.config.LANE_CHANGE
        opposite = "R" if self._lc_dir == "L" else "L"
        in_phase = now - self._lc_t0

        if self._lc_phase == "OUT":
            self._lc_pulse(car, self._lc_dir, lc["pulses"], now)
            if in_phase >= lc["out_s"]:
                self._lc_enter("BACK", now)
        elif self._lc_phase == "BACK":
            # 2배 펄스: 진입 조향을 지나 반대 lock까지 스윙해 차선에 맞춘다
            self._lc_pulse(car, opposite, 2 * lc["pulses"], now)
            if in_phase >= lc["back_s"]:
                self._lc_enter("STRAIGHT", now)
        elif self._lc_phase == "STRAIGHT":
            self._lc_pulse(car, self._lc_dir, lc["pulses"], now)
            if in_phase >= lc["straight_s"]:
                car.steer("F")
                car.drive(self.config.DRIVE_SPEED)
                self._lc_phase = None
                self._cooldown_until = now + lc["cooldown_s"]
