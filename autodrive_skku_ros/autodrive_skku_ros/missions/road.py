import time

try:
    import cv2
except ImportError:
    cv2 = None

from .base import Mission, traveled_m
from .lane_follow import LaneCenterTracker, follow_lane_poi, LANE_POI
from .. import config as _config
from ..nodes.lidar_node import side_clearance_m


# ---------------- 튜닝 파라미터 ----------------
# 전방 장애물(흰색 장애물 차량) 카메라 감지 — road 미션 ④.
# 대회 규격: 장애물 차량·정지선·실선/점선 모두 흰색 → 형태로 구분한다.
# (차선=가늘고 세로로 김, 정지선=가로로 얇은 밴드, 장애물=폭·높이 모두 큰 블롭)
# 흰색 임계는 config.WHITE_HSV(단일 소스) 사용 — 장애물 차량 도색이 페인트
# 차선과 다르게 찍히면 white_s_max/white_v_min override로 개별 조정.
OBSTACLE_CAM = dict(
    white_s_max=None, white_v_min=None,  # None=config.WHITE_HSV 공유값 사용
    roi_top=0.35, roi_bottom=0.95,   # bottom 프레임 세로 ROI (비율)
    col_lo=0.20, col_hi=0.80,   # 중앙 컬럼 밴드 — 우리 차선의 장애물만
    min_area_ratio=0.04,        # ROI 면적 대비 블롭 면적비 임계
    min_w_ratio=0.15,           # ROI 폭 대비 블롭 폭 (차선은 이보다 가늚)
    min_h_ratio=0.25,           # ROI 높이 대비 블롭 높이 (정지선은 이보다 낮음)
    min_fill=0.45,              # bbox 채움비 — 대각선 차선은 희박해서 탈락
)

# 차선 변경 기동 (road 미션 ③④) — 펄스(120ms)↔조향각 매핑 미측정, 전부 실차 튜닝 대상.
# 근거: 조향 ±20도 → 회전반경 L/tan20 ≈ 1.5m, 차선폭 0.85m → S자 각 구간 헤딩 ~40도.
# 펄스 간격은 config.STEER_PULSE_GAP_S(단일 소스, t_parking과 공유) 사용.
LANE_CHANGE = dict(
    pulses=4,          # 진입/복귀 조향 펄스 횟수
    out_s=1.5,         # 옆 차선으로 나가는 구간 지속 시간
    back_s=1.5,        # 반대 조향으로 차선 정렬하는 구간
    straight_s=0.8,    # 직진 안정화 구간
    speed=80,          # 기동 중 속도
    cooldown_s=2.0,    # 기동 후 재트리거 억제 시간
    # 오도메트리 기반 거리 조건 (선택): out_m/back_m을 설정하면 pose_conf가
    # min_pose_conf 이상일 때 해당 구간을 "이동 거리 도달"로도 끝낼 수 있다 —
    # 속도/배터리 변화에 덜 민감. 타이밍(out_s/back_s)은 항상 안전 상한으로
    # 남는다. None(기본)이면 순수 타이밍 동작 = 기존과 완전히 동일.
    out_m=None,
    back_m=None,
    min_pose_conf=0.3,
)


def detect_obstacle_ahead(frame, cam_cfg, debug=None):
    """bottom 프레임 중앙 ROI에서 흰색 장애물 차량 블롭 감지. True/False.

    대회 규격상 장애물 차량·정지선·차선이 전부 흰색이라 색이 아닌 "형태"로
    구분한다: 차선은 가늘고(폭 작음) 세로로 길며, 정지선은 가로로 얇은 밴드
    (높이 작음), 장애물 차량은 폭·높이가 모두 크고 bbox를 촘촘히 채우는
    블롭이다. 대각선으로 걸친 차선은 bbox가 커도 채움비(min_fill)에서 탈락.

    debug: dict를 넘기면 ROI 사각형/블롭 목록(합격 여부 포함)/판정을 채운다
    (debug_viz.draw_obstacle 오버레이용). 반환값/판정 로직은 불변.
    """
    if cv2 is None or frame is None:
        return False
    try:
        s_max, v_min = _config.white_hsv(cam_cfg)
        h, w = frame.shape[:2]
        y0, y1 = int(h * cam_cfg["roi_top"]), int(h * cam_cfg["roi_bottom"])
        x0, x1 = int(w * cam_cfg["col_lo"]), int(w * cam_cfg["col_hi"])
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, v_min), (179, s_max, 255))
        rh, rw = mask.shape[:2]
        num, _labels, stats, _cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        found = False
        blobs = []  # (x, y, w, h, 합격 여부) — 프레임 절대좌표
        for i in range(1, num):
            bx, by, bw, bh, area = stats[i]
            passed = not (bw < cam_cfg["min_w_ratio"] * rw or bh < cam_cfg["min_h_ratio"] * rh
                          or area < cam_cfg["min_area_ratio"] * rh * rw
                          or area / float(bw * bh) < cam_cfg["min_fill"])
            blobs.append((int(bx + x0), int(by + y0), int(bw), int(bh), passed))
            if passed:
                found = True
                if debug is None:
                    return True  # 오버레이 불필요 시 기존처럼 조기 종료
        if debug is not None:
            debug.update(roi=(x0, y0, x1, y1), blobs=blobs, result=found)
        return found
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
        self.config = config
        self.debug = {}
        self._lane_tracker = LaneCenterTracker()
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
            self._lane_change_tick(sensors, car, now)
            return

        # (4) 카메라로 전방 흰색 장애물 감지 → 라이다 측면 여유로 방향 결정
        obs_dbg = {}
        if now >= self._cooldown_until and \
                detect_obstacle_ahead(sensors.get("bottom"), OBSTACLE_CAM, debug=obs_dbg):
            self.debug["obstacle"] = obs_dbg
            self.lane_change(car, self.pick_avoid_direction(sensors.get("lidar_scan")))
            self._lane_change_tick(sensors, car, now)
            return
        self.debug["obstacle"] = obs_dbg

        # (2) 차선 인식 주행 — POI 사다리꼴 다단 밴드 우측차선 추종 (2026-07-16 적용,
        # 기존 vendor edge_detection은 traffic.py에서 계속 씀)
        self.debug["lane_poi"] = follow_lane_poi(
            self._lane_tracker, car, sensors.get("bottom"), LANE_POI)

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
        lc = LANE_CHANGE
        self._lc_dir = direction
        self._lc_phase = "OUT"
        self._lc_t0 = self._now()
        self._lc_pulses = 0
        self._lc_last_pulse = 0.0
        self._lc_pose0 = None  # 페이즈 시작 pose — 첫 틱에 신뢰 가능하면 기록
        car.drive(lc["speed"])

    def _lc_enter(self, phase, now):
        self._lc_phase = phase
        self._lc_t0 = now
        self._lc_pulses = 0
        self._lc_last_pulse = 0.0
        self._lc_pose0 = None

    def _lc_pulse(self, car, direction, target, now):
        if self._lc_pulses < target and \
                now - self._lc_last_pulse >= _config.STEER_PULSE_GAP_S:
            car.steer_pulse(direction)
            self._lc_pulses += 1
            self._lc_last_pulse = now

    def _lc_phase_done(self, in_phase, dist_key, time_key, sensors, lc):
        """페이즈 종료 판정: 타이밍(항상 상한) OR 오도메트리 거리 도달.
        거리 조건은 dist_key가 설정돼 있고 pose_conf가 임계 이상일 때만 —
        conf=0(미보정)이면 정확히 기존 타이밍 동작이다(fail-inert)."""
        if in_phase >= lc[time_key]:
            return True
        dist_target = lc.get(dist_key)
        if dist_target and sensors.get("pose_conf", 0.0) >= lc["min_pose_conf"]:
            d = traveled_m(self._lc_pose0, sensors.get("pose"))
            if d is not None and d >= dist_target:
                return True
        return False

    def _lane_change_tick(self, sensors, car, now):
        lc = LANE_CHANGE
        opposite = "R" if self._lc_dir == "L" else "L"
        in_phase = now - self._lc_t0

        # 페이즈 시작 기준 pose — 신뢰 가능해진 첫 틱에 기록
        if self._lc_pose0 is None and sensors.get("pose") is not None and \
                sensors.get("pose_conf", 0.0) >= lc["min_pose_conf"]:
            self._lc_pose0 = sensors["pose"]

        if self._lc_phase == "OUT":
            self._lc_pulse(car, self._lc_dir, lc["pulses"], now)
            if self._lc_phase_done(in_phase, "out_m", "out_s", sensors, lc):
                self._lc_enter("BACK", now)
        elif self._lc_phase == "BACK":
            # 2배 펄스: 진입 조향을 지나 반대 lock까지 스윙해 차선에 맞춘다
            self._lc_pulse(car, opposite, 2 * lc["pulses"], now)
            if self._lc_phase_done(in_phase, "back_m", "back_s", sensors, lc):
                self._lc_enter("STRAIGHT", now)
        elif self._lc_phase == "STRAIGHT":
            self._lc_pulse(car, self._lc_dir, lc["pulses"], now)
            if in_phase >= lc["straight_s"]:
                car.steer("F")
                car.drive(self.config.DRIVE_SPEED)
                self._lc_phase = None
                self._cooldown_until = now + lc["cooldown_s"]
