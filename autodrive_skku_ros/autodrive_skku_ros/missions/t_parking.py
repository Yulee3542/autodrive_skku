import math
import time
from collections import deque

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from .base import Mission
from ..nodes.lidar_node import filter_self, rear_min_m


class TParkingMission(Mission):
    """3. T 주차

    목표 (전부 테스트 구현 — 타이밍/임계값은 실차 튜닝 대상):
      (1) 라이다 기반 맵 빌딩          — 스캔 누적 (map_scans회)
      (2) 후방 카메라 기반 주차선 인식  — 흰 주차선 2줄 컬럼 히스토그램
      (3) 차선 인식 도로 주행 (후진)    — reverse_lane_steer (후진 조향 반전)
      (4) T주차 알고리즘에 따른 주차    — PARK 서브 페이즈 기동 시퀀스

    상태머신: MAP_BUILD → FIND_SLOT → REVERSE_ALIGN → PARK → DONE
    PARK 내부 서브 페이즈: TURN_IN → STRAIGHTEN → CREEP → HOLD
    오도메트리가 없어 라이다 스캔 + 후방 카메라 + 타이머만으로 동작한다.

    라이다는 후방 장착(0도=차량 후방)이라 후진 방향이 정확히 주 시야다 —
    슬롯 탐지(측면 갭)와 주차 완료(후방 거리) 판정 모두 라이다 담당.
    출차(pull-out)는 이 상태머신 범위 외 — 후속 과제 (시뮬 parking_mission의
    EXIT 페이즈 프로토타입 참고).
    """

    name = "t_parking"

    def on_start(self, car, config):
        self.config = config
        self.p = config.T_PARKING
        self._now = time.monotonic  # 테스트에서 가짜 시계 주입 지점
        self.state = "MAP_BUILD"
        self.scans = deque(maxlen=self.p["map_scans"])  # 맵 빌딩용 라이다 스캔 누적
        self._slot = None           # (bearing_deg, dist_m) — slot_found가 기록
        self._last_err_px = None    # reverse_lane_steer가 기록하는 주차선 중점 오차
        self._align_count = 0
        self._parked_count = 0
        self._park_phase = None     # PARK 서브 페이즈
        self._park_t0 = 0.0
        self._park_pulses = 0
        self._park_last_pulse = 0.0
        car.go()

    def step(self, sensors, car):
        if self.state == "MAP_BUILD":
            car.drive(self.config.SLOW_SPEED)
            car.steer("F")
            if sensors["lidar_scan"] is not None:
                self.map_update(sensors["lidar_scan"])
            if self.map_complete():
                self.state = "FIND_SLOT"

        elif self.state == "FIND_SLOT":
            car.drive(0)
            if self.slot_found(sensors):
                self.state = "REVERSE_ALIGN"

        elif self.state == "REVERSE_ALIGN":
            car.drive(-self.config.SLOW_SPEED)
            steer = self.reverse_lane_steer(sensors["rear"])
            if steer is not None:
                car.steer(steer)
            if self.aligned(sensors):
                self.state = "PARK"

        elif self.state == "PARK":
            self._park_tick(sensors, car)

        else:  # DONE
            car.stop()

    # ---- (4단계) PARK 기동 시퀀스 — 서브 페이즈 머신 ----

    def _park_enter(self, phase, now):
        self._park_phase = phase
        self._park_t0 = now
        self._park_pulses = 0
        self._park_last_pulse = 0.0

    def _park_pulse(self, car, direction, target, now):
        # 펄스 간격은 차선 변경과 동일한 값 공유 (steer_pulse 반복 전송 주기)
        gap = self.config.LANE_CHANGE["pulse_gap_s"]
        if self._park_pulses < target and now - self._park_last_pulse >= gap:
            car.steer_pulse(direction)
            self._park_pulses += 1
            self._park_last_pulse = now

    def _park_tick(self, sensors, car):
        now = self._now()
        if self._park_phase is None:
            self._park_enter("TURN_IN", now)
        in_phase = now - self._park_t0
        # 슬롯 쪽으로 "후진" 진입 — 후진 시 차체 뒤가 조향 반대쪽으로 돈다
        turn_dir = "L" if self.p["side"] == "R" else "R"
        counter = "R" if turn_dir == "L" else "L"

        if self._park_phase == "TURN_IN":
            car.drive(-self.config.SLOW_SPEED)
            self._park_pulse(car, turn_dir, self.p["turn_in_pulses"], now)
            if in_phase >= self.p["turn_in_s"]:
                self._park_enter("STRAIGHTEN", now)

        elif self._park_phase == "STRAIGHTEN":
            car.drive(-self.config.SLOW_SPEED)
            self._park_pulse(car, counter, self.p["turn_in_pulses"], now)
            if in_phase >= self.p["straighten_s"]:
                car.steer("F")
                self._park_enter("CREEP", now)

        elif self._park_phase == "CREEP":
            car.drive(-self.config.SLOW_SPEED)
            steer = self.reverse_lane_steer(sensors["rear"])
            if steer is not None:
                car.steer(steer)
            if self.parked(sensors) or in_phase >= self.p["park_max_s"]:
                car.drive(0)
                car.steer("F")
                self._park_enter("HOLD", now)

        elif self._park_phase == "HOLD":
            car.drive(0)  # 주차 완료 정지 유지 (규정 3~5초)
            if in_phase >= self.p["hold_s"]:
                self.state = "DONE"

    # ---- 판정 로직 ----

    def map_update(self, scan):
        """(1단계) 스캔 누적. deque(maxlen=map_scans)라 최신 스캔만 유지된다.

        오도메트리가 없어 점유 격자 정합은 불가 — 저속 직진 가정으로
        "최근 N회 스캔"을 맵으로 취급한다.
        """
        self.scans.append(scan)

    def map_complete(self):
        """(1단계) 스캔이 map_scans회 쌓이면 맵 빌딩 완료."""
        return len(self.scans) >= self.p["map_scans"]

    def slot_found(self, sensors):
        """측면 근접 클러스터(주차 차량) 사이의 갭으로 T주차 슬롯 판정.

        후방 0도 라이다의 측면 섹터(전방 기준 75~165도, side 방향)에서
        slot_max_lateral_m 이내 점만 취해 bearing 순으로 정렬, 이웃 점 사이
        chord 폭이 slot_gap_min_m 이상이면 두 주차 차량 사이 갭이다.
        """
        scan = sensors["lidar_scan"]
        if not scan:
            return False
        cfg = self.config
        sign = 1.0 if self.p["side"] == "L" else -1.0
        pts = [(b, d / 1000.0)
               for b, d in filter_self(scan, cfg.LIDAR_MOUNT, cfg.LIDAR_SELF_MASK_DEG)
               if 75.0 <= sign * b <= 165.0 and d / 1000.0 <= self.p["slot_max_lateral_m"]]
        if len(pts) < 2:
            return False
        pts.sort()
        for (b1, d1), (b2, d2) in zip(pts, pts[1:]):
            db = math.radians(abs(b2 - b1))
            chord = math.sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * math.cos(db))
            if chord >= self.p["slot_gap_min_m"]:
                self._slot = ((b1 + b2) / 2.0, (d1 + d2) / 2.0)
                return True
        return False

    def reverse_lane_steer(self, rear_frame):
        """(2,3단계) 후방 카메라 주차선 인식 → 'F'/'L'/'R'. 실패 시 None.

        하단 절반 ROI에서 흰색(STOP_LINE과 같은 임계) 컬럼 히스토그램을 만들어
        주차선 2줄의 컬럼 클러스터를 찾고, 두 줄 중점과 화면 중앙의 오차로
        조향을 정한다. "후진" 시 차체 뒤가 조향 반대쪽으로 돌아가므로
        전진 기준 방향을 구한 뒤 L↔R을 반전해 반환한다.
        """
        if cv2 is None or rear_frame is None:
            return None
        try:
            sl = self.config.STOP_LINE  # 흰색 임계 공유 (s_max/v_min)
            h, w = rear_frame.shape[:2]
            roi = rear_frame[h // 2:, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 0, sl["v_min"]), (179, sl["s_max"], 255))
            col_sum = mask.sum(axis=0).astype(float)
            if col_sum.max() <= 0:
                self._last_err_px = None
                return None
            cols = np.where(col_sum > 0.25 * col_sum.max())[0]
            # 10px 이상 벌어지면 다른 클러스터(다른 주차선)
            clusters = []
            start = prev = int(cols[0])
            for c in cols[1:]:
                c = int(c)
                if c - prev > 10:
                    clusters.append((start + prev) / 2.0)
                    start = c
                prev = c
            clusters.append((start + prev) / 2.0)
            if len(clusters) < 2:
                self._last_err_px = None
                return None  # 주차선 2줄이 다 보여야 정렬 가능
            mid = (clusters[0] + clusters[-1]) / 2.0
            err = mid - w / 2.0
            self._last_err_px = err
            if abs(err) <= self.p["align_tol_px"]:
                return "F"
            forward = "R" if err > 0 else "L"          # 전진 기준 목표 방향
            return "L" if forward == "R" else "R"       # 후진 조향 반전
        except Exception as e:
            print(f"[t_parking] 주차선 인식 실패, 이번 프레임 스킵: {e}")
            self._last_err_px = None
            return None

    def aligned(self, sensors):
        """슬롯 진입 정렬 판정 — 주차선 중점 오차가 연속 align_ticks 틱 허용치 내."""
        if self._last_err_px is None or abs(self._last_err_px) > self.p["align_tol_px"]:
            self._align_count = 0
            return False
        self._align_count += 1
        return self._align_count >= self.p["align_ticks"]

    def parked(self, sensors):
        """주차 완료 판정 — 후방(뒤 범퍼 기준) 거리가 연속 3틱 rear_stop_m 이내."""
        cfg = self.config
        d = rear_min_m(sensors["lidar_scan"], cfg.LIDAR_MOUNT,
                       cfg.LIDAR_REAR_SECTOR, cfg.LIDAR_SELF_MASK_DEG) \
            if sensors["lidar_scan"] else None
        if d is not None and d <= self.p["rear_stop_m"]:
            self._parked_count += 1
        else:
            self._parked_count = 0
        return self._parked_count >= 3
