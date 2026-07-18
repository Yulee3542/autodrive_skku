import math
import time
from collections import deque

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from .base import Mission, traveled_m
from .lane_follow import LaneCenterTracker, follow_lane_poi
from .occupancy import OccupancyMap
from .. import config as _config
from .. import filters
from ..nodes.lidar_node import filter_self, rear_min_m


# ---------------- 튜닝 파라미터 ----------------
# T주차 (t_parking 미션) — 주차칸 950x1500mm(규정), 완료 후 3~5초 정지(규정) 기준.
# 흰색 임계는 config.WHITE_HSV(단일 소스), 펄스 간격은 config.STEER_PULSE_GAP_S 사용.
T_PARKING = dict(
    side="R",              # 주차 슬롯이 있는 쪽 ('L'/'R') — 당일 코스 확인 후 설정
    map_scans=30,          # MAP_BUILD에서 누적할 스캔 수
    slot_gap_min_m=0.60,   # 주차 차량 사이 갭 최소 폭 (슬롯 판정)
    slot_max_lateral_m=2.0,  # 슬롯 후보로 인정할 최대 측면 거리
    align_tol_px=25,       # 후방캠 주차선 중점 정렬 허용 오차 (px)
    align_ticks=5,         # 연속 정렬 판정 틱 수
    # ---- 주차선 중점 오차(reverse_lane_steer) 칼만필터 (2026-07-18,
    # lane_follow.LaneCenterTracker와 동일 패턴) — 한 프레임 주차선 미검출로
    # 바로 _last_err_px=None을 만들지 않고 predict-only로 버티게 한다.
    kf_process_noise=4.0,       # px^2/tick
    kf_measurement_noise=4.0,   # px^2 📏
    kf_max_variance_px=100.0,   # 이 이상 불확실해지면 추정 폐기(None 취급) 📏
    turn_in_pulses=4,      # PARK 진입 조향 펄스 수
    turn_in_s=2.0,         # 슬롯 방향 후진 회전 구간
    straighten_s=1.5,      # 반대 조향으로 차체 정렬 구간
    rear_stop_m=0.30,      # 후방 이 거리 이내면 주차 완료 (뒤 범퍼 기준)
    hold_s=4.0,            # 완료 후 정지 유지 (규정 3~5초)
    park_max_s=12.0,       # PARK 상태 안전 타임아웃
    white_s_max=None, white_v_min=None,  # 주차선 흰색 override (None=config.WHITE_HSV)
    min_pose_conf=0.3,     # 이 이상이어야 오도메트리 거리 기록/사용 (0=미보정, 비활성)
    # 점유 격자 (오도메트리 신뢰 가능할 때만 활성 — 미보정 시 deque 폴백)
    occ_size_m=8.0,        # 격자 한 변 (미션 시작 pose가 중앙)
    occ_res_m=0.05,        # 셀 크기
    occ_min_hits=2,        # 이 이상 히트여야 점유 확정 (1회 노이즈 걸러냄)
    # 출차 (규정: 3~5초 정지 후 출차해 반대편 OUT 통과 — 출차실패 f7 -30,
    # OUT 도착실패 f8 -40). 진입 기동을 전진으로 되짚는 타이밍 미러 + 파라미터.
    exit_enabled=True,     # False면 HOLD 후 DONE(정지)만 — 기존 동작
    exit_mode="lane",      # "lane"=출차 후 차선유지로 OUT까지 주행(규정 기본) | "stop"=출차 후 정지
    exit_creep_s=3.0,      # 진입 CREEP 실측 기록이 없을 때의 전진 이탈 시간 폴백
    exit_margin_s=0.5,     # 전진 이탈 시간 여유 (진입보다 살짝 더 나감)
    exit_margin_m=0.15,    # 전진 이탈 거리 여유 (오도메트리 신뢰 시, 시간 여유와 동일 취지)
    exit_turn_s=2.0,       # 진입 방향과 같은 조향으로 호를 되짚는 구간
    exit_straight_s=1.0,   # 반대 조향으로 차체 재정렬 구간
    exit_max_s=15.0,       # EXIT 전체 안전 타임아웃 (park_max_s 패턴)
)


class TParkingMission(Mission):
    """3. T 주차

    목표 (전부 테스트 구현 — 타이밍/임계값은 실차 튜닝 대상):
      (1) 라이다 기반 맵 빌딩          — 스캔 누적 + 점유 격자(오도메트리 가용 시)
      (2) 후방 카메라 기반 주차선 인식  — 흰 주차선 2줄 컬럼 히스토그램
      (3) 차선 인식 도로 주행 (후진)    — reverse_lane_steer (후진 조향 반전)
      (4) T주차 알고리즘에 따른 주차    — PARK 서브 페이즈 기동 시퀀스
      (5) 출차 후 OUT 통과            — EXIT 서브 페이즈 (규정 f7/f8)

    상태머신: MAP_BUILD → FIND_SLOT → REVERSE_ALIGN → PARK → EXIT → LANE_FOLLOW|DONE
      PARK 서브 페이즈: TURN_IN → STRAIGHTEN → CREEP → HOLD
      EXIT 서브 페이즈: EXIT_CREEP → EXIT_TURN → EXIT_STRAIGHT
    출차는 진입 기동을 전진으로 되짚는 타이밍 미러: 전진으로 같은 호를
    되짚으려면 진입 TURN_IN과 "같은 방향" 조향이 맞다 (후진 좌조향 호 =
    전진 좌조향 호). exit_mode="lane"이면 재정렬 후 차선유지로 OUT까지 주행
    (종료는 규정대로 운영자 키 입력), "stop"이면 정지(DONE).

    라이다는 후방 장착(0도=차량 후방)이라 후진 방향이 정확히 주 시야다 —
    슬롯 탐지(측면 갭)와 주차 완료(후방 거리) 판정 모두 라이다 담당.
    """

    name = "t_parking"

    def on_start(self, car, config):
        self.config = config
        self.debug = {}
        self.p = T_PARKING
        self._now = time.monotonic  # 테스트에서 가짜 시계 주입 지점
        self.state = "MAP_BUILD"
        self.scans = deque(maxlen=self.p["map_scans"])  # 맵 빌딩용 라이다 스캔 누적
        self.occ = None             # 점유 격자 — 오도메트리 신뢰 가능해지면 생성
        self._slot = None           # (bearing_deg, dist_m) — slot_found가 기록
        self._last_err_px = None    # reverse_lane_steer가 기록하는 주차선 중점 오차(칼만필터링됨)
        self._err_kf = filters.ScalarKalmanFilter()
        self._align_count = 0
        self._parked_count = 0
        self._park_phase = None     # PARK 서브 페이즈
        self._park_t0 = 0.0
        self._park_pulses = 0
        self._park_last_pulse = 0.0
        self._park_pose0 = None     # 서브 페이즈 시작 pose (오도메트리 가용 시)
        self._creep_s_actual = None  # CREEP 실제 소요 시간 — 출차 미러링 기준
        self._creep_m_actual = None  # CREEP 실제 이동 거리 (pose_conf 충족 시)
        self._exit_t_start = 0.0    # EXIT 전체 타임아웃 기준 시각
        self._lane_tracker = LaneCenterTracker()  # 출차 후 LANE_FOLLOW용
        car.go()

    def step(self, sensors, car):
        if self.state == "MAP_BUILD":
            car.drive(self.config.SLOW_SPEED)
            car.steer("F")
            if sensors.get("lidar_scan") is not None:
                self.map_update(sensors.get("lidar_scan"), sensors)
            if self.map_complete():
                self.state = "FIND_SLOT"

        elif self.state == "FIND_SLOT":
            car.drive(0)
            if self.slot_found(sensors):
                self.state = "REVERSE_ALIGN"

        elif self.state == "REVERSE_ALIGN":
            car.drive(-self.config.SLOW_SPEED)
            steer = self.reverse_lane_steer(sensors.get("rear"),
                                            debug=self.debug.setdefault("parking_line", {}))
            if steer is not None:
                car.steer(steer)
            if self.aligned(sensors):
                self.state = "PARK"

        elif self.state == "PARK":
            self._park_tick(sensors, car)

        elif self.state == "EXIT":
            self._exit_tick(sensors, car)

        elif self.state == "LANE_FOLLOW":
            # (5단계) 출차 완료 — 차선유지로 OUT 영역까지 주행. 규정상 종료는
            # 운영자 키 입력이므로 이 상태에 터미널 조건은 없다.
            car.drive(self.config.DRIVE_SPEED)
            self.debug["lane_poi"] = follow_lane_poi(
                self._lane_tracker, car, sensors.get("bottom"))

        else:  # DONE
            car.stop()

    # ---- (4단계) PARK 기동 시퀀스 — 서브 페이즈 머신 ----

    def _park_enter(self, phase, now, pose=None):
        self._park_phase = phase
        self._park_t0 = now
        self._park_pulses = 0
        self._park_last_pulse = 0.0
        self._park_pose0 = pose

    def _park_pulse(self, car, direction, target, now):
        # 펄스 간격은 차선 변경과 동일한 값 공유 (steer_pulse 반복 전송 주기)
        gap = _config.STEER_PULSE_GAP_S
        if self._park_pulses < target and now - self._park_last_pulse >= gap:
            car.steer_pulse(direction)
            self._park_pulses += 1
            self._park_last_pulse = now

    def _trusted_pose(self, sensors):
        """pose_conf가 임계 이상일 때만 pose 반환 — 미보정(conf=0) 시 None."""
        if sensors.get("pose_conf", 0.0) >= self.p["min_pose_conf"]:
            return sensors.get("pose")
        return None

    def _park_tick(self, sensors, car):
        now = self._now()
        if self._park_phase is None:
            self._park_enter("TURN_IN", now, self._trusted_pose(sensors))
        in_phase = now - self._park_t0
        # 슬롯 쪽으로 "후진" 진입 — 후진 시 차체 뒤가 조향 반대쪽으로 돈다
        turn_dir = "L" if self.p["side"] == "R" else "R"
        counter = "R" if turn_dir == "L" else "L"

        if self._park_phase == "TURN_IN":
            car.drive(-self.config.SLOW_SPEED)
            self._park_pulse(car, turn_dir, self.p["turn_in_pulses"], now)
            if in_phase >= self.p["turn_in_s"]:
                self._park_enter("STRAIGHTEN", now, self._trusted_pose(sensors))

        elif self._park_phase == "STRAIGHTEN":
            car.drive(-self.config.SLOW_SPEED)
            self._park_pulse(car, counter, self.p["turn_in_pulses"], now)
            if in_phase >= self.p["straighten_s"]:
                car.steer("F")
                self._park_enter("CREEP", now, self._trusted_pose(sensors))

        elif self._park_phase == "CREEP":
            car.drive(-self.config.SLOW_SPEED)
            steer = self.reverse_lane_steer(sensors.get("rear"),
                                            debug=self.debug.setdefault("parking_line", {}))
            if steer is not None:
                car.steer(steer)
            if self.parked(sensors) or in_phase >= self.p["park_max_s"]:
                # 진입 CREEP의 실측 시간/거리를 기록 — 출차(EXIT_CREEP)가 같은
                # 만큼 전진해 슬롯을 빠져나가는 미러링 기준이 된다.
                self._creep_s_actual = in_phase
                self._creep_m_actual = traveled_m(self._park_pose0,
                                                  self._trusted_pose(sensors))
                car.drive(0)
                car.steer("F")
                self._park_enter("HOLD", now)

        elif self._park_phase == "HOLD":
            car.drive(0)  # 주차 완료 정지 유지 (규정 3~5초)
            if in_phase >= self.p["hold_s"]:
                if self.p["exit_enabled"]:
                    self.state = "EXIT"
                    self._park_phase = None  # _exit_tick이 EXIT_CREEP부터 시작
                else:
                    self.state = "DONE"

    # ---- (5단계) 출차 기동 시퀀스 — 진입 미러 서브 페이즈 머신 ----

    def _finish_exit(self, car, timed_out=False):
        # exit_result: 텔레메트리/로그에서 타임아웃(사실상 f7 출차실패)을 정상
        # 완료와 구분하기 위한 표시. 판정 로직 자체를 바꾸지는 않는다.
        self.debug["exit_result"] = "timeout" if timed_out else "ok"
        car.steer("F")
        if self.p["exit_mode"] == "lane":
            self.state = "LANE_FOLLOW"
            car.drive(self.config.DRIVE_SPEED)
        else:
            self.state = "DONE"
            car.stop()

    def _exit_tick(self, sensors, car):
        now = self._now()
        if self._park_phase is None:
            self._exit_t_start = now
            self._park_enter("EXIT_CREEP", now, self._trusted_pose(sensors))
        in_phase = now - self._park_t0
        # 진입과 같은 방향: 후진 좌조향으로 그린 호는 전진 좌조향으로 되짚는다
        turn_dir = "L" if self.p["side"] == "R" else "R"
        counter = "R" if turn_dir == "L" else "L"

        if now - self._exit_t_start >= self.p["exit_max_s"]:
            print("[t_parking] EXIT 타임아웃 — 안전 종료")
            self._finish_exit(car, timed_out=True)
            return

        if self._park_phase == "EXIT_CREEP":
            # 진입 CREEP의 실측(시간/거리)만큼 + 여유로 전진해 슬롯을 벗어난다
            car.drive(self.config.SLOW_SPEED)
            car.steer("F")
            done = in_phase >= (self._creep_s_actual or self.p["exit_creep_s"]) \
                + self.p["exit_margin_s"]
            if not done and self._creep_m_actual:
                d = traveled_m(self._park_pose0, self._trusted_pose(sensors))
                done = d is not None and \
                    d >= self._creep_m_actual + self.p["exit_margin_m"]
            if done:
                self._park_enter("EXIT_TURN", now, self._trusted_pose(sensors))

        elif self._park_phase == "EXIT_TURN":
            car.drive(self.config.SLOW_SPEED)
            self._park_pulse(car, turn_dir, self.p["turn_in_pulses"], now)
            if in_phase >= self.p["exit_turn_s"]:
                self._park_enter("EXIT_STRAIGHT", now, self._trusted_pose(sensors))

        elif self._park_phase == "EXIT_STRAIGHT":
            car.drive(self.config.SLOW_SPEED)
            self._park_pulse(car, counter, self.p["turn_in_pulses"], now)
            if in_phase >= self.p["exit_straight_s"]:
                self._finish_exit(car)

    # ---- 판정 로직 ----

    def map_update(self, scan, sensors):
        """(1단계) 스캔 누적.

        deque(maxlen=map_scans)는 항상 유지한다(폴백 + map_complete 판정).
        추가로 오도메트리가 신뢰 가능하면(pose_conf >= min_pose_conf) 점유
        격자(OccupancyMap)에 odom 정합 누적한다 — slot_found가 순간 스캔 대신
        누적 맵을 쓰게 되어 한두 프레임 노이즈/가림에 강건해진다. 미보정
        (conf=0)이면 격자를 아예 만들지 않아 기존 동작과 완전히 같다.
        """
        self.scans.append(scan)
        pose = self._trusted_pose(sensors)
        if pose is not None and np is not None:
            if self.occ is None:
                self.occ = OccupancyMap(size_m=self.p["occ_size_m"],
                                        res_m=self.p["occ_res_m"],
                                        min_hits=self.p["occ_min_hits"])
            self.occ.add_scan(scan, pose, self.config.LIDAR_MOUNT,
                              self.config.LIDAR_SELF_MASK_DEG)
            self.debug["occupancy"] = self.occ

    def map_complete(self):
        """(1단계) 스캔이 map_scans회 쌓이면 맵 빌딩 완료."""
        return len(self.scans) >= self.p["map_scans"]

    def slot_found(self, sensors):
        """측면 근접 클러스터(주차 차량) 사이의 갭으로 T주차 슬롯 판정.

        후방 0도 라이다의 측면 섹터(전방 기준 75~165도, side 방향)에서
        slot_max_lateral_m 이내 점만 취해 bearing 순으로 정렬, 이웃 점 사이
        chord 폭이 slot_gap_min_m 이상이면 두 주차 차량 사이 갭이다.

        점유 격자가 있으면(오도메트리 보정 완료) 순간 스캔 대신 누적 맵을
        현재 pose 기준 스캔으로 역변환(synthesize_scan)해 같은 chord 로직을
        돌린다 — 노이즈/가림에 강건. 없으면 기존 순간 스캔 그대로.
        """
        scan = sensors.get("lidar_scan")
        pose = self._trusted_pose(sensors)
        if self.occ is not None and pose is not None:
            scan = self.occ.synthesize_scan(
                pose, self.p["slot_max_lateral_m"] * 2.0, self.config.LIDAR_MOUNT)
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

    def reverse_lane_steer(self, rear_frame, debug=None):
        """(2,3단계) 후방 카메라 주차선 인식 → 'F'/'L'/'R'. 실패 시 None.

        하단 절반 ROI에서 흰색(STOP_LINE과 같은 임계) 컬럼 히스토그램을 만들어
        주차선 2줄의 컬럼 클러스터를 찾고, 두 줄 중점과 화면 중앙의 오차로
        조향을 정한다. "후진" 시 차체 뒤가 조향 반대쪽으로 돌아가므로
        전진 기준 방향을 구한 뒤 L↔R을 반전해 반환한다.

        오차(err)는 lane_follow.LaneCenterTracker와 동일한 칼만필터
        (self._err_kf)로 프레임 간 스무딩한다 — 주차선이 한 프레임 안 보여도
        (glare/각도 등) 곧바로 _last_err_px=None으로 정렬 카운트를 리셋하지
        않고 predict-only로 버틴다(2026-07-18). 분산이 kf_max_variance_px를
        넘어서야(오래 못 봄) 실제로 None 취급한다.

        debug: dict를 넘기면 클러스터/원시오차/필터링오차/판정/분산을 채운다
        (debug_viz.draw_parking_line 오버레이용). 반환값/판정 로직(첫 프레임의
        err==raw err 등)은 불변.
        """
        if cv2 is None or rear_frame is None:
            return None

        def _predict_only():
            self._err_kf.predict(self.p["kf_process_noise"])
            max_var = self.p["kf_max_variance_px"]
            variance = self._err_kf.variance()
            too_uncertain = max_var is not None and variance is not None and variance > max_var
            self._last_err_px = None if too_uncertain else self._err_kf.value()

        try:
            s_max, v_min = _config.white_hsv(self.p)  # 흰색 임계 (config.WHITE_HSV 공유)
            h, w = rear_frame.shape[:2]
            roi = rear_frame[h // 2:, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 0, v_min), (179, s_max, 255))
            col_sum = mask.sum(axis=0).astype(float)
            if debug is not None:
                debug.update(roi_y0=h // 2, tol=self.p["align_tol_px"],
                             clusters=[], mid=None, raw_err=None, err=None, steer=None)
            if col_sum.max() <= 0:
                _predict_only()
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
            if debug is not None:
                debug["clusters"] = clusters
            if len(clusters) < 2:
                _predict_only()
                return None  # 주차선 2줄이 다 보여야 정렬 가능
            mid = (clusters[0] + clusters[-1]) / 2.0
            raw_err = mid - w / 2.0
            self._err_kf.predict(self.p["kf_process_noise"])
            err = self._err_kf.update(raw_err, self.p["kf_measurement_noise"])
            self._last_err_px = err
            if abs(err) <= self.p["align_tol_px"]:
                steer = "F"
            else:
                forward = "R" if err > 0 else "L"          # 전진 기준 목표 방향
                steer = "L" if forward == "R" else "R"      # 후진 조향 반전
            if debug is not None:
                debug.update(mid=mid, raw_err=raw_err, err=err, steer=steer,
                             variance=self._err_kf.variance())
            return steer
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
        d = rear_min_m(sensors.get("lidar_scan"), cfg.LIDAR_MOUNT,
                       cfg.LIDAR_REAR_SECTOR, cfg.LIDAR_SELF_MASK_DEG) \
            if sensors.get("lidar_scan") else None
        if d is not None and d <= self.p["rear_stop_m"]:
            self._parked_count += 1
        else:
            self._parked_count = 0
        return self._parked_count >= 3
