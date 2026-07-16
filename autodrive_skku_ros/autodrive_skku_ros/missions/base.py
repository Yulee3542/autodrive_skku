import math


def traveled_m(pose0, pose1):
    """두 pose (x, y, theta) 사이의 평면 이동 거리(m). 어느 쪽이든 None이면
    None — 오도메트리 미가용 시 거리 조건이 조용히 비활성되도록."""
    if pose0 is None or pose1 is None:
        return None
    return math.hypot(pose1[0] - pose0[0], pose1[1] - pose0[1])


class Mission:
    """모든 미션의 베이스 클래스. mission_node(rclpy)가 매 tick step()을 호출한다.

    sensors dict 키:
      top / bottom   : 전방 카메라 상/하 프레임 (C920 분할, 없으면 None)
                       top=신호등·표지판용, bottom=차선용
      rear           : 후방 카메라 프레임 (T주차용, 없으면 None)
      lidar_min_m    : "후방" 섹터 최소 거리 m, 뒤 범퍼 기준 (없으면 None).
                       RP라이다가 후방 장착(0도=차량 후방)이라 전방은 자차
                       차체에 막힘 — 전방 장애물은 카메라(bottom)로 감지한다.
      lidar_scan     : 원본 스캔 [(quality, angle_deg, dist_mm), ...] (없으면 None).
                       각도 변환/자차 필터는 nodes.lidar_node의 순수 함수 사용.
      state          : 아두이노 상태 0 정지 / 1 전진 / 2 후진 (없으면 None)
      pose           : odometry_node의 상대 pose (x_m, y_m, theta_rad) —
                       미션 시작 이후 상대 좌표, 전역/GPS 기준 아님 (없으면 None)
      pose_conf      : pose 신뢰도 0.0~1.0 (기본 0.0). config.CAMERA_MOUNT/
                       ODOMETRY.pwm_to_mps 실측 전에는 항상 0.0 — 미션은
                       pose_conf가 자기 임계(min_pose_conf) 미만이면 pose를
                       쓰지 않고 기존 타이밍 로직으로 동작해야 한다(fail-inert).

    car: ArduinoNode
      go() / stop() / drive(speed: -255..255, 음수=후진)
      steer('F'|'L'|'R')  — 스티어링 모터 펄스 (같은 값 연속 호출 무시)
      steer_pulse(d)      — 펄스 강제 반복 전송 (주차 기동용)
    """

    name = "base"

    # 틱마다 감지기 분석 결과를 담는 스크래치 — mission_node의 오버레이 타이머가
    # 읽어 /debug/* 이미지 토픽으로 발행한다 (debug_viz.py 참고). 미션 로직은
    # 여기 값을 읽지 않는다(진단 전용). 각 미션 on_start()에서 self.debug = {}로
    # 인스턴스 사본을 만든다.
    debug = {}

    def on_start(self, car, config):
        pass

    def step(self, sensors, car):
        pass

    def on_stop(self, car):
        car.stop()
