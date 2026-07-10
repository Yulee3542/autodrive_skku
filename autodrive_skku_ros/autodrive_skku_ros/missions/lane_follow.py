try:
    from ..vendor import Function_Library as fl
except ImportError:  # 패키지 미설치 개발 환경 — 차선 인식 없이 골격만 동작
    fl = None


def follow_lane(env, car, frame, lane_edge_config):
    """차선 인식 후 조향. road/traffic 미션이 공유하는 통합 지점.

    검증된 fl.edge_detection()을 그대로 쓰되, 한 프레임에서 예외가 나도
    (나쁜 프레임/일시적 CV 오류) 미션 루프 전체가 죽지 않도록 격리한다.
    실패 시에는 steer를 아예 호출하지 않는다 — direction=None과 동일하게
    "이전 조향 유지"로 취급 (실패를 "F"로 강제 리셋하면 그 자체가 실제
    조향 액추에이션이라 더 위험함).
    """
    if frame is None or env is None:
        return

    try:
        direction = env.edge_detection(frame, **lane_edge_config)
    except Exception as e:
        print(f"[lane_follow] edge_detection 실패, 이번 프레임 스킵: {e}")
        return

    if direction == fl.FORWARD:
        car.steer("F")
    elif direction == fl.LEFT:
        car.steer("L")
    elif direction == fl.RIGHT:
        car.steer("R")
    # None이면 이전 조향 유지 (steer()의 dedupe 특성상 재전송 없음)
