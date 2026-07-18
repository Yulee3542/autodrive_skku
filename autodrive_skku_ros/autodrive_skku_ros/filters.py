"""범용 1차원 칼만필터 유틸 — 순수 함수/클래스, ROS·cv2·numpy 불필요.

lane_follow.LaneCenterTracker(차선 중심 픽셀), t_parking.reverse_lane_steer
(주차선 오차 픽셀), odometry_node.fuse(x/y/theta 각 축), arduino_node의 라이브
조향 POT 스트림이 전부 이 모듈의 ScalarKalmanFilter/PoseKalmanFilter를 공유한다.

기존에 흩어져 있던 스무딩(EMA 고정 alpha, 무필터링)과 달리, 매 스텝 실제
분산(P)으로부터 칼만 이득(K)을 계산해 블렌딩 비율을 정한다 — 측정을 못 받는
틱(predict-only)에는 추정값은 유지한 채 P만 커져, "지금 추정이 얼마나
불확실한가"를 오버레이/신뢰도 게이팅에 그대로 노출할 수 있다.

오프라인 셀프테스트: python3 -m autodrive_skku_ros.filters --selftest
"""


class ScalarKalmanFilter:
    """1차원 칼만필터 — 상태는 추정값(x)과 그 분산(P) 뿐 (F=H=B=1인 일반 KF).

    predict(): 측정 없는 틱에도 매번 호출 가능 — 추정값(x)은 그대로 두고
    분산(P)만 키운다(+control로 알려진 제어입력이 있으면 x에도 반영).
    update(): 첫 호출(x가 아직 None)이면 측정값으로 직접 초기화(x=z, P=r).
    이후 호출부터는 K=P/(P+r)로 계산한 이득으로 블렌딩한다.
    """

    def __init__(self, x0=None, p0=1.0):
        self.x = x0   # 추정값. None = 아직 초기화 전(첫 update 대기)
        self.p = p0   # 추정 분산(불확실성)

    def predict(self, q, dt=1.0, control=0.0):
        """예측 스텝: x <- x + control ; P <- P + q*dt.
        미초기화 상태(x is None)면 아무 것도 안 함 — 첫 update가 초기화한다."""
        if self.x is None:
            return
        self.x += control
        self.p += q * dt

    def update(self, z, r):
        """갱신 스텝: K <- P/(P+r) ; x <- x + K*(z-x) ; P <- (1-K)*P.
        미초기화 상태면 z로 직접 초기화(P<-r). 갱신 후 x를 반환한다."""
        if self.x is None:
            self.x = z
            self.p = r
            return self.x
        k = self.p / (self.p + r)
        self.x += k * (z - self.x)
        self.p *= (1.0 - k)
        return self.x

    def value(self):
        return self.x

    def variance(self):
        return self.p

    def reset(self, x0=None, p0=1.0):
        self.x, self.p = x0, p0


class PoseKalmanFilter:
    """odometry_node.fuse()가 쓰는 x/y/theta 독립 스칼라 KF 3개의 컨테이너 —
    pose 자체가 이 KF들의 상태다(노드가 별도로 x/y/theta 사본을 들고
    다니지 않아 동기화 버그를 피한다). x0=0.0으로 초기화(원점)하되 p0는
    크게 잡아 "값은 0이지만 사실상 전혀 신뢰 안 함" 상태로 시작한다."""

    def __init__(self, p0_pos=1e6, p0_theta=1e6):
        self.x = ScalarKalmanFilter(x0=0.0, p0=p0_pos)
        self.y = ScalarKalmanFilter(x0=0.0, p0=p0_pos)
        self.theta = ScalarKalmanFilter(x0=0.0, p0=p0_theta)

    @property
    def pose(self):
        return (self.x.value(), self.y.value(), self.theta.value())


def selftest():
    """ROS/cv2/numpy 불필요 — 순수 칼만필터 수식만 검증한다."""
    checks = []

    def check(name, ok):
        checks.append((name, ok))
        print(f"[{'OK' if ok else 'X '}] {name}")

    kf = ScalarKalmanFilter()
    check("초기화 전 predict()는 아무 효과 없음(값 없음 유지)",
          kf.value() is None)
    kf.predict(q=1.0)
    check("초기화 전 predict() 이후에도 값은 여전히 None",
          kf.value() is None)

    v = kf.update(10.0, r=4.0)
    check("첫 update()는 측정값으로 직접 초기화 (x=z, P=r)",
          v == 10.0 and kf.value() == 10.0 and kf.variance() == 4.0)

    p_before = kf.variance()
    kf.predict(q=1.0)
    check("predict-only: 값은 그대로, 분산은 커짐",
          kf.value() == 10.0 and kf.variance() > p_before)

    p_before2 = kf.variance()
    for _ in range(3):
        kf.predict(q=1.0)
    check("연속 predict-only: 분산이 단조 증가",
          kf.variance() > p_before2)

    kf2 = ScalarKalmanFilter(x0=0.0, p0=100.0)
    prev_p = kf2.variance()
    for _ in range(20):
        kf2.predict(q=0.1)
        kf2.update(5.0, r=1.0)
        check_p = kf2.variance()
        assert check_p <= prev_p + 1e-9, "분산이 증가하면 안 되는 구간에서 증가함"
        prev_p = check_p
    check("같은 값 반복 update -> 분산이 정상상태로 단조 감소",
          kf2.variance() < 100.0)
    check("같은 값 반복 update -> 추정값이 측정값에 수렴",
          abs(kf2.value() - 5.0) < 0.5)

    kf_tight = ScalarKalmanFilter(x0=0.0, p0=100.0)
    kf_loose = ScalarKalmanFilter(x0=0.0, p0=100.0)
    for _ in range(3):
        kf_tight.predict(q=0.1)
        kf_tight.update(5.0, r=0.1)   # 측정 신뢰도 높음(r 작음)
        kf_loose.predict(q=0.1)
        kf_loose.update(5.0, r=10.0)  # 측정 신뢰도 낮음(r 큼)
    check("측정 노이즈(r)가 작을수록 더 빨리 수렴",
          abs(kf_tight.value() - 5.0) < abs(kf_loose.value() - 5.0))

    kf3 = ScalarKalmanFilter(x0=0.0, p0=1.0)
    kf3.predict(q=0.0, control=3.0)
    check("control 인자는 예측 스텝에서 x에 그대로 더해짐 (process model)",
          kf3.value() == 3.0)

    pkf = PoseKalmanFilter(p0_pos=1e6, p0_theta=1e6)
    check("PoseKalmanFilter: x=0으로 초기화됐지만 분산은 매우 큼(사실상 미신뢰)",
          pkf.pose == (0.0, 0.0, 0.0) and pkf.x.variance() == 1e6)

    passed = sum(1 for _, ok in checks if ok)
    print(f"{passed}/{len(checks)} 통과")
    return 0 if passed == len(checks) else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    print("filters.py는 라이브러리 모듈입니다 — python3 -m autodrive_skku_ros.filters --selftest")
