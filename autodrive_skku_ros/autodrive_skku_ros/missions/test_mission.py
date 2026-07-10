from .base import Mission


class TestMission(Mission):
    """자동주행 로직 없음 — 센서/토픽은 그대로 흐르지만 차량을 조작하지 않는다.

    teleop_node나 ros2 topic pub으로 모터/조향을 수동 테스트할 때, 다른 미션처럼
    mission:=test로 똑같이 선택해서 쓸 수 있게 하는 용도(run_mission:=false와
    달리 mission_node 자체는 계속 뜬 채로 유지된다). on_start/step은 Mission
    베이스의 기본 동작(둘 다 아무것도 안 함)을 그대로 쓴다.
    """

    name = "test"
