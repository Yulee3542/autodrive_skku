"""autodrive_skku 전체 노드 + foxglove_bridge 기동.

사용 예:
  ros2 launch autodrive_skku_ros bringup.launch.py mission:=road foxglove_port:=8765

mission 인자는 (run_mission:=true일 때) 필수로 지정할 것 — ROS 2 launch 시스템은
자식 프로세스의 stdin을 연결하지 않는 알려진 제약(ros2/launch#735)이 있어
mission_node의 대화형 메뉴가 여기서는 동작하지 않는다(생략하면 mission_node가
에러 메시지를 내고 즉시 종료). 대화형 메뉴가 필요하면
'ros2 run autodrive_skku_ros mission_node'로 직접 실행할 것.

run_mission:=false 로 실행하면 mission_node 없이 아두이노/카메라/라이다/foxglove_bridge
만 뜬다 — 자율주행 미션이 차를 조작하지 않으므로 hw_test.py 대신 ros2 topic pub이나
Foxglove의 Publish 패널로 /car/cmd/{go,stop,drive,steer}에 직접 명령을 보내 모터를
수동 테스트할 수 있다. 예:
  ros2 launch autodrive_skku_ros bringup.launch.py run_mission:=false
  ros2 topic pub /car/cmd/go std_msgs/msg/Empty {} --once
  ros2 topic pub /car/cmd/drive std_msgs/msg/Int16 "{data: 80}" --once
  ros2 topic pub /car/cmd/steer_pulse std_msgs/msg/String "{data: 'L'}" --once
  ros2 topic pub /car/cmd/steer std_msgs/msg/String "{data: 'F'}" --once
  ros2 topic pub /car/cmd/stop std_msgs/msg/Empty {} --once

arduino_port/lidar_port를 생략하면 시리얼 포트를 자동 감지한다(기존
tools/ports.py의 autodetect_ports()를 launch 생성 시점에 그대로 재사용).

calibrate_steering:=true(기본값)면 arduino_node가 뜰 때 조향 POT(A6) 좌/우
풀락을 자동으로 찾는다 — 바퀴가 몇 초간 실제로 좌우로 움직이니 반드시 바퀴를
띄우거나 장애물 없는 곳에서 기동할 것. POT 미장착 차량이면 자동으로 스킵되므로
평소엔 그냥 둬도 되고, 정말 바퀴를 못 움직이는 상황(예: 정비 중)에서만
calibrate_steering:=false로 끌 것.

run_odometry:=true(기본값)면 odometry_node가 함께 뜬다 — VO(시각 오도메트리)와
커맨드-적분(가짜 데드레커닝)을 융합해 /car/pose(PoseStamped, 상대 좌표),
/car/pose_confidence(Float32)를 발행한다. config.CAMERA_MOUNT/ODOMETRY.pwm_to_mps가
아직 미측정이라 실측 전까지는 confidence=0으로 사실상 비활성 동작 — 평소엔 켜둬도
무해하다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from autodrive_skku_ros.nodes.ports import autodetect_ports, cleanup_stale_ros_state


def generate_launch_description():
    # 이전 실행이 크래시(Qt abort, rplidar buffer overflow 등 SIGABRT류)로
    # 죽으면 자식 프로세스가 좀비로 남거나 FastDDS 공유메모리 락 파일이 안
    # 지워져 이번 실행이 "Failed init_port fastrtps_port<N>" + 노드 디스커버리
    # 실패를 물려받는다(2026-07-17 실차에서 반복 확인) — 매 launch 시작 시 정리.
    _killed, _removed_shm = cleanup_stale_ros_state()
    if _killed or _removed_shm:
        print(f"[cleanup] 이전 실행 잔재 정리: 프로세스={_killed or '없음'}, "
              f"SHM 락 파일 {len(_removed_shm)}개 제거")

    auto_arduino, auto_lidar = autodetect_ports()

    run_mission_arg = DeclareLaunchArgument(
        "run_mission", default_value="true",
        description="false면 mission_node 없이 센서/액추에이터 노드만 기동 "
                     "(ros2 topic pub으로 수동 테스트용)")
    mission_arg = DeclareLaunchArgument(
        "mission", default_value="",
        description="road|traffic|t_parking (run_mission:=true면 필수 — "
                     "대화형 메뉴는 ros2 launch에서 안 됨)")
    arduino_port_arg = DeclareLaunchArgument(
        "arduino_port", default_value=auto_arduino or "",
        description="아두이노 시리얼 포트 (예: /dev/ttyACM0)")
    lidar_port_arg = DeclareLaunchArgument(
        "lidar_port", default_value=auto_lidar or "/dev/ttyUSB0",
        description="라이다 시리얼 포트")
    front_camera_arg = DeclareLaunchArgument("front_camera", default_value="0")
    rear_camera_arg = DeclareLaunchArgument(
        "rear_camera", default_value="-1", description="-1이면 후방 카메라 미사용")
    show_arg = DeclareLaunchArgument(
        "show", default_value="false",
        description="카메라 창 표시 (디스플레이 필요 — DISPLAY 환경변수 없으면 "
                     "mission_node가 경고 후 자동으로 꺼짐, 2026-07-17 이전엔 "
                     "SSH/헤드리스에서 프로세스 전체가 죽었음)")
    foxglove_port_arg = DeclareLaunchArgument("foxglove_port", default_value="8765")
    calibrate_steering_arg = DeclareLaunchArgument(
        "calibrate_steering", default_value="true",
        description="true면 arduino_node 기동 시 조향 POT 좌/우 풀락을 1회 자동 "
                     "탐색(수 초 소요, 바퀴가 실제로 움직임). POT 미장착이면 "
                     "자동으로 조용히 스킵됨. 바퀴를 못 띄운 상태 등에서는 false로.")
    run_odometry_arg = DeclareLaunchArgument(
        "run_odometry", default_value="true",
        description="false면 odometry_node 없이 기동. config.CAMERA_MOUNT/"
                     "ODOMETRY.pwm_to_mps 실측 전에는 어차피 confidence=0으로 "
                     "비활성 동작하므로 평소엔 켜둬도 무해함 — 순수 카메라/조향 "
                     "점검 세션 등에서만 끌 것.")
    tuning_params_arg = DeclareLaunchArgument(
        "tuning_params", default_value="",
        description="tools/dump_tuning.py가 만든 튜닝 params YAML 경로. 지정하면 "
                     "mission/odometry/lidar 노드가 그 값으로 기동해 이전 세션의 "
                     "튜닝이 재기동 후에도 유지된다. 비우면 dict 기본값 사용.")

    arduino_bridge = Node(
        package="autodrive_skku_ros",
        executable="arduino_node",
        parameters=[{
            "port": LaunchConfiguration("arduino_port"),
            "calibrate_steering": ParameterValue(
                LaunchConfiguration("calibrate_steering"), value_type=bool),
        }],
    )

    camera_publisher = Node(
        package="autodrive_skku_ros",
        executable="camera_node",
        parameters=[{
            "front_camera_index": ParameterValue(LaunchConfiguration("front_camera"), value_type=int),
            "rear_camera_index": ParameterValue(LaunchConfiguration("rear_camera"), value_type=int),
        }],
    )

    rplidar = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        parameters=[{
            "serial_port": LaunchConfiguration("lidar_port"),
            "serial_baudrate": 115200,
            "frame_id": "laser",
            "angle_compensate": True,
        }],
    )

    def _tuned_nodes(context):
        """tuning_params:=<yaml> 가 지정됐을 때만 params 파일을 노드에 붙인다 —
        빈 문자열 경로를 그대로 넘기면 launch가 파일을 열려다 실패하므로
        OpaqueFunction 안에서 조건 분기한다."""
        tuning_file = LaunchConfiguration("tuning_params").perform(context)
        extra = [tuning_file] if tuning_file else []

        lidar_geometry = Node(
            package="autodrive_skku_ros",
            executable="lidar_node",
            parameters=extra or None,
        )

        odometry = Node(
            package="autodrive_skku_ros",
            executable="odometry_node",
            parameters=extra or None,
            condition=IfCondition(LaunchConfiguration("run_odometry")),
        )

        mission = Node(
            package="autodrive_skku_ros",
            executable="mission_node",
            parameters=[{
                "mission": LaunchConfiguration("mission"),
                "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
            }] + extra,
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("run_mission")),
        )
        return [lidar_geometry, odometry, mission]

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[{
            "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
        }],
    )

    return LaunchDescription([
        run_mission_arg, mission_arg, arduino_port_arg, lidar_port_arg,
        front_camera_arg, rear_camera_arg, show_arg, foxglove_port_arg,
        calibrate_steering_arg, run_odometry_arg, tuning_params_arg,
        arduino_bridge, camera_publisher, rplidar,
        OpaqueFunction(function=_tuned_nodes),
        foxglove_bridge,
    ])
