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

steering_adc_left/steering_adc_right(기본 0.0=미설정)로 조향 POT(A6) 좌/우
풀락 ADC를 고정값으로 넘기면 arduino_node가 /car/steering_angle을 발행한다.
값은 미리 'python tools/hw_test.py --pot'로 한 번 수동 측정해서 넣을 것 —
기동할 때마다 자동으로 좌/우를 탐색하며 바퀴를 움직이던 이전 방식은 지도
교수 피드백(2026-07-18: 예측 불가능해 바람직하지 않음, 중앙 정렬은 하드웨어
텐션 스프링이 담당)에 따라 없앴다. 미설정으로 두면 /car/steering_pot(raw
ADC)만 발행되고 /car/steering_angle은 나오지 않는다(POT 미장착과 동일 취급).

log_drive:=true(기본값)면 mission_node가 매 제어 틱마다 현재 튜닝값과
조향/속도 명령을 타임스탬프와 함께 log_dir(기본 config.DRIVE_LOG_DIR) 아래
JSON Lines 파일로 남긴다 — 디지털 트윈에서 주행을 재현하기 위한 로그
(지도 교수 피드백, 2026-07-18). 끄려면 log_drive:=false.

run_odometry:=true(기본값)면 odometry_node가 함께 뜬다 — VO(시각 오도메트리)와
커맨드-적분(가짜 데드레커닝)을 융합해 /car/pose(PoseStamped, 상대 좌표),
/car/pose_confidence(Float32)를 발행한다. config.CAMERA_MOUNT/ODOMETRY.pwm_to_mps가
아직 미측정이라 실측 전까지는 confidence=0으로 사실상 비활성 동작 — 평소엔 켜둬도
무해하다.

run_arduino:=false로 실행하면 arduino_node 없이 기동한다 — 모터가 전혀 안
움직이므로 mission_node가 계산해 /car/cmd/steer 등에 발행하는 값을
"ros2 topic echo /car/cmd/steer"로 안전하게 관찰만 하고 싶을 때(검출 로직
검증, 차는 손으로 옮기며 확인) 쓴다. teleop_node로 실제 주행하면서 동시에
mission의 계산값만 참고하고 싶으면 run_arduino:=true(기본값)로 두고
mission_node/teleop_node를 같이 띄우면 되는데, 이 경우 실제 조향은 두
소스의 명령이 섞여 정확한 수동 조종은 안 된다는 점 주의(관찰 목적으로만).

run_lidar:=false로 실행하면 rplidar/lidar_node 없이 기동한다 — 라이다
연결이 불안정해 반복적으로 죽을 때, 순수 카메라 기반 검출만 볼 때 쓴다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from autodrive_skku_ros.nodes.ports import (
    autodetect_cameras, autodetect_ports, cleanup_stale_ros_state)


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

    # 카메라 모델(C920)로 자동 필터링 — USB 재열거로 /dev/videoN 번호가
    # 매번 바뀔 수 있어(2026-07-17 실차: video0이 엉뚱한 내장/타사 웹캠이라
    # front_camera 기본값 0이 틀렸던 사례) 이름으로 우리 카메라만 골라낸다.
    # 어느 인덱스가 전방/후방인지는 물리 마운트 방향 문제라 자동으로 알 수
    # 없음 — 찾은 순서대로 앞부터 배정하고, 틀리면 front_camera:=/rear_camera:=
    # 로 바꿀 것(Foxglove로 실제 화면 보고 확인).
    _auto_cams = autodetect_cameras()
    auto_front_camera = _auto_cams[0] if len(_auto_cams) >= 1 else 0
    auto_rear_camera = _auto_cams[1] if len(_auto_cams) >= 2 else -1

    run_mission_arg = DeclareLaunchArgument(
        "run_mission", default_value="true",
        description="false면 mission_node 없이 센서/액추에이터 노드만 기동 "
                     "(ros2 topic pub으로 수동 테스트용)")
    run_arduino_arg = DeclareLaunchArgument(
        "run_arduino", default_value="true",
        description="false면 arduino_node 없이 기동 — 모터를 전혀 안 움직이면서 "
                     "카메라/미션 검출 로직만 검증할 때(예: /car/cmd/steer 값이 "
                     "제대로 나오는지 손으로 차를 옮기며 확인). 조향 POT 캘리브레이션도 "
                     "생략됨. arduino_node가 없으면 미션이 게시하는 /car/cmd/* "
                     "메시지는 받는 쪽이 없어 그냥 사라질 뿐 안전함.")
    run_lidar_arg = DeclareLaunchArgument(
        "run_lidar", default_value="true",
        description="false면 rplidar/lidar_node 없이 기동 — 라이다 연결이 "
                     "불안정하거나(반복 크래시) 순수 카메라 기반 검출만 볼 때.")
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
    front_camera_arg = DeclareLaunchArgument(
        "front_camera", default_value=str(auto_front_camera),
        description="전방 카메라 /dev/videoN 인덱스 — 이름(C920)으로 자동 감지, "
                     "다른 카메라(웹캠 등)와 섞여 있으면 순서가 틀릴 수 있으니 "
                     "Foxglove로 확인 후 필요하면 직접 지정할 것")
    rear_camera_arg = DeclareLaunchArgument(
        "rear_camera", default_value=str(auto_rear_camera),
        description="-1이면 후방 카메라 미사용 — C920이 2대 이상 감지되면 "
                     "두 번째를 자동 사용, 아니면 -1")
    show_arg = DeclareLaunchArgument(
        "show", default_value="false",
        description="카메라 창 표시 (디스플레이 필요 — DISPLAY 환경변수 없으면 "
                     "mission_node가 경고 후 자동으로 꺼짐, 2026-07-17 이전엔 "
                     "SSH/헤드리스에서 프로세스 전체가 죽었음)")
    foxglove_port_arg = DeclareLaunchArgument("foxglove_port", default_value="8765")
    steering_adc_left_arg = DeclareLaunchArgument(
        "steering_adc_left", default_value="0.0",
        description="조향 POT 좌 풀락 ADC 고정값 — tools/hw_test.py --pot로 "
                     "미리 수동 측정. 0.0(right와 동일)이면 미설정 취급으로 "
                     "/car/steering_angle을 발행하지 않음.")
    steering_adc_right_arg = DeclareLaunchArgument(
        "steering_adc_right", default_value="0.0",
        description="조향 POT 우 풀락 ADC 고정값 — steering_adc_left 참고.")
    log_drive_arg = DeclareLaunchArgument(
        "log_drive", default_value="true",
        description="true면 mission_node가 매 틱 튜닝값+명령을 타임스탬프와 "
                     "함께 log_dir 아래 JSON Lines로 기록 (디지털 트윈 재현용).")
    log_dir_arg = DeclareLaunchArgument(
        "log_dir", default_value="",
        description="주행 로그 저장 디렉토리. 비우면 config.DRIVE_LOG_DIR 사용.")
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
            "steering_adc_left": ParameterValue(
                LaunchConfiguration("steering_adc_left"), value_type=float),
            "steering_adc_right": ParameterValue(
                LaunchConfiguration("steering_adc_right"), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration("run_arduino")),
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
        condition=IfCondition(LaunchConfiguration("run_lidar")),
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
            condition=IfCondition(LaunchConfiguration("run_lidar")),
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
                "log_drive": ParameterValue(LaunchConfiguration("log_drive"), value_type=bool),
                "log_dir": LaunchConfiguration("log_dir"),
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
        run_mission_arg, mission_arg, run_arduino_arg, run_lidar_arg,
        arduino_port_arg, lidar_port_arg,
        front_camera_arg, rear_camera_arg, show_arg, foxglove_port_arg,
        steering_adc_left_arg, steering_adc_right_arg,
        log_drive_arg, log_dir_arg,
        run_odometry_arg, tuning_params_arg,
        arduino_bridge, camera_publisher, rplidar,
        OpaqueFunction(function=_tuned_nodes),
        foxglove_bridge,
    ])
