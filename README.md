# autodrive_skku

국민 AI 자율주행 경진대회 차량 코드. 아두이노 메가(구동 모터 + 스티어링 모터) + 전방 C920 카메라(상/하 분할) + RPLidar(+선택 후방 카메라) 구성이며, **ROS 2** launch 한 번으로 모든 센서 노드가 뜨고 미션을 선택해 주행한다.

주 실행 환경은 **Ubuntu + ROS 2**(대회 권장 22.04+Humble, 24.04+Jazzy 등도 동작). 개발용으로 **WSL2**도 지원. 이 저장소는 ROS 2 패키지 하나(`autodrive_skku_ros`)이고, 차량 제어는 커스텀 메시지 없이 표준 `std_msgs`만 쓴다.

**처음 쓰는 사람은 아래 [사용 설명서](#사용-설명서)를 1단계부터 순서대로 따라가면 된다.** 문제가 생기면 [문제 해결](#문제-해결) 표와 [디버깅 사다리](#디버깅-사다리-ros-없이-하드웨어만-직접-확인), 코드 구조가 궁금하면 [참고 자료](#참고-자료)로.

---

## 사용 설명서

| 단계 | 하는 일 | 언제 |
|------|---------|------|
| [1. 설치](#1단계-설치) | clone + `setup.sh` (빌드까지 자동) | 최초 1회 |
| [2. WSL2에서 실행](#2단계-wsl2에서-실행-usb-연결) | usbipd로 USB 장치 4개 연결 | WSL2 개발 PC만, 세션마다 |
| [3. 펌웨어 업로드](#3단계-펌웨어-업로드-아두이노-메가) | `car_controller.ino` 업로드 | 최초 1회 + `.ino` 변경 시마다 |
| [4. 점검](#4단계-환경-및-하드웨어-점검) | `check_env.py` / `run_tests.py` / `hw_test.py` | 주행 전 |
| [5. 수동 조작](#5단계-수동-조작으로-구동-확인) | 키보드 텔레옵으로 모터/조향 확인 | 첫 주행 전 |
| [6. 자율주행](#6단계-자율주행-미션-실행) | `bringup.launch.py mission:=…` | 본 주행 |
| [7. 모니터링](#7단계-foxglove-모니터링) | Foxglove로 카메라/토픽 확인 | 주행 중 |
| [8. 코드 업데이트](#8단계-코드-업데이트-반영-2회차-이후) | `update.sh` (+ 필요시 펌웨어 재업로드) | 원격에 새 커밋이 생겼을 때 |

### 1단계. 설치

ROS 2가 이미 설치돼 있어야 한다(권장 Ubuntu 22.04+Humble — 다른 배포판이 깔려 있으면 `setup.sh`가 `$ROS_DISTRO`를 자동 감지해 그대로 쓴다).

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Yulee3542/autodrive_skku.git
cd autodrive_skku
./setup.sh                        # apt(rplidar_ros/foxglove_bridge) + rosdep + colcon build + pip 자동 설정
source ~/ros2_ws/install/setup.bash
```

- `source ~/ros2_ws/install/setup.bash`는 새 터미널을 열 때마다 필요하다 — `~/.bashrc`에 추가해두면 편하다.
- 대회 실차(Ubuntu 직결)는 2단계를 건너뛰고 3단계로. Windows 개발 PC(WSL2)는 2단계부터.

### 2단계. WSL2에서 실행 (USB 연결)

Windows 노트북에서 개발할 때만 필요 — 대회 당일 실차는 Ubuntu 직결이라 이 단계 전체를 건너뛴다. WSL2는 USB 장치가 기본적으로 안 보이므로 **카메라 2개 + 아두이노 + 라이다, 총 4개 장치를 usbipd로 붙여야 한다.**

```powershell
# 최초 1회 (관리자 PowerShell)
winget install usbipd                  # 설치 후 새 창에서 인식됨
usbipd list                            # BUSID 확인 (카메라 2개/Arduino/CP210x)
usbipd bind --busid <ID>               # 장치마다 1회 (USB 포트 바꾸면 재실행 필요)
```
```powershell
# 이후 세션마다 (WSL 켤 때/장치 재연결할 때)
usbipd attach --wsl --busid <ID>                # 장치 4개 각각
usbipd attach --wsl --busid <ID> --auto-attach  # 자동 재연결 원하면(창 유지 필요)
```
```bash
# WSL 안에서 확인
ls /dev/video*                     # 카메라 2개 (video0, video2 등 — 짝수만 실제 장치인 경우가 많음)
ls /dev/ttyACM* /dev/ttyUSB*       # 아두이노(ttyACM*)/라이다(ttyUSB*)
python3 tools/check_env.py         # 위 4개 + 파이썬/ROS 패키지 한 번에 점검
```

usbipd 문제 해결:
- `usbipd list` 상태가 `Attached`가 아니면 아직 attach 전.
- 카메라 attach돼도 `/dev/video*` 안 생기면 `wsl --update`.
- `--show` 카메라 창은 WSLg(Windows 11 기본)로 그대로 뜬다.
- 장치가 없어도 각 노드는 경고만 내고 실행된다 — 로직 개발은 하드웨어 없이 가능.
- 아두이노가 시리얼 연결 시 리셋되며 USB 장치로 잠깐 사라졌다 나타날 수 있음(이때 attach가 끊기므로 `--auto-attach` 권장). 그래도 계속 끊기면(특히 펌웨어 업로드 중) [3단계](#3단계-펌웨어-업로드-아두이노-메가)의 Windows Arduino IDE 경로로.

### 3단계. 펌웨어 업로드 (아두이노 메가)

`arduino/car_controller/car_controller.ino`를 아두이노 메가 2560에 업로드한다 (외부 라이브러리 불필요). `.ino`는 파이썬/ROS와 별개로 컴파일·업로드되는 펌웨어다 — 한 번 올리면 보드가 그 프로그램을 독립적으로 계속 실행하며, 파이썬 쪽은 이미 업로드된 보드와 시리얼로 통신만 한다. **PC 쪽 코드만 바뀌었을 땐 재업로드가 필요 없고, `arduino/*.ino`가 바뀌었을 때만 다시 올리면 된다.**

업로드 방법 두 가지:
- **WSL2 안에서 GUI 없이**: `./tools/upload_firmware.sh` (최초 실행 시 `arduino-cli` 설치 + `arduino:avr` 코어 설치까지 자동으로 함, 포트 생략하면 자동 감지). **업로드 도중 `Broken pipe`/`timeout`/`getsync failed` 등으로 반복 실패하면 usbipd 연결이 리셋 타이밍을 못 버티는 것** — 재시도 루프 대신 바로 아래 Windows IDE 경로로 넘어갈 것.
- **Windows 쪽 Arduino IDE**: usbipd로 WSL에 attach된 상태면 먼저 `usbipd detach --busid <ID>`로 풀어준 뒤, `\\wsl.localhost\<배포판이름>\home\<user>\...\arduino\car_controller\car_controller.ino` 경로로 Windows Arduino IDE에서 직접 열어 업로드(보드: "Arduino Mega or Mega 2560", 프로세서: ATmega2560). 업로드 후 ROS 쪽에서 쓰려면 다시 `usbipd attach --wsl --busid <ID>`.

펌웨어가 PC와 주고받는 명령의 정의는 [시리얼 프로토콜](#시리얼-프로토콜-9600bps) 참고.

### 4단계. 환경 및 하드웨어 점검

```bash
python3 tools/check_env.py    # 카메라/시리얼/파이썬·ROS 패키지 인식 확인
python3 tools/run_tests.py    # 미션/센서 로직 스모크 테스트 (하드웨어 불필요, --list로 목록)
python3 tools/hw_test.py      # ⚠ 모터가 실제로 움직임 — 바퀴를 띄우고 실행
```

`hw_test.py`는 카메라/라이다/`ros2 launch` 전부 필요 없이 아두이노만 연결하면 되는 최소 구동 테스트다 — 여기서 바퀴가 돌면 배선/펌웨어는 정상이고, 안 돌면 ROS로 가기 전에 하드웨어부터 잡아야 한다. 옵션(`--forward`/`--no-steer`/`--port`)과 그 다음 좁혀가기는 [디버깅 사다리](#디버깅-사다리-ros-없이-하드웨어만-직접-확인) 참고.

### 5단계. 수동 조작으로 구동 확인

자율주행 미션이 차를 조작하지 않는 상태에서 모터/조향을 직접 확인하고 싶으면 `mission:=test`가 가장 간단하다 — **`test` 미션 자체가 키보드 텔레옵 조종 인터페이스를 겸한다**:

```bash
ros2 launch autodrive_skku_ros bringup.launch.py run_mission:=false
# 별도 터미널에서:
ros2 run autodrive_skku_ros mission_node --ros-args -p mission:=test
```
`mission_node`처럼 실제 stdin이 필요해 반드시 `ros2 run`으로 직접 실행해야 한다. Enter 없이 키 하나로 즉시 반영된다: `g`=주행 허용, `w`/`x`=속도 +20/-20, `space`=속도 0, `a`/`d`=좌/우 조향 펄스, `f`=조향 중립, `s`=정지, `h`=도움말.

별도 프로세스로 조종하고 싶으면 기존 `teleop_node`도 그대로 쓸 수 있다(어떤 미션이 떠 있든 무관하게 `/car/cmd/*`에 직접 발행 — **이걸로 모터를 움직이려면 `arduino_node`도 같이 떠 있어야 한다**, `teleop_node`는 명령을 발행만 함):

```bash
ros2 run autodrive_skku_ros arduino_node    # 별도 터미널 — 실제 시리얼 연결 담당
ros2 run autodrive_skku_ros teleop_node     # 조종
```

한 번씩 스크립트로 명령을 보내고 싶으면 `ros2 topic pub`도 그대로 쓸 수 있다:

```bash
ros2 topic pub /car/cmd/go std_msgs/msg/Empty {} --once
ros2 topic pub /car/cmd/drive std_msgs/msg/Int16 "{data: 80}" --once
ros2 topic pub /car/cmd/steer_pulse std_msgs/msg/String "{data: 'L'}" --once   # 매번 강제 펄스
ros2 topic pub /car/cmd/steer std_msgs/msg/String "{data: 'F'}" --once        # dedup(같은 값 재전송 무시)
ros2 topic pub /car/cmd/stop std_msgs/msg/Empty {} --once
```
Foxglove 앱의 "Publish" 패널로도 같은 토픽에 발행할 수 있다. 문자열 payload(`'F'`/`'L'`/`'R'`)는 실제로 아두이노에 그대로 전송되는 시리얼 바이트와 같다.

### 6단계. 자율주행 미션 실행

```bash
ros2 launch autodrive_skku_ros bringup.launch.py mission:=road show:=true
```

**`mission:=` 인자는 (기본값인 `run_mission:=true`일 때) 필수다** — `ros2 launch`는 자식 프로세스의 stdin을 연결하지 않는 ROS 2 launch 자체의 알려진 제약([ros2/launch#735](https://github.com/ros2/launch/issues/735)) 때문에 대화형 메뉴가 여기서는 안 뜬다(생략하면 에러 내고 바로 종료). 메뉴가 필요하면 다른 노드는 launch로 띄운 채 아래처럼 `mission_node`만 별도 터미널에서 직접 실행할 것(이 경우 stdin 정상 동작):

```bash
ros2 run autodrive_skku_ros mission_node
```

| launch 인자 | 설명 |
|------|------|
| `mission:={road,traffic,t_parking,test}` (`run_mission:=true`면 필수) | [미션 상세](#미션-상세) 참고 |
| `run_mission:=false` | `mission_node` 없이 센서/액추에이터 노드만 기동 — [5단계](#5단계-수동-조작으로-구동-확인) 참고 |
| `arduino_port:=/dev/ttyACM0` | 아두이노 포트 (기본: 자동 감지) |
| `lidar_port:=/dev/ttyUSB0` | 라이다 포트 (기본: 자동 감지) |
| `front_camera:=0` | 전방 카메라 인덱스 |
| `rear_camera:=2` | 후방 카메라 인덱스 (T주차용, `-1`이면 미사용) |
| `show:=true` | 카메라 창 표시 (`q`로 종료, 디스플레이 있는 환경 한정) |
| `foxglove_port:=8765` | [Foxglove 모니터링](#7단계-foxglove-모니터링) WebSocket 포트 |
| `calibrate_steering:=false` | 기본 `true` — 조향 POT 좌/우 풀락 자동 탐색(바퀴가 몇 초간 실제로 움직임, POT 미장착이면 자동 스킵). [조향 POT 자동 캘리브레이션](#조향-pot-자동-캘리브레이션-선택-하드웨어) 참고 |
| `run_odometry:=false` | 기본 `true` — 상대 pose 추정(`/car/pose`) 노드. 실측 파라미터 입력 전에는 confidence=0으로 비활성 동작이라 켜둬도 무해. [오도메트리](#오도메트리-odometry_node) 참고 |
| `tuning_params:=config/mission_tuning.yaml` | `tools/dump_tuning.py`가 저장한 튜닝 YAML로 기동 — 이전 세션의 `ros2 param set` 튜닝이 재기동 후에도 유지된다. [실차 튜닝](#실차-튜닝-ros2-param) 참고 |

**종료는 `Ctrl+C` 한 번** — SIGINT/SIGTERM 둘 다 모터에 정지 신호를 보낸 뒤 종료된다(안전 종료 처리됨). 이상하면 [문제 해결](#문제-해결) 표부터 볼 것.

하드웨어 공용 설정(포트, 카메라 인덱스, 기본 속도, 라이다 장착 보정 등)은 `autodrive_skku_ros/autodrive_skku_ros/config.py`에 있다. 미션별 튜닝값은 그 값을 실제로 쓰는 `missions/*.py` 파일 상단에 있다 — 자세한 위치는 [저장소 구조](#저장소-구조) 참고. 이 튜닝값들은 주행 중 `ros2 param set`으로도 즉시 바꿀 수 있다 — [실차 튜닝](#실차-튜닝-ros2-param) 참고.

#### 실차 첫 주행 체크리스트

1. `car_controller.ino` 업로드 ([3단계](#3단계-펌웨어-업로드-아두이노-메가) 참고. `run_test_fixed.ino` 대체, 같은 핀맵)
2. `python3 tools/check_env.py` — 장치/ROS 패키지 인식 확인
3. 바퀴를 띄운 상태에서 `python3 tools/hw_test.py` (또는 시리얼 모니터로 `G`/`2`/`S`/`L`/`R` 수동 확인) — 전진/조향 모듈 개별 확인 (막히면 [디버깅 사다리](#디버깅-사다리-ros-없이-하드웨어만-직접-확인))
4. `ros2 launch autodrive_skku_ros bringup.launch.py mission:=road show:=true` — 전진/차선 조향 확인
5. `/lidar/rear_min_m`(Foxglove) 또는 후진 동작으로 후방 감지 확인 (t_parking 미션이 사용)
6. 시리얼 케이블을 뽑아 500ms 내 정지(워치독) 확인
7. (POT 장착 차량만) 기동 로그에서 "조향 캘리브레이션 완료: adc_left=..., adc_right=..." 확인, `/car/steering_angle`(Foxglove)이 조향 펄스에 따라 바뀌는지 확인
8. Foxglove에서 `/debug/lane_poi` 오버레이 확인 — 밴드 안에 차선 클러스터가 잡히고 빨간 조향 목표선이 우측 차선 중앙을 따라가는지. 안 잡히면 `ros2 param set /mission_node lane_poi.white_thresh <값>`으로 조명에 맞게 라이브 튜닝 ([실차 튜닝](#실차-튜닝-ros2-param))
9. (traffic) `/debug/stop_line` 바그래프를 실제 정지선 앞에서 확인 — 정지선에서 빨간 행이 연속으로 나오고 횡단보도에서는 안 나오는지
10. (오도메트리 활성화하려면) `pwm_to_mps` 실측: 직선 구간을 일정 PWM으로 달리게 하고 거리/시간을 재서 `ros2 param set /odometry_node odometry.pwm_to_mps <m/s÷PWM>`. 카메라 높이/틸트도 실측해 `camera_mount.height_m`/`camera_mount.tilt_deg` 입력 → `/car/pose_confidence`가 0보다 커지는지 확인
11. (t_parking) 넓은 공터에서 전체 사이클 확인 — MAP_BUILD→…→HOLD(3~5초)→EXIT(출차)→차선유지. 손은 항상 킬스위치(Ctrl+C)에. 오도메트리 미보정 상태에서도 기존과 동일하게 동작해야 정상(격자/거리 조건은 자동 비활성)
12. 세션 종료 전 `python3 tools/dump_tuning.py` — 이날 튜닝한 값을 YAML로 뽑고, diff를 소스 dict에 반영해 커밋

### 7단계. Foxglove 모니터링

`bringup.launch.py`가 `foxglove_bridge`를 함께 띄우므로 별도 코드 없이 발행되는 모든 토픽을 Foxglove에서 볼 수 있다.

1. [Foxglove 앱](https://foxglove.dev/download)(데스크톱 또는 웹)을 연다.
2. "Open connection" → "Foxglove WebSocket" → `ws://<차량 IP 또는 localhost>:8765` 로 접속.
3. 패널을 직접 구성하거나, 미션별로 미리 만들어둔 레이아웃을 "Import layout"으로 불러온다:
   - `foxglove/road_layout.json` — `/debug/lane_poi`·`/debug/obstacle` 오버레이 + 원본 전방 + 3D 라이다 + pose
   - `foxglove/traffic_layout.json` — `/debug/traffic_light`·`/debug/stop_line` 오버레이 + 원본 전방 + `/car/state`
   - `foxglove/parking_layout.json` — `/debug/parking_line` 오버레이 + 원본 후방 + 3D(`/lidar/scan_corrected`+`/debug/occupancy`+`/car/pose`) + `/lidar/rear_min_m`

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/camera/front` | `sensor_msgs/CompressedImage` | 전방 원본 프레임 (신호등/차선 분할 전) |
| `/camera/back` | `sensor_msgs/CompressedImage` | 후방(T주차) 프레임, 미사용 시 없음 |
| `/scan` | `sensor_msgs/LaserScan` | rplidar_ros 원본 스캔 (라이다 원시 각도 규약) |
| `/lidar/scan_corrected` | `sensor_msgs/LaserScan` | 자차 반사 제거 + 차량 기준 bearing으로 정렬 — 미션이 실제로 "보는" 시야 |
| `/lidar/rear_min_m` | `std_msgs/Float32` | 뒤 범퍼 기준 후방 섹터 최소 거리 (NaN=미검출) |
| `/car/state` | `std_msgs/Int8` | 0 정지 / 1 전진 / 2 후진 |
| `/car/cmd/go`, `/car/cmd/stop` | `std_msgs/Empty` | 주행 허용 / 정지 |
| `/car/cmd/drive` | `std_msgs/Int16` | 속도 -255..255, 음수=후진 |
| `/car/cmd/steer`, `/car/cmd/steer_pulse` | `std_msgs/String` | 조향 `'F'`/`'L'`/`'R'` (steer=dedup, steer_pulse=매번 강제) |
| `/car/steering_pot` | `std_msgs/Int32` | 조향 POT 원시 ADC(0~1023) — POT 미장착이면 값이 안 움직임 |
| `/car/steering_angle` | `std_msgs/Float32` | 캘리브레이션된 조향각(deg) — 캘리브레이션 성공한 경우만 발행 |
| `/car/pose` | `geometry_msgs/PoseStamped` | 미션 시작 기준 상대 pose — [오도메트리](#오도메트리-odometry_node) 참고 |
| `/car/pose_confidence` | `std_msgs/Float32` | pose 신뢰도 0~1 (실측 파라미터 미입력 상태에서는 항상 0) |
| `/debug/lane_poi` | `sensor_msgs/CompressedImage` | POI 차선 인식 오버레이 — 밴드/클러스터/조향 목표/deadzone (road·t_parking 출차 후) |
| `/debug/obstacle` | `sensor_msgs/CompressedImage` | 장애물 블롭 판정 오버레이 — ROI/블롭 bbox/합격 여부 (road) |
| `/debug/stop_line` | `sensor_msgs/CompressedImage` | 정지선 행 채움비 바그래프 오버레이 (traffic) |
| `/debug/traffic_light` | `sensor_msgs/CompressedImage` | 신호등 픽셀비/판정 오버레이 (traffic) |
| `/debug/parking_line` | `sensor_msgs/CompressedImage` | 후방캠 주차선 2줄/중점/허용대 오버레이 (t_parking) |
| `/debug/occupancy` | `nav_msgs/OccupancyGrid` | T주차 점유 격자 (오도메트리 보정 후에만 발행, frame_id=odom) |

`/debug/*`는 `mission_node`가 감지기 분석 결과를 프레임에 그려 기본 5Hz로 발행하는 튜닝용 토픽이다. `ros2 param set /mission_node debug.overlay false`로 주행 중 끌 수 있다(주기는 `debug.overlay_hz`, 기동 시 고정). 미션에 해당 감지기가 없으면 그 토픽은 아예 생기지 않는다.

WSL2에서 개발 중이면 Windows 쪽 Foxglove 앱은 WSL 내부 IP(`ip addr show eth0`)로 접속해야 한다(usbipd로 붙인 장치와는 별개 이슈).

### 8단계. 코드 업데이트 반영 (2회차 이후)

**누군가 원격 저장소에 수정을 push했으면(팀원이든 Claude든), 실차/개발 PC에서 아래를 반드시 순서대로 다시 해야 한다** — 하나라도 빠뜨리면 "고쳤다고 들었는데 실제로는 옛날 코드/펌웨어로 테스트하고 있는" 상황이 된다(실제로 겪었던 문제):

1. **브랜치 확인**: `git branch --show-current`로 원하는 브랜치가 맞는지 먼저 확인. `main`과 작업 브랜치(예: `restructure/student-friendly-nodes`)를 착각하면 이후 전부 삽질하게 된다.
2. **코드 받기**: `setup.sh` 대신 `update.sh`(또는 그냥 `git pull`):
   ```bash
   cd ~/ros2_ws/src/autodrive_skku
   ./update.sh
   source ~/ros2_ws/install/setup.bash
   ```
   `git pull --ff-only`가 실패하면(로컬에 커밋 안 된 실차 튜닝 등) 스크립트가 멈춘다 — 자동 머지·강제 덮어쓰기 안 하므로 `git status`로 직접 확인 후 처리할 것. 파이썬 파일만 바뀐 경우 `colcon build --symlink-install`(두 스크립트가 이미 이 옵션으로 빌드) 덕분에 재빌드 없이 바로 반영된다 — `setup.py`/`package.xml`(새 노드 실행파일 추가/이름변경 등)이 바뀔 때만 재빌드 필요.
   - 재빌드했는데도 `ros2 pkg executables autodrive_skku_ros`에 새 실행파일 이름이 안 보이면, colcon이 entry_points 변경을 증분 빌드에서 못 잡아낸 것(흔한 증상) — `rm -rf build/autodrive_skku_ros install/autodrive_skku_ros` 후 `colcon build --symlink-install --packages-select autodrive_skku_ros`로 완전히 새로 빌드할 것.
3. **`arduino/*.ino`가 바뀌었으면 펌웨어 재업로드 필수**: `colcon build`/`git pull`은 `.ino` 파일에 손도 안 대므로(ROS 패키지 밖에 있음), 펌웨어 쪽 커밋이 있었다면 반드시 직접 재업로드해야 실차에 반영된다:
   ```bash
   ./tools/upload_firmware.sh "" arduino/car_controller
   ```
   (커밋 메시지에 "재업로드 필요"라고 적혀 있으면 백 퍼센트 이 경우다. 업로드 방법은 [3단계](#3단계-펌웨어-업로드-아두이노-메가) 참고.)

---

## 참고 자료

### 미션 상세

| 미션 | 세부 목표 | 상태 |
|------|-----------|------|
| `road` 도로 주행 | ① 직진·스티어링 ② 차선 인식 주행(POI 사다리꼴 다단 밴드) ③ 차선 변경 ④ 장애물 회피 차선 변경 | ①② 동작 / ③④ 테스트 구현 (실차 튜닝 대상). ③④는 오도메트리 보정 시 거리 기반 종료(`out_m`/`back_m`)도 가능 |
| `traffic` 신호등 주행 | ① 정지선 인식 ② 신호등 라이트 인식 | ② 동작 / ① 테스트 구현 (실차 튜닝 대상) |
| `t_parking` T 주차 | ① 라이다 맵 빌딩(+점유 격자) ② 후방캠 주차선 인식 ③ 후진 차선 주행 ④ T주차 알고리즘 ⑤ 출차 후 OUT 통과 | ①~⑤ 테스트 구현 (실차 튜닝 대상). 상태머신 MAP_BUILD→FIND_SLOT→REVERSE_ALIGN→PARK→EXIT→LANE_FOLLOW\|DONE. 출차는 규정 필수(출차실패 f7 −30, OUT 도착실패 f8 −40) — `exit_mode`('lane'=출차 후 차선유지 주행/'stop'=정지), `exit_enabled=false`면 기존처럼 HOLD 후 정지 |
| `test` 수동 테스트 | 자동주행 없음 — 미션 자체가 키보드 텔레옵 조종(`ros2 run`으로 직접 실행 필요) | 동작 |

"테스트 구현"은 로직이 완성돼 `tools/run_tests.py`의 스모크 테스트(FakeCar/가짜 시계로 상태머신 end-to-end)를 통과하지만, 실차에서 타이밍·임계값 튜닝은 아직 안 됐다는 뜻이다. 각 미션 파일(`autodrive_skku_ros/autodrive_skku_ros/missions/*.py`) 상단 docstring에 세부 목표·동작 방식이, 파일 상단 상수에 그 미션의 튜닝값이 정리돼 있다. `Mission.step(sensors, car)` 인터페이스가 고정돼 있어 튜닝은 상수 수정(또는 주행 중 `ros2 param set` — [실차 튜닝](#실차-튜닝-ros2-param))만으로 끝난다.

새 미션 추가: `Mission`(`missions/base.py`)을 상속한 클래스를 만들고 `missions/__init__.py`의 `MISSIONS`에 등록하면 메뉴에 자동으로 나타난다.

카메라/차선 인식 관련 세부는 [아키텍처 참고](#아키텍처-참고-개발자용) 참고.

### 저장소 구조

```
autodrive_skku/                        (git repo root — 컨테이너, 그 자체는 ROS 패키지 아님)
├── autodrive_skku_ros/                 # ament_python 패키지 — 노드/미션/런치
│   ├── autodrive_skku_ros/
│   │   ├── config.py                   # 하드웨어 공용 설정 (포트/카메라/기본속도/라이다 장착 보정)
│   │   ├── nodes/                      # 각 파일 = "순수 로직 클래스 + 얇은 ROS 래퍼(ros_main)"
│   │   │                               #   구조 — ROS 없이 python3 -m ...nodes.<파일명> --selftest로 검증 가능
│   │   │   ├── arduino_node.py         #   시리얼 프로토콜 + /car/cmd/* 구독, /car/state·steering_* 발행
│   │   │   ├── camera_node.py          #   카메라 캡처(상하분할/회전) + /camera/* 발행
│   │   │   ├── lidar_node.py           #   라이다 지오메트리(순수 함수) + /lidar/* 발행
│   │   │   ├── odometry_node.py        #   VO+커맨드적분 융합 상대 pose → /car/pose 발행
│   │   │   ├── mission_node.py         #   센서 구독 → sensors dict 구성 → Mission.step() 오케스트레이터
│   │   │   │                           #   + /debug/* 오버레이 발행 (5Hz, debug.overlay 파라미터로 on/off)
│   │   │   ├── ports.py                #   시리얼 포트 자동 감지 (ROS 비의존)
│   │   │   └── teleop_node.py          #   /car/cmd/*에 직접 발행하는 독립 키보드 조종
│   │   ├── missions/                   # 미션 로직 (road / traffic / t_parking / test, lane_follow 공유)
│   │   │                               #   각 파일 상단에 그 미션의 튜닝값 (OBSTACLE_CAM/LANE_CHANGE=
│   │   │                               #   road.py, LANE_EDGE/LANE_POI=lane_follow.py, STOP_LINE/
│   │   │                               #   TRAFFIC_PIXEL_RATIO=traffic.py, T_PARKING=t_parking.py)
│   │   │                               #   occupancy.py = T주차 점유 격자 (--selftest 가능)
│   │   ├── tuning.py                   # ros2 param set ↔ 튜닝 dict 바인딩 (실차 라이브 튜닝)
│   │   ├── debug_viz.py                # /debug/* 오버레이 드로잉 (프로토타입 시각화의 ROS 포팅)
│   │   └── vendor/                     # SKKU 제공 Function_Library (수정 금지)
│   └── launch/bringup.launch.py        # 전체 노드 + rplidar_ros + foxglove_bridge 기동
├── arduino/car_controller/             # 차량 펌웨어 (.ino)
├── arduino/pin_test/                   # 순수 GPIO 출력 진단 스케치 (시리얼 프로토콜 없이 핀만 테스트)
├── foxglove/                           # Foxglove 레이아웃 3종 ("Import layout"으로 사용):
│                                        # road_layout / traffic_layout / parking_layout.json
├── prototypes/                         # Windows 네이티브 캡처로 시각 튜닝하는 독립 프로토타입
│                                        # (lane_center_poi_windows_test.py — LANE_POI의 원본, ROS 포팅 완료)
├── setup.sh / update.sh                # 워크스페이스 최초 설정 / 최신화
├── requirements.txt                    # rplidar-roboticia만 — vendor 라이브러리 강제 임포트용(rosdep 해결 불가)
└── tools/                              # check_env.py(환경 점검), run_tests.py(모듈별 on/off 테스트 러너,
                                         # 하드웨어 불필요), smoke_test_lane_follow/missions/tuning/debug_viz.py
                                         # (run_tests가 재사용하는 개별 스모크 테스트, 단독 실행도 가능),
                                         # dump_tuning.py(세션 튜닝값 추출 → YAML + 소스 반영용 diff),
                                         # hw_test.py(실차 전진/조향 수동 테스트, 모터 실제로 움직임),
                                         # upload_firmware.sh(arduino-cli로 WSL2 안에서 펌웨어 업로드)
```

각 `nodes/*.py`는 `python3 -m autodrive_skku_ros.nodes.<파일명> --selftest`로 ROS 없이 바로 검증할 수 있다(예: `arduino_node --selftest`, `missions/occupancy.py`도 동일). 하드웨어 설정과 미션 튜닝값의 **단일 진실은 파이썬 모듈 상수**(config.py/각 미션 파일)이고, 여기에 `tuning.py`가 같은 값을 ROS 파라미터로도 노출한다 — 파라미터를 안 건드리면 상수 그대로 동작하므로(선언 기본값=상수 값) 회귀 위험 없이 실차에서만 라이브 튜닝을 쓸 수 있다. 자세한 워크플로는 [실차 튜닝](#실차-튜닝-ros2-param) 참고.

### 실차 튜닝 (ros2 param)

미션이 매 틱 읽는 튜닝 상수(LANE_POI, LANE_CHANGE, T_PARKING, 공유 흰색 임계 등)가 전부 ROS 파라미터로 노출돼 있어, 차 앞에서 재기동/리빌드 없이 값을 바꿔가며 `/debug/*` 오버레이로 즉시 확인할 수 있다.

```bash
ros2 param list /mission_node                                # 노출된 파라미터 전체 (namespace.key 형식)
ros2 param get /mission_node lane_poi.white_thresh
ros2 param set /mission_node lane_poi.white_thresh 150       # 즉시 반영 — /debug/lane_poi로 확인
ros2 param set /mission_node lane_change.out_s 2.0
ros2 param set /mission_node white.v_min 160                 # 정지선/주차선/장애물 공유 흰색 임계
ros2 param set /odometry_node odometry.pwm_to_mps 0.011      # 실측값 입력 (rebuild 불필요)
ros2 param set /odometry_node camera_mount.height_m 0.52
```

규약과 주의:

- **타입이 상수 기본값에서 온다** — 정수 파라미터(`lane_poi.white_thresh` 등)에 `150.0`을 주면 타입 불일치로 거부된다. `roi_frac`류 tuple은 배열로: `ros2 param set /mission_node lane_poi.roi_frac "[0.6, 0.95]"`.
- **`0.0` = 미측정(None)** — `camera_mount.height_m`, `obstacle_cam.white_s_max` 같은 "미측정/override 미사용" 슬롯은 0.0이 None을 뜻한다. 0.0을 다시 넣으면 해당 기능이 비활성/공유값 폴백으로 돌아간다.
- **기동 중(차선 변경/주차 기동) 변경은 다음 틱부터 적용** — 값이 찢어지지는 않지만, 기동 중이 아닐 때 바꾸는 것을 권장.
- **라이다 캘리브레이션(`lidar_mount.*`, `lidar.*`)은 노드별 사본** — `/lidar_geometry_node`와 `/mission_node` 양쪽에 같이 set해야 둘 다 반영된다.

**세션 튜닝값 보존 워크플로**: ① 주행하며 `ros2 param set`으로 튜닝 → ② 세션 끝에 `python3 tools/dump_tuning.py` — 기본값과 달라진 값만 `config/mission_tuning.yaml`로 저장하고 소스 반영용 diff를 출력 → ③ 당장 다음 주행은 `tuning_params:=config/mission_tuning.yaml`로 기동하면 유지 → ④ 값이 확정되면 diff대로 소스 dict 기본값을 고쳐 커밋(단일 진실은 항상 소스 상수, YAML은 임시 저장).

### 아키텍처 참고 (개발자용)

- 카메라는 물리적으로 **2대(전방/후방)** — `camera_node`는 그에 맞춰 `/camera/front`, `/camera/back` 딱 2개 토픽만 발행한다. 전방 **C920 한 대**의 프레임을 신호등용/차선용으로 상/하 분할하는 건 `camera_node`가 아니라 detection 쪽(`mission_node`)의 몫이다(검증된 방식):
  - `mission_node`가 `/camera/front`를 받아 상단 절반 → `sensors["top"]`(신호등 인식), 하단 절반 → `sensors["bottom"]`(차선 인식)으로 나눠 넣는다. `config.CAMERA_SPLIT`(기본 `true`)을 `false`로 두면 분할 없이 원본 프레임을 top/bottom 양쪽에 그대로 전달한다.
- 전방 카메라는 **파노라믹(가로/landscape) 마운트**가 기준이다(2026-07-16, portrait 마운트 폐기).
  `config.FRONT_CAMERA_ROTATE`는 기본 `None` — 회전 보정이 필요 없다. 세로 마운트로 다시 바꾸는
  경우에만 `"CW"`/`"CCW"`/`"180"`로 설정할 것(좌우 반전 등 안 맞으면 이 값부터 확인).
- 후방 카메라는 `config.py`의 `REAR_CAMERA` 또는 `rear_camera:=` launch 인자로 지정하면 `/camera/back`으로 발행되고, `mission_node`가 그대로 `sensors["rear"]`에 넣는다 (T주차용, 회전 보정 없음).
- 차선 인식은 두 경로가 있다 (둘 다 `missions/lane_follow.py`):
  - **`road` 미션(+t_parking 출차 후)**: POI 사다리꼴 다단 밴드 우측차선 추종(`follow_lane_poi`/`LANE_POI`, 2026-07-16 적용) — vendor `edge_detection`이 차선 없는 환경(실내 등)에서 주변 구조물 엣지를 오검출하는 문제로 개발한 대안. 원본은 `prototypes/lane_center_poi_windows_test.py`(Windows 네이티브 캡처로 시각 튜닝, ROS 포팅 완료)이고, 같은 시각화가 `/debug/lane_poi` 오버레이로 나온다.
  - **`traffic` 미션**: 팀이 검증한 `vendor/Function_Library.py`의 `edge_detection`(`follow_lane()`/`LANE_EDGE`) 그대로 유지.
- 여러 미션이 공유하는 감지 상수는 `config.py`에 단일 소스로 있다: `WHITE_HSV`(정지선/주차선/장애물 공통 흰색 임계 — 감지기별 `white_s_max`/`white_v_min` override 가능), `STEER_PULSE_GAP_S`(조향 펄스 반복 주기).

### 오도메트리 (odometry_node)

시각 오도메트리(VO — 전방 카메라 하단 프레임의 지면 특징점 추적)와 커맨드-적분(속도 PWM·조향각의 자전거 모델 적분)을 융합해 **미션 시작 이후의 상대 pose**를 추정한다 — `/car/pose`(PoseStamped)와 `/car/pose_confidence`(0~1)로 발행. IEEE 5520874의 VO+데드레커닝 융합에서 착안했지만 이 차량에는 휠 인코더가 없어 "데드레커닝" 항이 커맨드 적분일 뿐이라 슬립/정지/배터리 처짐은 감지하지 못한다.

`config.CAMERA_MOUNT`(카메라 높이/틸트)와 `config.ODOMETRY['pwm_to_mps']`가 실측 전(`None`)인 동안은 confidence=0으로 사실상 비활성 동작(fail-inert)이라 평소엔 켜둬도 무해하다 — 실측값을 채우면 그때부터 실제 추정이 시작된다. 실측값은 소스 수정 없이 실차에서 바로 넣을 수 있다: `ros2 param set /odometry_node camera_mount.height_m 0.52` 식으로 ([실차 튜닝](#실차-튜닝-ros2-param), `0.0`=미측정 규약). 끄려면 `run_odometry:=false`. 상대 오도메트리라 누적 오차는 시간이 지나면 무한정 커진다는 점 주의. 셀프테스트: `python3 -m autodrive_skku_ros.nodes.odometry_node --selftest`.

오도메트리가 보정되면(conf>0) 미션들이 자동으로 활용한다: `mission_node`가 `/car/pose`를 `sensors["pose"]`/`sensors["pose_conf"]`로 넣어주고, road 차선 변경은 거리 기반 종료(`lane_change.out_m`/`back_m`), t_parking은 점유 격자 맵빌딩(`/debug/occupancy`)과 출차 거리 미러링에 쓴다. 미보정(conf=0)이면 전부 자동 비활성 — 기존 타이밍 동작과 완전히 같다.

### 시리얼 프로토콜 (9600bps)

**조향은 차동이 아니라 전용 스티어링 모터의 120ms 펄스 방식이다**: `L`/`R` 한 번 = 한 펄스만큼 바퀴가 돌아가고 그 각도가 유지된다. `F`는 조향 모터 정지.

| 방향 | 명령 | 의미 |
|------|------|------|
| PC→차량 | `G` / `1` | 주행 허용 (V 미수신 시 기본속도 전진 — 수동 테스트용) |
| PC→차량 | `2` | 후진 (수동 테스트용) |
| PC→차량 | `S` / `3` | 정지 |
| PC→차량 | `V<int>\n` | 속도 -255..255, **음수 = 후진** (자율주행은 이걸 사용) |
| PC→차량 | `L` / `R` | 스티어링 모터 한 펄스 (120ms) |
| PC→차량 | `F` | 스티어링 모터 정지 |
| 차량→PC | `0`/`1`/`2` | 정지 / 전진 / 후진 |
| 차량→PC | `P <adc>` | 조향 POT 원시값(A6, 0~1023), 50ms마다 — 항상 보냄(POT 미장착이면 플로팅값이라 의미 없음) |

안전 장치:
- **워치독**: `V` 명령 수신 후 500ms 이상 시리얼이 끊기면 자동 정지 (파이썬 쪽은 200ms마다 keepalive 전송)
- `V`를 한 번도 받지 못하면 기존 `run_test_fixed.ino`처럼 G/2/S 수동 명령으로 동작 (구버전 호환)
- **속도 램프(Cubic Polynomial Trajectory)**: `drive(speed)`가 지정한 목표 속도까지 `V` 값을 즉시 점프시키지 않고 `SPEED_RAMP_S`(기본 0.5초, `arduino_node.py`) 동안 3차 다항식으로 부드럽게 도달시킨다(급가속/급정지로 인한 휠슬립·드리프트 완화, 2026-07-17 도입). `stop()`은 이 램프를 타지 않고 즉시 정지 — 안전 우선. 실차 튜닝 시스템(`ros2 param set`)에는 아직 안 물려 있음(코드 상수 수정 후 재빌드 필요) — 다음 단계 후보.

### 조향 POT 자동 캘리브레이션 (선택 하드웨어)

조향 링키지에 가변저항(POT)을 달고 와이퍼 핀을 아두이노 메가의 **A6**에 연결하면(전용 보드 아니고 이 메가에 직결 — GND/5V도 같이 배선), `arduino_node`가 뜰 때마다 `calibrate_steering:=true`(기본값)로 자동 캘리브레이션을 한다:

1. `steer_pulse("L")`을 반복하며 POT ADC가 더 이상 안 바뀔 때까지(기계적 풀락) 진행
2. 반대쪽도 `steer_pulse("R")`로 동일하게 진행
3. 두 풀락 값의 중간으로 조향을 되돌려 놓음
4. 이후 `/car/steering_pot`(raw ADC), `/car/steering_angle`(±`STEERING_LIMIT_DEG`로 환산한 deg)를 계속 발행

POT이 없으면(펄스를 줘도 ADC가 안 바뀌면) 자동으로 조용히 스킵되고 기존 펄스 방식 그대로 동작한다 — 항상 켜둬도 안전하다. 단, **캘리브레이션 중 바퀴가 실제로 좌우로 움직이므로** 바퀴를 띄우거나 장애물 없는 곳에서 기동할 것(정 안 되면 `calibrate_steering:=false`).

📏 2026-07 실측: 지금 장착된 POT은 조향 링키지와 완전한 1:1 커플링이 아니라, 풀락 좌우(±20도, 총 40도) 스윙에도 ADC가 4카운트 정도밖에 안 바뀐다 — `calibrate_steering()`의 `min_span`/`stable_tol`/`recenter_tol` 기본값이 이 좁은 실측 범위 기준으로 맞춰져 있다(`arduino_node.py` 참고). 이 상태에서 `/car/steering_angle`은 사실상 좌/중앙/우 정도만 구분되는 거친 해상도다 — 더 정밀하게 쓰려면 POT-조향 커플링(백래시 등)을 기계적으로 개선해야 한다.

실측 중 조향 펄스 직후 ADC가 순간적으로 튀는 현상(스티어링 모터 노이즈로 추정)도 관찰됨 — `calibrate_steering()`은 각 펄스 후 한 번만 읽지 않고 짧게 여러 번 읽어 중앙값을 쓰는 방식(`_read_pot_median`)으로 이런 스파이크를 걸러낸다. 그래도 캘리브레이션이 계속 "미검출"로 스킵되면 소프트웨어 튜닝보다 POT 배선(모터 전원선과 분리, 접지 공유점, 디커플링 커패시터 등) 쪽을 의심할 것.

---

## 디버깅 사다리 (ROS 없이 하드웨어만 직접 확인)

모터가 안 움직일 때 "ROS/토픽 쪽 문제"와 "펌웨어/배선 쪽 문제"를 아래 순서로 좁혀나간다.

**1단계 — `hw_test.py`**: 카메라/라이다/`ros2 launch` 전부 필요 없이 아두이노만 연결하면 됨.
```bash
python3 tools/hw_test.py                       # 전진 + 조향 둘 다
python3 tools/hw_test.py --forward             # 전진 모듈만
python3 tools/hw_test.py --no-steer            # 조향만 빼고
python3 tools/hw_test.py --port /dev/ttyACM0 --speed 80 --duration 2
```
바퀴를 지면에서 띄운 상태에서 실행할 것 — 실제로 모터가 움직인다(`car.go()`를 먼저 호출하므로 go-게이트 문제는 없음). 안 움직이면 배선/펌웨어 문제, 움직이는데 `teleop_node`/`test` 미션으로는 안 움직이면 ROS 쪽(포트 자동감지, 토픽 연결)을 의심.

**2단계 — `pin_test.ino`**: 그래도 안 움직이면 시리얼 프로토콜/워치독 로직 자체를 배제하고 순수 GPIO만 확인. `digitalWrite`/`analogWrite`만으로 LEFT → RIGHT → STEER 순서로 각 채널을 1초씩 직접 돌린다(시리얼 명령 대기 없음):
```bash
./tools/upload_firmware.sh /dev/ttyACM0 arduino/pin_test
```
방향 판단은 반드시 고정된 기준으로 할 것(예: "운전자가 뒤에서 앞을 보는 시점" — 매번 같은 기준으로 봐야 함, 기준이 흔들리면 잘못된 결론에 도달하기 쉽다). 이것도 안 움직이면 핀/배선/모터 드라이버 전원(아두이노 USB 전원과 별도로 모터 드라이버에 구동 전원이 들어가는지) 쪽 문제로 좁혀진다. 확인 후 반드시 `./tools/upload_firmware.sh`로 `car_controller.ino`를 다시 올릴 것.

## 문제 해결

| 증상 | 해결 |
|------|------|
| `/dev/ttyUSB0` permission denied | `./setup.sh`가 dialout 그룹에 추가함 — **재로그인** 필요 |
| `could not open port ...: Device or resource busy` | 다른 프로세스가 이미 그 포트를 잡고 있음 — `screen`으로 시리얼 모니터링하다 안 끄고 나갔을 때 흔함(`screen -ls`로 남은 세션 확인 후 `screen -X -S <세션> quit`, 또는 `fuser`/`lsof /dev/ttyACM*`로 PID 찾아서 `kill`) |
| `/dev/ttyACM0`이 없다고 나옴(`ls`했을 때 안 보임) | 아두이노가 `/dev/ttyACM1` 등 다른 번호로 잡혔을 수 있음(연결 순서에 따라 바뀜) — `ls /dev/ttyACM* /dev/ttyUSB*`로 실제 번호 확인 후 `arduino_port:=`로 직접 지정 |
| 카메라 열기 실패 | 다른 프로그램이 점유 중인지 확인, WSL2면 [usbipd attach](#2단계-wsl2에서-실행-usb-연결) |
| 아두이노/라이다 포트 뒤바뀜 | `arduino_port:=`/`lidar_port:=` launch 인자로 직접 지정 |
| 차가 안 움직임 | 미션이 `car.go()`를 호출했는지, 펌웨어 업로드 여부 확인. `teleop_node`/`test` 미션으로 수동 테스트 중이면 **`w`/`x`로 속도를 주기 전에 반드시 `g`부터 눌러야 한다** — 펌웨어 워치독 게이트(`canGo`)가 열려 있지 않으면 속도값은 받아도 실제 구동은 0으로 처리됨. `s`를 누르면 게이트가 다시 닫히므로 그 다음엔 다시 `g`부터. `teleop_node`만 띄우고 `arduino_node`를 안 띄웠어도 이 증상이 남 |
| 조향이 계속 한쪽으로 감 | 펄스 방식이라 자동 복원 안 됨 — 반대 방향 펄스로 복귀 필요. POT이 달려 있으면 기동할 때마다 자동으로 중앙 복귀됨 |
| 기동할 때마다 바퀴가 몇 초간 좌우로 저절로 움직임 | 조향 POT 자동 캘리브레이션(정상 동작) — 원치 않으면 `calibrate_steering:=false` |
| `ros2 launch`에서 미션 메뉴가 안 뜸/입력이 안 먹힘 | `mission:=road`처럼 launch 인자로 미리 지정할 것 — 대화형 메뉴는 `ros2 run autodrive_skku_ros mission_node`로 직접 실행할 때만 stdin이 정상 동작한다 |
| `colcon build`에서 새 실행파일 이름이 안 보임 | [8단계](#8단계-코드-업데이트-반영-2회차-이후) 참고 — 증분 빌드가 entry_points 변경을 못 잡는 흔한 문제 |
| `colcon build`가 이 프로젝트와 무관한 다른 패키지 때문에 실패/중단됨 | 워크스페이스 `src/`에 다른 패키지가 같이 있으면 그중 하나만 깨져도 기본적으로 전체 빌드가 중단된다 — `colcon build --packages-up-to autodrive_skku_ros`로 이 프로젝트만 빌드 대상으로 좁힐 것(`setup.sh`/`update.sh`는 이미 이렇게 함). `.venv` 등 파이썬 가상환경이 활성화된 상태로 빌드하면 다른 이유로도 실패할 수 있으니 `deactivate` 후 시도할 것 — ROS 2 전환 후에는 `.venv`가 원래 필요 없다(이전 워크플로의 잔재라면 삭제해도 됨) |
| `.sh` 스크립트 실행 시 `Permission denied` | `setup.sh`/`update.sh`가 매번 실행권한을 자동으로 복구하지만, 그 스크립트 자체를 직접 처음 실행할 땐 아직 안 걸려 있을 수 있음 — `chmod +x setup.sh` 등으로 한 번만 직접 부여 |
| `/scan`의 좌우/전후가 기대와 다름 | `rplidar_ros`의 각도 규약이 기존 파이썬 `rplidar` 라이브러리와 다를 수 있음 — `config.LIDAR_MOUNT`(`yaw_offset_deg`/`invert`)를 실차에서 재보정 |
