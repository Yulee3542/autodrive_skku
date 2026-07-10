# autodrive_skku

국민 AI 자율주행 경진대회 차량 코드. 아두이노 메가(구동 모터 + 스티어링 모터) + 전방 C920 카메라(상/하 분할) + RPLidar(+선택 후방 카메라) 구성이며, **ROS 2** launch 한 번으로 모든 센서 노드가 뜨고 미션을 선택해 주행한다. 공식 `foxglove_bridge`가 함께 뜨므로 [Foxglove](https://foxglove.dev/) 앱에서 카메라/라이다/차량 상태를 실시간으로 모니터링할 수 있다.

주 실행 환경은 **Ubuntu + ROS 2** (대회 권장 조합은 22.04 + Humble이지만 24.04 + Jazzy 등 이미 설치된 배포판도 그대로 동작한다 — `setup.sh`가 `$ROS_DISTRO`를 자동 감지), 개발용으로 **WSL2**도 지원한다 (카메라/시리얼은 usbipd 연결 필요 — 아래 [WSL2에서 실행](#wsl2에서-실행) 참고).

이 저장소는 두 개의 ROS 2 패키지를 담고 있다: 노드/미션 로직인 `autodrive_skku_ros`와 차량 제어 커스텀 메시지 `autodrive_msgs`. 콜콘 워크스페이스의 `src/` 아래 저장소 전체를 clone하면 두 패키지가 함께 빌드된다.

## 빠른 시작 (Ubuntu / WSL2, 최초 1회)

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Yulee3542/autodrive_skku.git
cd autodrive_skku
./setup.sh                                  # apt(rplidar_ros/foxglove_bridge) + rosdep + colcon build + pip(rplidar-roboticia) 자동 설정
source ~/ros2_ws/install/setup.bash
python3 tools/check_env.py                  # 카메라/시리얼/ROS 패키지 점검
python3 tools/run_tests.py                  # 모듈별 on/off 테스트 러너 (--list로 목록 확인, 하드웨어 불필요)
ros2 launch autodrive_skku_ros bringup.launch.py mission:=road show:=true
```

**`mission:=` 인자는 (기본값인 `run_mission:=true`일 때) 필수다** — ROS 2 launch
시스템이 자식 프로세스의 stdin을 연결하지 않는 알려진 제약
([ros2/launch#735](https://github.com/ros2/launch/issues/735)) 때문에, `ros2 launch`로
띄운 `mission_node`는 기존처럼 메뉴에서 고르는 대화형 선택을 할 수 없다(생략하면
에러 메시지를 내고 바로 종료). 메뉴가 꼭 필요하면 다른 노드는 그대로 launch로 띄운 채
`mission_node`만 별도 터미널에서 직접 실행할 것 (이 경우는 stdin이 정상 동작한다):

```bash
ros2 run autodrive_skku_ros mission_node
```

| launch 인자 | 설명 |
|------|------|
| `mission:={road,traffic,t_parking}` (`run_mission:=true`면 필수) | `ros2 launch`에서는 대화형 메뉴 대신 이걸로 지정 (위 제약 참고) |
| `run_mission:=false` | `mission_node` 없이 센서/액추에이터 노드만 기동 — 아래 "수동 모터 테스트" 참고 |
| `arduino_port:=/dev/ttyACM0` | 아두이노 포트 (기본: 자동 감지) |
| `lidar_port:=/dev/ttyUSB0` | 라이다 포트 (기본: 자동 감지) |
| `front_camera:=0` | 전방 카메라 인덱스 |
| `rear_camera:=2` | 후방 카메라 인덱스 (T주차용, `-1`이면 미사용) |
| `show:=true` | 카메라 창 표시 (`q`로 종료, 디스플레이 있는 환경 한정) |
| `foxglove_port:=8765` | Foxglove WebSocket 포트 |

### 수동 모터 테스트 (미션 없이)

자율주행 미션이 차를 조작하지 않는 상태에서 모터/조향을 직접 확인하고 싶으면
(기존 `tools/hw_test.py`와 같은 목적) `mission:=test`가 가장 간단하다 — **`test`
미션 자체가 키보드 텔레옵 조종 인터페이스를 겸한다** (별도 `teleop_node` 실행 불필요):

```bash
ros2 launch autodrive_skku_ros bringup.launch.py run_mission:=false

# 별도 터미널에서:
ros2 run autodrive_skku_ros mission_node --ros-args -p mission:=test
```
`mission_node`처럼 실제 stdin이 필요해(ros2 launch는 자식 프로세스 stdin을 안
붙여줌 — 위 제약 참고) 반드시 `ros2 run`으로 직접 실행해야 한다. Enter 없이
키 하나로 즉시 반영된다: `g`=주행 허용, `w`/`x`=속도 +20/-20, `space`=속도 0,
`a`/`d`=좌/우 조향 펄스, `f`=조향 중립, `s`=정지, `h`=도움말.

별도 프로세스로 조종하고 싶으면 기존 `teleop_node`도 그대로 쓸 수 있다(어떤
미션이 떠 있든 무관하게 `/car/cmd/*`에 직접 발행 — `test` 미션은 그 토픽에
아무것도 안 보내므로 서로 안 부딪힌다):

```bash
ros2 run autodrive_skku_ros teleop_node
```

한 번씩 스크립트로 명령을 보내고 싶으면 `ros2 topic pub`도 그대로 쓸 수 있다:

```bash
ros2 topic pub /car/cmd/go std_msgs/msg/Empty {} --once
ros2 topic pub /car/cmd/drive autodrive_msgs/msg/DriveCmd "{speed: 80}" --once
ros2 topic pub /car/cmd/steer autodrive_msgs/msg/SteerCmd "{direction: 'L', pulse: true}" --once
ros2 topic pub /car/cmd/stop std_msgs/msg/Empty {} --once
```

Foxglove 앱의 "Publish" 패널로도 같은 토픽에 발행할 수 있다.

기본값(속도, 정지 거리, 차선 인식 파라미터 등)은 전부 `autodrive_skku_ros/autodrive_skku_ros/config.py`에 있다 — 미션 튜닝값은 ROS 파라미터화하지 않고 그대로 파이썬 모듈로 두었다(대회 전 회귀 위험 최소화).

## 코드 업데이트 반영 (2회차 이후)

최초 설정 이후 최신 코드만 반영하려면 `setup.sh` 대신 `update.sh`를 쓴다:

```bash
cd ~/ros2_ws/src/autodrive_skku
./update.sh
source ~/ros2_ws/install/setup.bash
```

`git pull --ff-only`가 실패하면(로컬에 커밋 안 된 실차 튜닝 등) 스크립트가 멈춘다 — 자동 머지·강제 덮어쓰기는 하지 않으므로 직접 `git status`로 확인 후 처리할 것. 파이썬 파일만 바뀐 경우 `colcon build --symlink-install`(setup.sh/update.sh가 이미 이 옵션으로 빌드함) 덕분에 재빌드 없이 바로 반영된다 — `autodrive_msgs`의 메시지 정의나 `setup.py`/`package.xml`이 바뀔 때만 다시 빌드가 필요하다.

## Foxglove 모니터링

`bringup.launch.py`가 `foxglove_bridge`를 함께 띄우므로 별도 코드 없이 발행되는 모든 토픽을 Foxglove에서 볼 수 있다.

1. [Foxglove 앱](https://foxglove.dev/download)(데스크톱 또는 웹)을 연다.
2. "Open connection" → "Foxglove WebSocket" → `ws://<차량 IP 또는 localhost>:8765` 로 접속.
3. 아래 토픽을 패널에 추가:

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/camera/top` | `sensor_msgs/CompressedImage` | 상단(신호등) 프레임 |
| `/camera/bottom` | `sensor_msgs/CompressedImage` | 하단(차선) 프레임 |
| `/camera/rear` | `sensor_msgs/CompressedImage` | 후방(T주차) 프레임, 미사용 시 없음 |
| `/scan` | `sensor_msgs/LaserScan` | rplidar_ros 원본 스캔 (라이다 원시 각도 규약) |
| `/lidar/scan_corrected` | `sensor_msgs/LaserScan` | 자차 반사 제거 + 차량 기준 bearing으로 정렬 — 미션이 실제로 "보는" 시야 |
| `/lidar/rear_min_m` | `std_msgs/Float32` | 뒤 범퍼 기준 후방 섹터 최소 거리 (NaN=미검출) |
| `/car/state` | `std_msgs/Int8` | 0 정지 / 1 전진 / 2 후진 |

WSL2에서 개발 중이면 Windows 쪽 Foxglove 앱은 WSL 내부 IP(`ip addr show eth0`)로 접속해야 한다(usbipd로 붙인 장치와는 별개 이슈).

## 미션

| 미션 | 세부 목표 | 상태 |
|------|-----------|------|
| `road` 도로 주행 | ① 직진·스티어링 ② 차선 인식 주행 ③ 차선 변경 ④ 장애물 회피 차선 변경 | ①② 동작 / ③④ TODO |
| `traffic` 신호등 주행 | ① 정지선 인식 ② 신호등 라이트 인식 | ② 동작 / ① TODO |
| `t_parking` T 주차 | ① 라이다 맵 빌딩 ② 후방캠 주차선 인식 ③ 후진 차선 주행 ④ T주차 알고리즘 | 상태머신 골격 / ①~④ TODO |
| `test` 수동 테스트 | 자동주행 없음 — 미션 자체가 키보드 텔레옵 조종(`ros2 run`으로 직접 실행 필요) | 동작 |

각 미션 파일(`autodrive_skku_ros/autodrive_skku_ros/missions/*.py`) 상단 docstring에 세부 목표와 채워야 할 `TODO` 메서드가 정리돼 있다. 상태 전이·주행·조향 골격은 완성돼 있으므로 **담당 팀은 자기 미션 파일의 TODO 메서드만 채우면 된다.** 이 로직들은 ROS 2 전환 전과 100% 동일하다 — `Mission.step(sensors, car)` 인터페이스가 그대로 유지되므로.

새 미션 추가: `Mission`(`missions/base.py`)을 상속한 클래스를 만들고 `missions/__init__.py`의 `MISSIONS`에 등록하면 메뉴에 자동으로 나타난다.

## 카메라 구성

- 전방 **C920 한 대**의 프레임을 상/하로 분할해 사용한다 (검증된 방식):
  - `sensors["top"]` = 상단 절반 → 신호등 인식
  - `sensors["bottom"]` = 하단 절반 → 차선 인식
- 전방 카메라는 **portrait(세로) 마운트**가 기준이다 — 수직 화각이 넓어져(≈78°) 가까운 차선과
  먼 신호등을 한 프레임에 담기 유리하다. C920은 하드웨어 회전이 없으므로 `config.FRONT_CAMERA_ROTATE`
  (`"CW"`/`"CCW"`/`"180"`)로 캡처 후 소프트웨어 보정한다 — 실제 마운트 방향과 안 맞으면(좌우 반전 등)
  이 값부터 확인.
- 후방 카메라는 `config.py`의 `REAR_CAMERA` 또는 `rear_camera:=` launch 인자로 지정하면 `sensors["rear"]`로 들어온다 (T주차용, 회전 보정 없음).
- 차선 인식은 팀이 검증한 `vendor/Function_Library.py`의 `edge_detection`을 그대로 사용하며, 파라미터는 `config.LANE_EDGE`에 있다. `road`/`traffic` 미션은 `missions/lane_follow.py`의 `follow_lane()`을 공유해서 호출한다(중복 제거 + 프레임 단위 예외 격리).

## 저장소 구조

```
autodrive_skku/                        (git repo root — 컨테이너, 그 자체는 ROS 패키지 아님)
├── autodrive_skku_ros/                 # ament_python 패키지 — 노드/미션/런치
│   ├── autodrive_skku_ros/
│   │   ├── config.py                   # 포트/속도/임계값/차선 파라미터
│   │   ├── nodes/                      # 아두이노/카메라 브릿지, 라이다 지오메트리, 미션 오케스트레이터
│   │   ├── missions/                   # 미션 로직 (road / traffic / t_parking, lane_follow 공유)
│   │   └── vendor/                     # SKKU 제공 Function_Library (수정 금지)
│   └── launch/bringup.launch.py        # 전체 노드 + rplidar_ros + foxglove_bridge 기동
├── autodrive_msgs/                     # ament_cmake 패키지 — DriveCmd/SteerCmd 커스텀 메시지
├── arduino/car_controller/             # 차량 펌웨어 (.ino)
├── setup.sh                            # 새 워크스페이스 최초 설정
├── update.sh                           # 이미 설치된 워크스페이스 최신화
├── requirements.txt                    # rplidar-roboticia만 — vendor 라이브러리 강제 임포트용(rosdep 해결 불가)
└── tools/                              # check_env.py(환경 점검), smoke_test_lane_follow.py,
                                         # run_tests.py(모듈별 on/off 테스트 러너, 하드웨어 불필요),
                                         # hw_test.py(실차 전진/조향 수동 테스트, 모터 실제로 움직임)
```

초기 실차 검증 스크립트(`main3_c920_record.py`, `run_test_fixed.ino`)는 위 미션/펌웨어 코드로
전부 포팅 완료되어 저장소에서 제거됐다. (2026-07-10: ROS 2 전환 전 순수 Python 버전은
`pre-ros-migration` 태그로 남아 있다.)

## 펌웨어 (아두이노 메가)

`arduino/car_controller/car_controller.ino`를 Arduino IDE로 업로드한다 (외부 라이브러리 불필요). 실차 검증본 `run_test_fixed.ino`와 같은 핀맵이며, 가변 속도·후진·워치독이 추가됐다. **시리얼 프로토콜은 ROS 2 전환과 무관하게 그대로다** — `arduino_bridge_node`가 기존 `ArduinoNode` 클래스를 그대로 감싼다.

**조향은 차동이 아니라 전용 스티어링 모터의 120ms 펄스 방식이다**: `L`/`R` 한 번 = 한 펄스만큼 바퀴가 돌아가고 그 각도가 유지된다. `F`는 조향 모터 정지.

### 시리얼 프로토콜 (9600bps)

| 방향 | 명령 | 의미 |
|------|------|------|
| PC→차량 | `G` / `1` | 주행 허용 (V 미수신 시 기본속도 전진 — 수동 테스트용) |
| PC→차량 | `2` | 후진 (수동 테스트용) |
| PC→차량 | `S` / `3` | 정지 |
| PC→차량 | `V<int>\n` | 속도 -255..255, **음수 = 후진** (자율주행은 이걸 사용) |
| PC→차량 | `L` / `R` | 스티어링 모터 한 펄스 (120ms) |
| PC→차량 | `F` | 스티어링 모터 정지 |
| 차량→PC | `0`/`1`/`2` | 정지 / 전진 / 후진 |

안전 장치:
- **워치독**: `V` 명령 수신 후 500ms 이상 시리얼이 끊기면 자동 정지 (파이썬 쪽은 200ms마다 keepalive 전송)
- `V`를 한 번도 받지 못하면 기존 `run_test_fixed.ino`처럼 G/2/S 수동 명령으로 동작 (구버전 호환)

## WSL2에서 실행

WSL2는 USB 장치가 기본적으로 안 보이므로 **카메라 2개 + 아두이노 + 라이다, 총 4개 장치를 usbipd로 붙여야 한다.**

### 0. 최초 1회: usbipd 설치

Windows PowerShell(관리자):
```powershell
winget install usbipd
```
설치 후 새 PowerShell 창을 열어야 `usbipd` 명령이 인식된다.

### 1. 장치별 BUSID 확인 및 bind (장치마다 최초 1회, 관리자 PowerShell)

```powershell
usbipd list
```
`Connected` 목록에서 카메라 2개(C920 등 UVC 장치), 아두이노 메가(Arduino/CH340), 라이다(CP210x/Silicon Labs)의 BUSID를 확인한다 (예: `1-3`, `2-4` 형태). 확인한 BUSID마다:
```powershell
usbipd bind --busid <ID>
```
`bind`는 Windows가 그 장치를 WSL과 공유하도록 등록하는 것으로, 한 번 해두면 이후에는 다시 안 해도 된다(장치 자체를 바꾸지 않는 한). **USB 포트를 바꿔 꽂으면 BUSID가 달라지므로 다시 bind해야 한다** — 가능하면 항상 같은 포트를 쓸 것.

### 2. 세션마다: WSL에 attach

포트를 바꾸지 않았다면 매번 WSL을 재시작하거나 장치를 재연결할 때 attach가 필요하다:
```powershell
usbipd attach --wsl --busid <ID>   # 장치 4개 각각
```
매번 수동으로 치기 번거로우면 `--auto-attach`를 쓰면 그 창을 켜둔 동안 장치가 뽑혔다 꽂혀도(예: 아두이노 리셋) 자동으로 재연결해준다 — 4개 장치마다 별도 PowerShell 창에서 실행:
```powershell
usbipd attach --wsl --busid <ID> --auto-attach
```

### 3. WSL 안에서 확인

```bash
ls /dev/video*                     # 카메라 2개 (video0, video2 등 — 짝수만 실제 장치인 경우가 많음)
ls /dev/ttyACM* /dev/ttyUSB*       # 아두이노(ttyACM*)/라이다(ttyUSB*)
python3 tools/check_env.py         # 위 4개 + 파이썬/ROS 패키지 한 번에 점검
```

### usbipd 문제 해결

- `usbipd list`에서 상태가 `Attached`가 아니라 `Shared`나 `Not shared`면 아직 attach 전 — 1~2단계 다시 확인.
- 카메라가 attach돼도 `/dev/video*`가 안 생기면 WSL 커널이 UVC를 지원하는지 확인 (`wsl --update`).
- `--show` 카메라 창은 WSLg(Windows 11 기본)로 그대로 뜬다.
- 장치가 없어도 각 노드는 경고만 내고 실행된다 — 로직 개발은 하드웨어 없이 가능.
- 아두이노가 시리얼 연결 시 리셋되면서 USB 장치로 잠깐 사라졌다 나타날 수 있다 — 이 경우 attach가 끊기므로 `--auto-attach`를 권장.

## 문제 해결

| 증상 | 해결 |
|------|------|
| `/dev/ttyUSB0` permission denied | `./setup.sh`가 dialout 그룹에 추가함 — **재로그인** 필요 |
| 카메라 열기 실패 | 다른 프로그램이 점유 중인지 확인, WSL2면 usbipd attach |
| 아두이노/라이다 포트 뒤바뀜 | `arduino_port:=`/`lidar_port:=` launch 인자로 직접 지정 |
| 차가 안 움직임 | 미션이 `car.go()`를 호출했는지, 펌웨어 업로드 여부 확인. `teleop_node`/`test` 미션으로 수동 테스트 중이면 **`w`/`x`로 속도를 주기 전에 반드시 `g`부터 눌러야 한다** — 펌웨어 워치독 게이트(`canGo`)가 열려 있지 않으면 속도값은 받아도 실제 구동은 0으로 처리됨. `s`를 누르면 게이트가 다시 닫히므로 그 다음엔 다시 `g`부터 |
| 조향이 계속 한쪽으로 감 | 펄스 방식이라 자동 복원 안 됨 — 반대 방향 펄스로 복귀 필요 |
| `ros2 launch`에서 미션 메뉴가 안 뜸/입력이 안 먹힘 | `mission:=road`처럼 launch 인자로 미리 지정할 것 — 대화형 메뉴는 이 launch를 포그라운드 터미널에서 단독 실행할 때만 stdin이 정상 동작한다 |
| `colcon build`에서 `autodrive_msgs`가 `ModuleNotFoundError: No module named 'em'`으로 실패 | Python 가상환경(`.venv` 등)이 활성화된 상태 — `deactivate` 후 `build/`/`install/`/`log/`를 지우고 재시도(이미 실패한 빌드가 venv python 경로를 CMake 캐시에 남겨 deactivate만으로는 안 지워짐). ROS 2 전환 후에는 `.venv`가 필요 없다(이전 워크플로의 잔재라면 삭제해도 됨) |
| `colcon build`가 이 프로젝트와 무관한 다른 패키지 때문에 실패/중단됨 | 워크스페이스 `src/`에 다른 패키지가 같이 있으면 그중 하나만 깨져도 기본적으로 전체 빌드가 중단된다 — `colcon build --packages-up-to autodrive_skku_ros`로 이 프로젝트(+`autodrive_msgs`)만 빌드 대상으로 좁힐 것(`setup.sh`/`update.sh`는 이미 이렇게 함) |
| `/scan`의 좌우/전후가 기대와 다름 | `rplidar_ros`의 각도 규약이 기존 파이썬 `rplidar` 라이브러리와 다를 수 있음 — `config.LIDAR_MOUNT`(`yaw_offset_deg`/`invert`)를 실차에서 재보정 |

## 실차 첫 주행 체크리스트

1. `car_controller.ino` 업로드 (`run_test_fixed.ino` 대체 — 같은 핀맵)
2. `python3 tools/check_env.py` — 장치/ROS 패키지 인식 확인
3. 바퀴를 띄운 상태에서 `python3 tools/hw_test.py` (또는 시리얼 모니터로 `G`/`2`/`S`/`L`/`R` 수동 확인) — 전진/조향 모듈 개별 확인
4. `ros2 launch autodrive_skku_ros bringup.launch.py mission:=road show:=true` — 전진/차선 조향 확인
5. `/lidar/rear_min_m`(Foxglove) 또는 후진 동작으로 후방 감지 확인 (t_parking 미션이 사용)
6. 시리얼 케이블을 뽑아 500ms 내 정지(워치독) 확인
