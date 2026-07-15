# autodrive_skku

국민 AI 자율주행 경진대회 차량 코드. 아두이노 메가(구동 모터 + 스티어링 모터) + 전방 C920 카메라(상/하 분할) + RPLidar(+선택 후방 카메라) 구성이며, **ROS 2** launch 한 번으로 모든 센서 노드가 뜨고 미션을 선택해 주행한다.

주 실행 환경은 **Ubuntu + ROS 2**(대회 권장 22.04+Humble, 24.04+Jazzy 등도 동작). 개발용으로 **WSL2**도 지원. 이 저장소는 ROS 2 패키지 하나(`autodrive_skku_ros`)이고, 차량 제어는 커스텀 메시지 없이 표준 `std_msgs`만 쓴다.

---

## 빠른 시작 (Ubuntu / WSL2, 최초 1회)

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Yulee3542/autodrive_skku.git
cd autodrive_skku
./setup.sh                                  # apt(rplidar_ros/foxglove_bridge) + rosdep + colcon build + pip 자동 설정
source ~/ros2_ws/install/setup.bash
python3 tools/check_env.py                  # 카메라/시리얼/ROS 패키지 점검
python3 tools/run_tests.py                  # 모듈별 on/off 테스트 러너 (--list로 목록, 하드웨어 불필요)
ros2 launch autodrive_skku_ros bringup.launch.py mission:=road show:=true
```

WSL2에서 개발 중이면 `check_env.py`/`bringup.launch.py`가 카메라·시리얼을 잡으려면 먼저 [WSL2에서 실행](#wsl2에서-실행) 절차(usbipd)부터 해야 한다. 대회 실차(Ubuntu 직결)는 이 부분 건너뛰어도 됨.

**`mission:=` 인자는 (기본값인 `run_mission:=true`일 때) 필수다** — `ros2 launch`는 자식 프로세스의 stdin을 연결하지 않는 ROS 2 launch 자체의 알려진 제약([ros2/launch#735](https://github.com/ros2/launch/issues/735)) 때문에 대화형 메뉴가 여기서는 안 뜬다(생략하면 에러 내고 바로 종료). 메뉴가 필요하면 다른 노드는 launch로 띄운 채 아래처럼 `mission_node`만 별도 터미널에서 직접 실행할 것(이 경우 stdin 정상 동작):

```bash
ros2 run autodrive_skku_ros mission_node
```

| launch 인자 | 설명 |
|------|------|
| `mission:={road,traffic,t_parking}` (`run_mission:=true`면 필수) | [미션](#미션) 참고 |
| `run_mission:=false` | `mission_node` 없이 센서/액추에이터 노드만 기동 — 아래 "수동 모터 테스트" 참고 |
| `arduino_port:=/dev/ttyACM0` | 아두이노 포트 (기본: 자동 감지) |
| `lidar_port:=/dev/ttyUSB0` | 라이다 포트 (기본: 자동 감지) |
| `front_camera:=0` | 전방 카메라 인덱스 |
| `rear_camera:=2` | 후방 카메라 인덱스 (T주차용, `-1`이면 미사용) |
| `show:=true` | 카메라 창 표시 (`q`로 종료, 디스플레이 있는 환경 한정) |
| `foxglove_port:=8765` | [Foxglove 모니터링](#foxglove-모니터링) WebSocket 포트 |
| `calibrate_steering:=false` | 기본 `true` — 조향 POT 좌/우 풀락 자동 탐색(바퀴가 몇 초간 실제로 움직임, POT 미장착이면 자동 스킵). 자세한 건 [펌웨어](#펌웨어-아두이노-메가) 절 참고 |

**종료는 `Ctrl+C` 한 번** — SIGINT/SIGTERM 둘 다 모터에 정지 신호를 보낸 뒤 종료된다(안전 종료 처리됨). 이상하면 [문제 해결](#문제-해결) 표부터 볼 것. 처음 실차를 몰기 전에는 [실차 첫 주행 체크리스트](#실차-첫-주행-체크리스트)를 따를 것.

### 수동 모터 테스트 (미션 없이)

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

하드웨어 공용 설정(포트, 카메라 인덱스, 기본 속도, 라이다 장착 보정 등)은 `autodrive_skku_ros/autodrive_skku_ros/config.py`에 있다. 미션별 튜닝값은 그 값을 실제로 쓰는 `missions/*.py` 파일 상단에 있다 — 자세한 위치는 [저장소 구조](#저장소-구조) 참고.

문제가 안 풀리면 [디버깅 사다리](#디버깅-사다리-ros-없이-하드웨어만-직접-확인)로.

---

## 코드 업데이트 반영 (2회차 이후)

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
   (커밋 메시지에 "재업로드 필요"라고 적혀 있으면 백 퍼센트 이 경우다. 업로드 방법은 [펌웨어](#펌웨어-아두이노-메가) 절 참고.)

---

## WSL2에서 실행

Windows 노트북에서 개발할 때만 필요 — 대회 당일 실차는 Ubuntu 직결이라 이 절 전체를 건너뛴다. WSL2는 USB 장치가 기본적으로 안 보이므로 **카메라 2개 + 아두이노 + 라이다, 총 4개 장치를 usbipd로 붙여야 한다.**

```powershell
# 최초 1회 (관리자 PowerShell)
winget install usbipd                  # 설치 후 새 창에서 인식됨
usbipd list                            # BUSID 확인 (카메라 2개/Arduino/CP210x)
usbipd bind --busid <ID>               # 장치마다 1회 (USB 포트 바꾸면 재실행 필요)
```
```powershell
# 이후 세션마다 (WSL 켤 때/장치 재연결할 때) — 사용법(빠른 시작)으로 가기 전에 먼저
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
- 아두이노가 시리얼 연결 시 리셋되며 USB 장치로 잠깐 사라졌다 나타날 수 있음(이때 attach가 끊기므로 `--auto-attach` 권장). 그래도 계속 끊기면(특히 펌웨어 업로드 중) [펌웨어](#펌웨어-아두이노-메가) 절의 Windows Arduino IDE 경로로.

---

## 미션

| 미션 | 세부 목표 | 상태 |
|------|-----------|------|
| `road` 도로 주행 | ① 직진·스티어링 ② 차선 인식 주행 ③ 차선 변경 ④ 장애물 회피 차선 변경 | ①② 동작 / ③④ TODO |
| `traffic` 신호등 주행 | ① 정지선 인식 ② 신호등 라이트 인식 | ② 동작 / ① TODO |
| `t_parking` T 주차 | ① 라이다 맵 빌딩 ② 후방캠 주차선 인식 ③ 후진 차선 주행 ④ T주차 알고리즘 | 상태머신 골격 / ①~④ TODO |
| `test` 수동 테스트 | 자동주행 없음 — 미션 자체가 키보드 텔레옵 조종(`ros2 run`으로 직접 실행 필요) | 동작 |

각 미션 파일(`autodrive_skku_ros/autodrive_skku_ros/missions/*.py`) 상단 docstring에 세부 목표와 채워야 할 `TODO` 메서드가 정리돼 있다. 상태 전이·주행·조향 골격은 완성돼 있으므로 **담당 팀은 자기 미션 파일의 TODO 메서드만 채우면 된다.** `Mission.step(sensors, car)` 인터페이스가 고정돼 있어 이 로직들은 그대로 유지된다.

새 미션 추가: `Mission`(`missions/base.py`)을 상속한 클래스를 만들고 `missions/__init__.py`의 `MISSIONS`에 등록하면 메뉴에 자동으로 나타난다.

카메라/차선 인식 관련 세부는 [아키텍처 참고](#아키텍처-참고-개발자용) 참고.

---

## 저장소 구조

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
│   │   │   ├── mission_node.py         #   센서 구독 → sensors dict 구성 → Mission.step() 오케스트레이터
│   │   │   ├── ports.py                #   시리얼 포트 자동 감지 (ROS 비의존)
│   │   │   └── teleop_node.py          #   /car/cmd/*에 직접 발행하는 독립 키보드 조종
│   │   ├── missions/                   # 미션 로직 (road / traffic / t_parking, lane_follow 공유)
│   │   │                               #   각 파일 상단에 그 미션의 튜닝값 (OBSTACLE_CAM/LANE_CHANGE=
│   │   │                               #   road.py, LANE_EDGE=lane_follow.py, STOP_LINE/
│   │   │                               #   TRAFFIC_PIXEL_RATIO=traffic.py, T_PARKING=t_parking.py)
│   │   └── vendor/                     # SKKU 제공 Function_Library (수정 금지)
│   └── launch/bringup.launch.py        # 전체 노드 + rplidar_ros + foxglove_bridge 기동
├── arduino/car_controller/             # 차량 펌웨어 (.ino)
├── arduino/pin_test/                   # 순수 GPIO 출력 진단 스케치 (시리얼 프로토콜 없이 핀만 테스트)
├── setup.sh / update.sh                # 워크스페이스 최초 설정 / 최신화
├── requirements.txt                    # rplidar-roboticia만 — vendor 라이브러리 강제 임포트용(rosdep 해결 불가)
└── tools/                              # check_env.py(환경 점검), smoke_test_lane_follow.py,
                                         # run_tests.py(모듈별 on/off 테스트 러너, 하드웨어 불필요),
                                         # hw_test.py(실차 전진/조향 수동 테스트, 모터 실제로 움직임),
                                         # upload_firmware.sh(arduino-cli로 WSL2 안에서 펌웨어 업로드)
```

각 `nodes/*.py`는 `python3 -m autodrive_skku_ros.nodes.<파일명> --selftest`로 ROS 없이 바로 검증할 수 있다(예: `arduino_node --selftest`). 둘 다(하드웨어 설정/미션 튜닝값) ROS 파라미터화하지 않고 파이썬 모듈 상수로 뒀다(대회 전 회귀 위험 최소화).

---

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
| `/car/cmd/go`, `/car/cmd/stop` | `std_msgs/Empty` | 주행 허용 / 정지 |
| `/car/cmd/drive` | `std_msgs/Int16` | 속도 -255..255, 음수=후진 |
| `/car/cmd/steer`, `/car/cmd/steer_pulse` | `std_msgs/String` | 조향 `'F'`/`'L'`/`'R'` (steer=dedup, steer_pulse=매번 강제) |
| `/car/steering_pot` | `std_msgs/Int32` | 조향 POT 원시 ADC(0~1023) — POT 미장착이면 값이 안 움직임 |
| `/car/steering_angle` | `std_msgs/Float32` | 캘리브레이션된 조향각(deg) — 캘리브레이션 성공한 경우만 발행 |

WSL2에서 개발 중이면 Windows 쪽 Foxglove 앱은 WSL 내부 IP(`ip addr show eth0`)로 접속해야 한다(usbipd로 붙인 장치와는 별개 이슈).

---

## 펌웨어 (아두이노 메가)

`arduino/car_controller/car_controller.ino`를 아두이노 메가 2560에 업로드한다 (외부 라이브러리 불필요). `.ino`는 파이썬/ROS와 별개로 컴파일·업로드되는 펌웨어다 — 한 번 올리면 보드가 그 프로그램을 독립적으로 계속 실행하며, 파이썬 쪽은 이미 업로드된 보드와 시리얼로 통신만 한다.

업로드 방법 두 가지:
- **WSL2 안에서 GUI 없이**: `./tools/upload_firmware.sh` (최초 실행 시 `arduino-cli` 설치 + `arduino:avr` 코어 설치까지 자동으로 함, 포트 생략하면 자동 감지). **업로드 도중 `Broken pipe`/`timeout`/`getsync failed` 등으로 반복 실패하면 usbipd 연결이 리셋 타이밍을 못 버티는 것** — 재시도 루프 대신 바로 아래 Windows IDE 경로로 넘어갈 것.
- **Windows 쪽 Arduino IDE**: usbipd로 WSL에 attach된 상태면 먼저 `usbipd detach --busid <ID>`로 풀어준 뒤, `\\wsl.localhost\<배포판이름>\home\<user>\...\arduino\car_controller\car_controller.ino` 경로로 Windows Arduino IDE에서 직접 열어 업로드(보드: "Arduino Mega or Mega 2560", 프로세서: ATmega2560). 업로드 후 ROS 쪽에서 쓰려면 다시 `usbipd attach --wsl --busid <ID>`.

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
| 차량→PC | `P <adc>` | 조향 POT 원시값(A0, 0~1023), 50ms마다 — 항상 보냄(POT 미장착이면 플로팅값이라 의미 없음) |

안전 장치:
- **워치독**: `V` 명령 수신 후 500ms 이상 시리얼이 끊기면 자동 정지 (파이썬 쪽은 200ms마다 keepalive 전송)
- `V`를 한 번도 받지 못하면 기존 `run_test_fixed.ino`처럼 G/2/S 수동 명령으로 동작 (구버전 호환)

### 조향 POT 자동 캘리브레이션 (선택 하드웨어)

조향 링키지에 가변저항(POT)을 달고 와이퍼 핀을 아두이노 메가의 **A0**에 연결하면(전용 보드 아니고 이 메가에 직결 — GND/5V도 같이 배선), `arduino_node`가 뜰 때마다 `calibrate_steering:=true`(기본값)로 자동 캘리브레이션을 한다:

1. `steer_pulse("L")`을 반복하며 POT ADC가 더 이상 안 바뀔 때까지(기계적 풀락) 진행
2. 반대쪽도 `steer_pulse("R")`로 동일하게 진행
3. 두 풀락 값의 중간으로 조향을 되돌려 놓음
4. 이후 `/car/steering_pot`(raw ADC), `/car/steering_angle`(±`STEERING_LIMIT_DEG`로 환산한 deg)를 계속 발행

POT이 없으면(펄스를 줘도 ADC가 안 바뀌면) 자동으로 조용히 스킵되고 기존 펄스 방식 그대로 동작한다 — 항상 켜둬도 안전하다. 단, **캘리브레이션 중 바퀴가 실제로 좌우로 움직이므로** 바퀴를 띄우거나 장애물 없는 곳에서 기동할 것(정 안 되면 `calibrate_steering:=false`).

📏 2026-07 실측: 지금 장착된 POT은 조향 링키지와 완전한 1:1 커플링이 아니라, 풀락 좌우(±20도, 총 40도) 스윙에도 ADC가 4카운트 정도밖에 안 바뀐다 — `calibrate_steering()`의 `min_span`/`stable_tol`/`recenter_tol` 기본값이 이 좁은 실측 범위 기준으로 맞춰져 있다(`arduino_node.py` 참고). 이 상태에서 `/car/steering_angle`은 사실상 좌/중앙/우 정도만 구분되는 거친 해상도다 — 더 정밀하게 쓰려면 POT-조향 커플링(백래시 등)을 기계적으로 개선해야 한다.

---

## 아키텍처 참고 (개발자용)

- 전방 **C920 한 대**의 프레임을 상/하로 분할해 사용한다 (검증된 방식):
  - `sensors["top"]` = 상단 절반 → 신호등 인식
  - `sensors["bottom"]` = 하단 절반 → 차선 인식
- 전방 카메라는 **portrait(세로) 마운트**가 기준이다 — 수직 화각이 넓어져(≈78°) 가까운 차선과
  먼 신호등을 한 프레임에 담기 유리하다. C920은 하드웨어 회전이 없으므로 `config.FRONT_CAMERA_ROTATE`
  (`"CW"`/`"CCW"`/`"180"`)로 캡처 후 소프트웨어 보정한다 — 실제 마운트 방향과 안 맞으면(좌우 반전 등)
  이 값부터 확인.
- 후방 카메라는 `config.py`의 `REAR_CAMERA` 또는 `rear_camera:=` launch 인자로 지정하면 `sensors["rear"]`로 들어온다 (T주차용, 회전 보정 없음).
- 차선 인식은 팀이 검증한 `vendor/Function_Library.py`의 `edge_detection`을 그대로 사용하며, 파라미터는 `missions/lane_follow.py`의 `LANE_EDGE`에 있다. `road`/`traffic` 미션은 이 파일의 `follow_lane()`을 공유해서 호출한다(중복 제거 + 프레임 단위 예외 격리).

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

---

## 문제 해결

| 증상 | 해결 |
|------|------|
| `/dev/ttyUSB0` permission denied | `./setup.sh`가 dialout 그룹에 추가함 — **재로그인** 필요 |
| `could not open port ...: Device or resource busy` | 다른 프로세스가 이미 그 포트를 잡고 있음 — `screen`으로 시리얼 모니터링하다 안 끄고 나갔을 때 흔함(`screen -ls`로 남은 세션 확인 후 `screen -X -S <세션> quit`, 또는 `fuser`/`lsof /dev/ttyACM*`로 PID 찾아서 `kill`) |
| `/dev/ttyACM0`이 없다고 나옴(`ls`했을 때 안 보임) | 아두이노가 `/dev/ttyACM1` 등 다른 번호로 잡혔을 수 있음(연결 순서에 따라 바뀜) — `ls /dev/ttyACM* /dev/ttyUSB*`로 실제 번호 확인 후 `arduino_port:=`로 직접 지정 |
| 카메라 열기 실패 | 다른 프로그램이 점유 중인지 확인, WSL2면 [usbipd attach](#wsl2에서-실행) |
| 아두이노/라이다 포트 뒤바뀜 | `arduino_port:=`/`lidar_port:=` launch 인자로 직접 지정 |
| 차가 안 움직임 | 미션이 `car.go()`를 호출했는지, 펌웨어 업로드 여부 확인. `teleop_node`/`test` 미션으로 수동 테스트 중이면 **`w`/`x`로 속도를 주기 전에 반드시 `g`부터 눌러야 한다** — 펌웨어 워치독 게이트(`canGo`)가 열려 있지 않으면 속도값은 받아도 실제 구동은 0으로 처리됨. `s`를 누르면 게이트가 다시 닫히므로 그 다음엔 다시 `g`부터. `teleop_node`만 띄우고 `arduino_node`를 안 띄웠어도 이 증상이 남 |
| 조향이 계속 한쪽으로 감 | 펄스 방식이라 자동 복원 안 됨 — 반대 방향 펄스로 복귀 필요. POT이 달려 있으면 기동할 때마다 자동으로 중앙 복귀됨 |
| 기동할 때마다 바퀴가 몇 초간 좌우로 저절로 움직임 | 조향 POT 자동 캘리브레이션(정상 동작) — 원치 않으면 `calibrate_steering:=false` |
| `ros2 launch`에서 미션 메뉴가 안 뜸/입력이 안 먹힘 | `mission:=road`처럼 launch 인자로 미리 지정할 것 — 대화형 메뉴는 이 launch를 포그라운드 터미널에서 단독 실행할 때만 stdin이 정상 동작한다 |
| `colcon build`에서 새 실행파일 이름이 안 보임 | [코드 업데이트 반영](#코드-업데이트-반영-2회차-이후) 참고 — 증분 빌드가 entry_points 변경을 못 잡는 흔한 문제 |
| `colcon build`가 이 프로젝트와 무관한 다른 패키지 때문에 실패/중단됨 | 워크스페이스 `src/`에 다른 패키지가 같이 있으면 그중 하나만 깨져도 기본적으로 전체 빌드가 중단된다 — `colcon build --packages-up-to autodrive_skku_ros`로 이 프로젝트만 빌드 대상으로 좁힐 것(`setup.sh`/`update.sh`는 이미 이렇게 함). `.venv` 등 파이썬 가상환경이 활성화된 상태로 빌드하면 다른 이유로도 실패할 수 있으니 `deactivate` 후 시도할 것 — ROS 2 전환 후에는 `.venv`가 원래 필요 없다(이전 워크플로의 잔재라면 삭제해도 됨) |
| `.sh` 스크립트 실행 시 `Permission denied` | `setup.sh`/`update.sh`가 매번 실행권한을 자동으로 복구하지만, 그 스크립트 자체를 직접 처음 실행할 땐 아직 안 걸려 있을 수 있음 — `chmod +x setup.sh` 등으로 한 번만 직접 부여 |
| `/scan`의 좌우/전후가 기대와 다름 | `rplidar_ros`의 각도 규약이 기존 파이썬 `rplidar` 라이브러리와 다를 수 있음 — `config.LIDAR_MOUNT`(`yaw_offset_deg`/`invert`)를 실차에서 재보정 |

---

## 실차 첫 주행 체크리스트

1. `car_controller.ino` 업로드 (`./tools/upload_firmware.sh` 또는 Windows Arduino IDE — 위 [펌웨어](#펌웨어-아두이노-메가) 절 참고. `run_test_fixed.ino` 대체, 같은 핀맵)
2. `python3 tools/check_env.py` — 장치/ROS 패키지 인식 확인
3. 바퀴를 띄운 상태에서 `python3 tools/hw_test.py` (또는 시리얼 모니터로 `G`/`2`/`S`/`L`/`R` 수동 확인) — 전진/조향 모듈 개별 확인 (막히면 [디버깅 사다리](#디버깅-사다리-ros-없이-하드웨어만-직접-확인))
4. `ros2 launch autodrive_skku_ros bringup.launch.py mission:=road show:=true` — 전진/차선 조향 확인
5. `/lidar/rear_min_m`(Foxglove) 또는 후진 동작으로 후방 감지 확인 (t_parking 미션이 사용)
6. 시리얼 케이블을 뽑아 500ms 내 정지(워치독) 확인
7. (POT 장착 차량만) 기동 로그에서 "조향 캘리브레이션 완료: adc_left=..., adc_right=..." 확인, `/car/steering_angle`(Foxglove)이 조향 펄스에 따라 바뀌는지 확인
