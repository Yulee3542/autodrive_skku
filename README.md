# autodrive_skku

국민 AI 자율주행 경진대회 차량 코드. 아두이노 메가(구동 모터 + 스티어링 모터) + 전방 C920 카메라(상/하 분할) + RPLidar(+선택 후방 카메라) 구성이며, `main.py` 한 번 실행으로 모든 센서 노드가 뜨고 미션을 선택해 주행한다.

주 실행 환경은 **Ubuntu**, 개발용으로 **WSL2**도 지원한다 (카메라/시리얼은 usbipd 연결 필요 — 아래 [WSL2에서 실행](#wsl2에서-실행) 참고).

## 빠른 시작 (Ubuntu / WSL2)

```bash
git clone https://github.com/Yulee3542/autodrive_skku.git
cd autodrive_skku
./setup.sh                     # apt + venv + 패키지 + 시리얼 권한 자동 설정
source .venv/bin/activate
python tools/check_env.py      # 카메라/시리얼/패키지 점검
python tools/run_tests.py      # 모듈별 on/off 테스트 러너 (--list로 목록 확인)
python main.py                 # 미션 메뉴가 뜬다
```

미션을 미리 정해서 바로 실행할 수도 있다:

```bash
python main.py --mission road --show
```

| 인자 | 설명 |
|------|------|
| `--mission {road,traffic,t_parking}` | 생략하면 메뉴에서 선택 |
| `--arduino /dev/ttyACM0` | 아두이노 포트 (기본: 자동 감지) |
| `--lidar /dev/ttyUSB0` | 라이다 포트 (기본: 자동 감지) |
| `--front-camera 0` | 전방 카메라 인덱스 |
| `--rear-camera 2` | 후방 카메라 인덱스 (T주차용, 기본 미사용) |
| `--no-split` | 전방 프레임 상/하 분할 비활성화 |
| `--show` | 카메라 창 표시 (`q`로 종료) |

기본값(속도, 정지 거리, 차선 인식 파라미터 등)은 전부 `config.py`에 있다.

## 미션

| 미션 | 세부 목표 | 상태 |
|------|-----------|------|
| `road` 도로 주행 | ① 직진·스티어링 ② 차선 인식 주행 ③ 차선 변경 ④ 장애물 회피 차선 변경 | ①② 동작 / ③④ TODO |
| `traffic` 신호등 주행 | ① 정지선 인식 ② 신호등 라이트 인식 | ② 동작 / ① TODO |
| `t_parking` T 주차 | ① 라이다 맵 빌딩 ② 후방캠 주차선 인식 ③ 후진 차선 주행 ④ T주차 알고리즘 | 상태머신 골격 / ①~④ TODO |

각 미션 파일(`src/missions/*.py`) 상단 docstring에 세부 목표와 채워야 할 `TODO` 메서드가 정리돼 있다. 상태 전이·주행·조향 골격은 완성돼 있으므로 **담당 팀은 자기 미션 파일의 TODO 메서드만 채우면 된다.**

새 미션 추가: `Mission`(`src/missions/base.py`)을 상속한 클래스를 만들고 `src/missions/__init__.py`의 `MISSIONS`에 등록하면 메뉴에 자동으로 나타난다.

## 카메라 구성

- 전방 **C920 한 대**의 프레임을 상/하로 분할해 사용한다 (검증된 방식):
  - `sensors["top"]` = 상단 절반 → 신호등 인식
  - `sensors["bottom"]` = 하단 절반 → 차선 인식
- 전방 카메라는 **portrait(세로) 마운트**가 기준이다 — 수직 화각이 넓어져(≈78°) 가까운 차선과
  먼 신호등을 한 프레임에 담기 유리하다. C920은 하드웨어 회전이 없으므로 `config.FRONT_CAMERA_ROTATE`
  (`"CW"`/`"CCW"`/`"180"`)로 캡처 후 소프트웨어 보정한다 — 실제 마운트 방향과 안 맞으면(좌우 반전 등)
  이 값부터 확인.
- 후방 카메라는 `config.py`의 `REAR_CAMERA` 또는 `--rear-camera`로 지정하면 `sensors["rear"]`로 들어온다 (T주차용, 회전 보정 없음).
- 차선 인식은 팀이 검증한 `src/vendor/Function_Library.py`의 `edge_detection`을 그대로 사용하며, 파라미터는 `config.LANE_EDGE`에 있다. `road`/`traffic` 미션은 `src/missions/lane_follow.py`의 `follow_lane()`을 공유해서 호출한다(중복 제거 + 프레임 단위 예외 격리).

## 폴더 구조

```
├── main.py                    # 진입점: 미션 선택 + 모든 노드 기동
├── config.py                  # 포트/속도/임계값/차선 파라미터
├── setup.sh                   # 새 환경 자동 설정
├── arduino/car_controller/    # 차량 펌웨어 (.ino)
├── src/
│   ├── nodes/                 # 센서/액추에이터 스레드 (arduino, camera, lidar)
│   ├── missions/              # 미션 로직 (road / traffic / t_parking, lane_follow 공유)
│   └── vendor/                # SKKU 제공 Function_Library (수정 금지)
└── tools/                     # check_env.py(환경 점검), smoke_test_lane_follow.py,
│                               # run_tests.py(모듈별 on/off 테스트 러너)
```

초기 실차 검증 스크립트(`main3_c920_record.py`, `run_test_fixed.ino`)는 위 미션/펌웨어 코드로
전부 포팅 완료되어 저장소에서 제거됐다.

## 펌웨어 (아두이노 메가)

`arduino/car_controller/car_controller.ino`를 Arduino IDE로 업로드한다 (외부 라이브러리 불필요). 실차 검증본 `run_test_fixed.ino`와 같은 핀맵이며, 가변 속도·후진·워치독이 추가됐다.

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

WSL2는 USB 장치가 기본적으로 안 보이므로 **카메라 + 아두이노 + 라이다를 usbipd로 붙여야 한다.**

1. Windows PowerShell(관리자):
   ```powershell
   winget install usbipd
   usbipd list                        # 장치 BUSID 확인
   usbipd bind --busid <ID>           # 장치마다 1회
   usbipd attach --wsl --busid <ID>   # WSL 재시작/장치 재연결 시마다
   ```
2. WSL 안에서 확인:
   ```bash
   ls /dev/video*                     # 카메라
   ls /dev/ttyACM* /dev/ttyUSB*       # 아두이노/라이다
   python tools/check_env.py
   ```

- 카메라가 attach돼도 `/dev/video*`가 안 생기면 WSL 커널이 UVC를 지원하는지 확인 (`wsl --update`).
- `--show` 카메라 창은 WSLg(Windows 11 기본)로 그대로 뜬다.
- 장치가 없어도 `main.py`는 경고만 내고 실행된다 — 로직 개발은 하드웨어 없이 가능.

## 문제 해결

| 증상 | 해결 |
|------|------|
| `/dev/ttyUSB0` permission denied | `./setup.sh`가 dialout 그룹에 추가함 — **재로그인** 필요 |
| 카메라 열기 실패 | 다른 프로그램이 점유 중인지 확인, WSL2면 usbipd attach |
| 아두이노/라이다 포트 뒤바뀜 | `config.py`의 `ARDUINO_PORT`/`LIDAR_PORT` 직접 지정 |
| 차가 안 움직임 | 미션이 `car.go()`를 호출했는지, 펌웨어 업로드 여부 확인 |
| 조향이 계속 한쪽으로 감 | 펄스 방식이라 자동 복원 안 됨 — 반대 방향 펄스로 복귀 필요 |

## 실차 첫 주행 체크리스트

1. `car_controller.ino` 업로드 (`run_test_fixed.ino` 대체 — 같은 핀맵)
2. `python tools/check_env.py` — 장치 인식 확인
3. 바퀴를 띄운 상태에서 시리얼 모니터로 `G`/`2`/`S`/`L`/`R` 수동 확인
4. `python main.py --mission road --show` — 전진/차선 조향 확인
5. `V-60` 후진 동작 확인 (t_parking 미션이 사용)
6. 시리얼 케이블을 뽑아 500ms 내 정지(워치독) 확인
