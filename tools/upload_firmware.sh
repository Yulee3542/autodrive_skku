#!/usr/bin/env bash
# car_controller.ino를 아두이노 메가 2560에 컴파일+업로드한다 — arduino-cli 사용,
# WSL2/Ubuntu에서 Arduino IDE GUI 없이 동작한다 (usbipd로 이미 붙인 시리얼 포트를
# 그대로 씀 — ROS 통신에 쓰는 포트와 동일한 경로로 업로드도 가능).
#
# 사용법:
#   ./tools/upload_firmware.sh                              # 포트 자동 감지, car_controller 업로드
#   ./tools/upload_firmware.sh /dev/ttyACM0                 # 포트 직접 지정
#   ./tools/upload_firmware.sh /dev/ttyACM0 arduino/pin_test  # 다른 스케치 업로드 (예: 핀 진단용)
set -e
cd "$(dirname "$0")/.."

SKETCH_DIR="${2:-arduino/car_controller}"
FQBN="arduino:avr:mega"

if ! command -v arduino-cli >/dev/null 2>&1; then
    echo "== arduino-cli 설치 (~/.local/bin) =="
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
        | BINDIR="$HOME/.local/bin" sh
    export PATH="$HOME/.local/bin:$PATH"
    if ! command -v arduino-cli >/dev/null 2>&1; then
        echo "arduino-cli 설치는 됐지만 PATH에 없습니다 — ~/.bashrc에 추가 후 재로그인:"
        echo '  export PATH="$HOME/.local/bin:$PATH"'
        exit 1
    fi
fi

if ! arduino-cli core list | grep -q "^arduino:avr"; then
    echo "== arduino:avr 코어 설치 (Mega 2560용) =="
    arduino-cli core update-index
    arduino-cli core install arduino:avr
fi

PORT="${1:-}"
if [ -z "$PORT" ]; then
    PORT=$(arduino-cli board list | grep -oE '/dev/tty(ACM|USB)[0-9]+' | head -1)
    if [ -z "$PORT" ]; then
        echo "포트를 찾지 못했습니다 — ./tools/upload_firmware.sh /dev/ttyACM0 처럼 직접 지정하세요"
        echo "(usbipd로 WSL에 attach가 안 됐으면 먼저 attach할 것 — README 'WSL2에서 실행' 참고)"
        exit 1
    fi
fi

echo "== 컴파일 (FQBN: $FQBN) =="
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"

echo "== 업로드 (포트: $PORT) =="
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"

echo
echo "업로드 완료. 확인: python3 tools/hw_test.py"
