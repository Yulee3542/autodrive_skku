#!/usr/bin/env bash
# autodrive_skku 환경 자동 설정 (Ubuntu / WSL2)
# 사용법: ./setup.sh
set -e
cd "$(dirname "$0")"

echo "== apt 패키지 설치 =="
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip v4l-utils

echo "== Python 가상환경 (.venv) =="
[ -d .venv ] || python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt

echo "== 시리얼 포트 권한 (dialout 그룹) =="
if ! id -nG "$USER" | grep -qw dialout; then
    sudo usermod -aG dialout "$USER"
    echo "   dialout 그룹에 추가됨 — 로그아웃 후 재로그인해야 적용됩니다."
else
    echo "   이미 dialout 그룹에 속해 있음."
fi

if grep -qi microsoft /proc/version 2>/dev/null; then
    cat <<'EOF'

[WSL2 감지] 카메라와 시리얼(아두이노/라이다)은 Windows에서 usbipd로 붙여야 합니다.
  Windows PowerShell(관리자)에서:
    winget install usbipd
    usbipd list
    usbipd bind --busid <ID>          # 장치마다 한 번 (최초 1회)
    usbipd attach --wsl --busid <ID>  # WSL 부팅/장치 재연결 시마다
  자세한 내용은 README.md 'WSL2에서 실행' 절 참고.
EOF
fi

echo
echo "설정 완료. 실행:"
echo "  source .venv/bin/activate"
echo "  python tools/check_env.py   # 장치 점검"
echo "  python main.py              # 미션 선택 후 주행"
