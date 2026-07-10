#!/usr/bin/env bash
# 이미 설치된 워크스페이스를 최신 코드로 갱신 — setup.sh(최초 1회 설정)와 분리된
# "2회차 이후" 전용 스크립트. git pull이 --ff-only로 실패하면(로컬에 커밋 안 된
# 실차 튜닝 값이 있는 경우 등) 여기서 멈춘다 — 자동 머지/강제 덮어쓰기는 하지 않는다.
# 사용법: ./update.sh
set -e
cd "$(dirname "$0")"

if [ -n "$VIRTUAL_ENV" ]; then
    echo "Python 가상환경($VIRTUAL_ENV)이 활성화돼 있습니다 — colcon build가 이 venv의"
    echo "python3를 집어써서 ROS의 rosidl 코드생성(em 모듈 등)이 깨질 수 있습니다."
    echo "'deactivate' 실행 후 다시 시도하세요."
    exit 1
fi

echo "== git pull =="
git pull --ff-only

WS_ROOT="$(cd ../.. && pwd)"
cd "$WS_ROOT"

echo "== rosdep 의존성 갱신 (워크스페이스: $WS_ROOT) =="
rosdep install --from-paths src --ignore-src -r -y

echo "== colcon build (--symlink-install) =="
colcon build --symlink-install

cd - >/dev/null
echo "== rplidar-roboticia pip 갱신 (vendor/Function_Library.py 강제 임포트용) =="
pip3 install --user -r requirements.txt

echo
echo "완료 — 'source $WS_ROOT/install/setup.bash' 후 재실행하세요."
