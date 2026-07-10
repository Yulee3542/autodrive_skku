#!/usr/bin/env bash
# autodrive_skku ROS 2(Humble) 환경 자동 설정 — 새 워크스페이스 최초 1회 설정용.
# 이 저장소는 콜콘 워크스페이스의 src/ 아래 clone돼 있어야 한다
# (예: ~/ros2_ws/src/autodrive_skku). 이미 설치된 워크스페이스를 최신 코드로만
# 갱신하려면 setup.sh가 아니라 update.sh를 사용할 것.
# 사용법: ./setup.sh
set -e
cd "$(dirname "$0")"

if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "ROS 2 Humble이 설치돼 있지 않습니다. 먼저 설치하세요:"
        echo "  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
        exit 1
    fi
fi

echo "== apt 패키지 설치 (rplidar_ros / foxglove_bridge / rosdep) =="
sudo apt-get update
sudo apt-get install -y v4l-utils python3-rosdep \
    ros-humble-rplidar-ros ros-humble-foxglove-bridge

if [ ! -d /etc/ros/rosdep ]; then
    sudo rosdep init
fi
rosdep update

WS_ROOT="$(cd ../.. && pwd)"
echo "== rosdep으로 패키지 의존성 설치 (워크스페이스: $WS_ROOT) =="
cd "$WS_ROOT"
rosdep install --from-paths src --ignore-src -r -y

echo "== colcon build (--symlink-install) =="
colcon build --symlink-install

cd - >/dev/null

echo "== rplidar-roboticia pip 설치 (vendor/Function_Library.py 강제 임포트용) =="
pip3 install --user -r requirements.txt

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
echo "  source $WS_ROOT/install/setup.bash"
echo "  python3 tools/check_env.py   # 장치/패키지 점검"
echo "  ros2 launch autodrive_skku_ros bringup.launch.py mission:=road"
