#!/usr/bin/env bash
# 一键启动：相机 + 舵机串口 + pidtest + rqt_reconfigure（需已 colcon build）
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

# ROS 发行版（无 jazzy 时用 humble）
if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
elif [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "未找到 /opt/ros/jazzy 或 /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "请先编译: cd \"$WS\" && colcon build --packages-select servo_message servo_controller trace_energy pidtest" >&2
  exit 1
fi
set +u
source "$WS/install/setup.bash"
set -u

# 当前 ros2 使用的 Python 必须能 import serial（conda 环境常见缺失）
if ! python3 -c "import serial" 2>/dev/null; then
  echo "[start_pidtest_visual] 正在安装 pyserial 到当前 Python: $(command -v python3)"
  python3 -m pip install -q pyserial
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/.ros_logs}"
mkdir -p "$ROS_LOG_DIR"

exec ros2 launch pidtest pidtest_visual.launch.py "$@"
