#!/usr/bin/env bash
# 一键：停掉其它节点后，用 ROS2 兼容的 Python 3.10 跑舵机扫掠（勿在 conda base 下直接 python3）。
set +o nounset 2>/dev/null || true
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"
set +o nounset 2>/dev/null || true
source /opt/ros/humble/setup.bash
set +o nounset 2>/dev/null || true
source "$WS/install/setup.bash"
exec bash "$WS/tools/run_with_ros2_python.sh" "$WS/tools/servo_yaw_pitch_sweep_test.py" "$@"
