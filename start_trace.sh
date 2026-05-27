#!/bin/bash
# 一键编译并启动；请把 SCRIPT_DIR 改成你本机仓库所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/trace_energy_ws" || exit 1
source /opt/ros/humble/setup.bash
colcon build --packages-select trace_energy
source install/setup.bash
ros2 launch trace_energy trace_start.launch.py "$@"
