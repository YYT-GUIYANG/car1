#!/usr/bin/env bash
# ROS 2 Humble 的 rclpy 针对 Python 3.10 构建；conda base 常为 3.12，直接 python3 会报
# ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
#
# 用法（在工作区根目录，已 source humble + install 之后）:
#   bash tools/run_with_ros2_python.sh tools/servo_yaw_pitch_sweep_test.py --yaw-id 11
# 或先: conda deactivate
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS"

pick_py() {
    if [[ -x /usr/bin/python3.10 ]]; then
        printf '%s' /usr/bin/python3.10
        return 0
    fi
    if /usr/bin/python3 -c 'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)' 2>/dev/null; then
        printf '%s' /usr/bin/python3
        return 0
    fi
    return 1
}

if [[ $# -lt 1 ]]; then
    echo "用法: bash tools/run_with_ros2_python.sh <脚本.py> [参数...]" >&2
    exit 2
fi

if ! PY="$(pick_py)"; then
    echo "未找到可用的 Python 3.10（ROS 2 Humble 需要与系统一致的 3.10）。" >&2
    echo "若在 conda 环境中，请先执行: conda deactivate" >&2
    echo "然后再: source /opt/ros/humble/setup.bash && source install/setup.bash" >&2
    echo "并运行: bash tools/run_with_ros2_python.sh tools/servo_yaw_pitch_sweep_test.py ..." >&2
    exit 1
fi

exec "$PY" "$@"
