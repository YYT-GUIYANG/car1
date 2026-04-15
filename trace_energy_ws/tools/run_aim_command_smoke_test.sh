#!/usr/bin/env bash
# 无相机：验证 aim_command 在 aim_workflow_enabled:=false 时仍能切换 aim_engaged（tracking_debug[10]）。
set +o nounset 2>/dev/null || true
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/.ros_smoke_logs}"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# 避免与本机其它终端里已在跑的 trace / Qt 同域串 aim_command、tracking_debug
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +o nounset 2>/dev/null || true
source /opt/ros/humble/setup.bash
set +o nounset 2>/dev/null || true
source "$WS/install/setup.bash"

: "${SERVO_MESSAGE_INSTALL:=/tmp/ros_servo_test_ws/install/servo_message}"
if [[ -d "$SERVO_MESSAGE_INSTALL/lib" ]]; then
  export LD_LIBRARY_PATH="$SERVO_MESSAGE_INSTALL/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"

TRACE_PID=""
cleanup() {
  if [[ -n "$TRACE_PID" ]] && kill -0 "$TRACE_PID" 2>/dev/null; then
    kill "$TRACE_PID" 2>/dev/null || true
    wait "$TRACE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[aim-smoke] 启动 trace_calculator（aim_workflow_enabled:=false）…"
ros2 run trace_energy trace_calculator --ros-args \
  -p control_enabled:=false \
  -p startup_hold_frames:=0 \
  -p energy_mode:=large \
  -p track_color_square_only:=true \
  -p aim_workflow_enabled:=false \
  -p publish_tracking_debug:=true \
  -p show_vis_window:=false \
  >/tmp/trace_energy_aim_smoke.log 2>&1 &
TRACE_PID=$!

sleep 2
if ! kill -0 "$TRACE_PID" 2>/dev/null; then
  echo "[aim-smoke] trace_calculator 已退出，日志："
  tail -40 /tmp/trace_energy_aim_smoke.log
  exit 1
fi

echo "[aim-smoke] 运行 aim_command 烟测…"
python3 "$WS/tools/aim_command_smoke_test.py"
RC=$?

cleanup
trap - EXIT
exit "$RC"
