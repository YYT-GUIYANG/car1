#!/usr/bin/env bash
# 无相机烟测：启动 trace_calculator + 合成灰图，检查丢目标后 pitch/yaw 是否进入搜索摆动。
set +o nounset 2>/dev/null || true
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"
# 避免无 ~/.ros 写权限时 spdlog 崩溃（CI / 沙箱）
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/.ros_smoke_logs}"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +o nounset 2>/dev/null || true
source /opt/ros/humble/setup.bash
set +o nounset 2>/dev/null || true
source "$WS/install/setup.bash"

# 若工作区路径含中文导致 servo_message 无法在 build 树生成 idl，可先把包拷到 /tmp 仅编译消息包：
#   mkdir -p /tmp/ros_servo_msg_ws/src
#   cp -a "$WS/src/servo_message" /tmp/ros_servo_msg_ws/src/
#   (cd /tmp/ros_servo_msg_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select servo_message)
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

echo "[smoke] 启动 trace_calculator（control_enabled=false, startup_hold_frames=0）…"
ros2 run trace_energy trace_calculator --ros-args \
  -p control_enabled:=false \
  -p startup_hold_frames:=0 \
  -p energy_mode:=large \
  -p track_color_square_only:=true \
  -p publish_tracking_debug:=true \
  -p show_vis_window:=false \
  >/tmp/trace_energy_smoke_calc.log 2>&1 &
TRACE_PID=$!

sleep 2
if ! kill -0 "$TRACE_PID" 2>/dev/null; then
  echo "[smoke] trace_calculator 已退出，日志尾部："
  tail -30 /tmp/trace_energy_smoke_calc.log
  exit 1
fi

echo "[smoke] 运行合成图 + tracking_debug 采集…"
python3 "$WS/tools/search_mode_synthetic_test.py"
RC=$?

cleanup
trap - EXIT
exit "$RC"
