#!/usr/bin/env bash
# 工作区路径含中文时，ROS2 rosidl 生成 servo_message 会损坏路径；在 /tmp 英文路径编译后合并到本 install。
set +o nounset 2>/dev/null || true
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP_WS="${TRACE_SERVO_MSG_BUILD_WS:-/tmp/trace_servo_msg_ws}"

echo "[repair] 在 $TMP_WS 编译 servo_message …"
rm -rf "$TMP_WS"
mkdir -p "$TMP_WS/src"
cp -a "$WS/src/servo_message" "$TMP_WS/src/"
(
  cd "$TMP_WS"
  set +o nounset 2>/dev/null || true
  source /opt/ros/humble/setup.bash
  # 勿用 --symlink-install：否则 install 内符号链接指向本临时 build，rsync 后删 $TMP_WS 会断链
  colcon build --packages-select servo_message --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3.10
)

echo "[repair] 合并 install/servo_message → $WS/install …"
mkdir -p "$WS/install/servo_message"
rsync -a --delete "$TMP_WS/install/servo_message/" "$WS/install/servo_message/"

CFG="$WS/install/servo_message/share/servo_message/cmake/servo_messageConfig.cmake"
if [[ ! -f "$CFG" ]]; then
  echo "[repair] 错误：合并后仍缺少 $CFG（临时工作区编译可能失败，请查看上方 colcon 日志）。" >&2
  exit 1
fi

echo "[repair] 编译 servo_controller + trace_energy（忽略工作区内 servo_message 源码包）…"
cd "$WS"
set +o nounset 2>/dev/null || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
colcon build \
  --symlink-install \
  --packages-ignore servo_message \
  --packages-select servo_controller trace_energy

echo "[repair] 完成。请使用: source $WS/install/setup.bash"
