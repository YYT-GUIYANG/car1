#!/usr/bin/env bash
# ========== 唯一推荐：在本目录执行 ==========
#   bash go.sh
# 即：编译 trace_energy（可 SKIP_BUILD=1 跳过）→ 启动 相机+追踪画窗+舵机+Qt；Qt 发 aim_command，默认与舵机自动跟同时有效。
# 可选：RUN_VERIFY=1 bash go.sh  先跑烟测
# 无桌面：SHOW_VIS_WINDOW=false launch_aim_panel:=false bash go.sh
set +o nounset 2>/dev/null || true
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

# ROS2 Humble 的 ros2/rclpy 绑定在 Python3.10；conda base 常为 3.12，会报 No module named rclpy._rclpy_pybind11
if [[ -n "${CONDA_DEFAULT_ENV:-}" ]] && command -v conda >/dev/null 2>&1; then
  echo ">>> [go.sh] 退出 Conda「$CONDA_DEFAULT_ENV」，改用系统环境加载 ROS（否则 rqt/ros2 Python 易冲突）" >&2
  # shellcheck disable=SC1091
  source "$(conda info --base 2>/dev/null)/etc/profile.d/conda.sh" 2>/dev/null || true
  i=0
  while [[ -n "${CONDA_SHLVL:-}" && "${CONDA_SHLVL}" != "0" && $i -lt 12 ]]; do
    conda deactivate 2>/dev/null || break
    i=$((i + 1))
  done
  unset i
fi

set +o nounset 2>/dev/null || true
source /opt/ros/humble/setup.bash
set +o nounset 2>/dev/null || true
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "未找到 install/setup.bash。首次请在本目录执行: colcon build --symlink-install" >&2
  echo "若工作区路径含中文导致 servo_message 失败，请执行: bash \"$WS/tools/repair_servo_message_and_build.sh\"" >&2
  exit 1
fi
source "$WS/install/setup.bash"

# 不完整安装时只有 rosidl 片段、缺 servo_messageConfig.cmake，trace_energy 的 CMake 会报 find_package 失败
SERVO_CFG="$WS/install/servo_message/share/servo_message/cmake/servo_messageConfig.cmake"
if [[ ! -f "$SERVO_CFG" ]]; then
  echo "" >&2
  echo ">>> 检测到 servo_message 安装不完整（缺少 servo_messageConfig.cmake）。" >&2
  echo ">>> 常见原因：路径含中文时在本工作区直接编 servo_message 失败，或合并中断。" >&2
  echo ">>> 正在自动运行: bash \"$WS/tools/repair_servo_message_and_build.sh\"" >&2
  echo "" >&2
  bash "$WS/tools/repair_servo_message_and_build.sh"
  source "$WS/install/setup.bash"
elif [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  colcon build --packages-select trace_energy --symlink-install
  source "$WS/install/setup.bash"
fi

if [[ "${RUN_VERIFY:-0}" == "1" ]]; then
  bash "$WS/tools/verify_all.sh"
fi

echo "" >&2
echo ">>> 双画面：「Camera /processed_image (raw)」原始图 +「Received Image」追踪；Qt「瞄准控制」发 /aim_command 。" >&2
echo ">>> 本终端=主控台请一直运行；另开终端再 ros2 topic echo /servo_control（否则无输出）。" >&2
echo "" >&2

exec bash "$WS/start_trace_energy.sh" "$@"
