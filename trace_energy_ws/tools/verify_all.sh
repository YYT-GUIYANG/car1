#!/usr/bin/env bash
# 维护者/CI：修复 servo_message（中文路径）、编译依赖包、跑搜索烟测；通过后再交给实机截图。
set +o nounset 2>/dev/null || true
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

echo "=== [1/4] servo_message 库是否完整 ==="
if compgen -G "$WS/install/servo_message/lib/*.so" > /dev/null; then
  echo "OK: install/servo_message/lib 下已有 .so"
else
  echo "缺失，执行 repair …"
  bash "$WS/tools/repair_servo_message_and_build.sh"
fi

echo "=== [2/4] ldd trace_calculator ==="
set +o nounset 2>/dev/null || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
BIN="$WS/install/trace_energy/lib/trace_energy/trace_calculator"
if ! ldd "$BIN" | grep -q 'libservo_message__rosidl_typesupport_cpp.so =>'; then
  echo "FAIL: 仍找不到 libservo_message，请运行 tools/repair_servo_message_and_build.sh"
  exit 1
fi
echo "OK: libservo_message 已解析"

echo "=== [3/4] 搜索模式烟测（无相机）==="
bash "$WS/tools/run_search_smoke_test.sh"

echo "=== [4/4] aim_command / Qt 话题烟测（无相机）==="
bash "$WS/tools/run_aim_command_smoke_test.sh"

echo "=== 全部通过。实机可执行: cd \"$WS\" && source /opt/ros/humble/setup.bash && source install/setup.bash && ./start_trace_energy.sh ==="
