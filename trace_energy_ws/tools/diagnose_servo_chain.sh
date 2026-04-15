#!/usr/bin/env bash
# 舵机不动时逐项自检（不启动相机）。在 trace_energy_ws 根目录: bash tools/diagnose_servo_chain.sh
set +o nounset 2>/dev/null || true
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"
echo "=== 1) trace_calculator 默认参数里 control_enabled（源码 grep）==="
if grep -q 'declare_parameter<bool>("control_enabled", true)' "$WS/src/trace_energy/src/trace_calculator.cpp" 2>/dev/null; then
  echo "OK: C++ 默认 control_enabled=true（ros2 run 未带参也会发 /servo_control）"
else
  echo "WARN: 未匹配到 control_enabled 默认 true，请打开 trace_calculator.cpp 确认"
fi
echo ""
echo "=== 2) launch 默认 control_enabled ==="
grep -A1 '"control_enabled"' "$WS/src/trace_energy/launch/trace_start.launch.py" | head -5 || true
echo ""
echo "=== 3) start_trace_energy.sh 是否传 control_enabled:=true ==="
grep control_enabled "$WS/start_trace_energy.sh" || true
echo ""
echo "=== 4) 串口节点 servonum 合法范围（UARTServo 要求 1~16）==="
grep -n "0 < servonum" "$WS/src/servo_controller/servo_controller/servo_uart_node.py" || true
echo ""
echo "=== 5) 无相机时 trace 不会进 process_image：须 read_image 正常发 /processed_image ==="
echo "    若摄像头严格模式打不开，read_image 会 shutdown，整条链路无舵机更新。"
echo ""
echo "=== 6) 自动化烟测（无舵机硬件）==="
if [[ -f "$WS/tools/verify_all.sh" ]]; then
  bash "$WS/tools/verify_all.sh" && echo "verify_all: PASS" || echo "verify_all: FAIL"
else
  echo "缺少 tools/verify_all.sh"
fi
