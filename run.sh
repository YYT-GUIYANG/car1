#!/usr/bin/env bash
# 从本目录一键启动：编译（可跳过）+ start_trace_energy（相机+追踪+舵机+可选 Qt）
# 用法:
#   bash run.sh
# 无桌面/SSH:
#   SHOW_VIS_WINDOW=false launch_aim_panel:=false bash run.sh
# 跳过编译:
#   SKIP_BUILD=1 bash run.sh
set +o nounset 2>/dev/null || true
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ROOT/trace_energy_ws/go.sh" "$@"
