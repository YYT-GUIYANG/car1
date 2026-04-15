#!/usr/bin/env bash
# 在「非 conda」环境下打开 rqt_image_view，订阅 trace 节点发布的处理后图像（默认 /trace_debug_image）。
# 用法:
#   bash tools/view_trace_debug_image.sh
#   bash tools/view_trace_debug_image.sh /my_topic
set +o nounset 2>/dev/null || true
set -eo pipefail

TOPIC="${1:-/trace_debug_image}"

if command -v conda >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "$(conda info --base 2>/dev/null)/etc/profile.d/conda.sh" 2>/dev/null || true
  j=0
  while [[ -n "${CONDA_SHLVL:-}" && "${CONDA_SHLVL}" != "0" && $j -lt 12 ]]; do
    conda deactivate 2>/dev/null || break
    j=$((j + 1))
  done
  unset j
fi

set +o nounset 2>/dev/null || true
source /opt/ros/humble/setup.bash
set +o nounset 2>/dev/null || true

echo ">>> 使用系统 ROS Python 打开 rqt_image_view，重映射 image -> $TOPIC" >&2
exec ros2 run rqt_image_view rqt_image_view --ros-args -r "image:=$TOPIC"
