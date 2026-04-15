#!/usr/bin/env bash
# conda base 等环境常 export 了严格模式；ROS humble 的 setup.bash 会访问未定义变量（如 AMENT_TRACE_SETUP_FILES）
set +o nounset 2>/dev/null || true
set -eo pipefail
#
# 推荐一条命令（含编译）：bash "$(dirname "$0")/go.sh"
# 本脚本：相机 read_image → topic processed_image → trace_calculator（此处追方块+舵机+OpenCV 窗口「Received Image」）
# 无桌面/SSH：SHOW_VIS_WINDOW=false SERVO_PORT=… bash …/start_trace_energy.sh launch_aim_panel:=false

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

if [[ -n "${CONDA_DEFAULT_ENV:-}" ]] && command -v conda >/dev/null 2>&1; then
  echo "[start_trace_energy] 退出 Conda「$CONDA_DEFAULT_ENV」以兼容 ROS2 Humble（Python3.10）" >&2
  # shellcheck disable=SC1091
  source "$(conda info --base 2>/dev/null)/etc/profile.d/conda.sh" 2>/dev/null || true
  i=0
  while [[ -n "${CONDA_SHLVL:-}" && "${CONDA_SHLVL}" != "0" && $i -lt 12 ]]; do
    conda deactivate 2>/dev/null || break
    i=$((i + 1))
  done
  unset i
fi

: "${CAMERA_DEVICE:=}"
: "${SERVO_PORT:=/dev/ttyACM0}"

# launch 依赖 trace_energy + servo_controller；servo_message 需带 .so（中文路径下常需一键修复）
if [[ ! -d "$WS/install/servo_controller" ]]; then
  echo "错误：未安装 servo_controller（launch 需要相机 + 追踪 + 舵机串口）。" >&2
  echo "在本目录执行: bash \"$WS/tools/verify_all.sh\" 或 colcon build --symlink-install" >&2
  exit 1
fi
if [[ ! -f "$WS/install/servo_message/lib/libservo_message__rosidl_typesupport_cpp.so" ]]; then
  echo "错误：servo_message 未完整安装（缺动态库，常见于工作区路径含中文导致 rosidl 失败）。" >&2
  echo "请执行: bash \"$WS/tools/repair_servo_message_and_build.sh\"" >&2
  exit 1
fi

set +o nounset 2>/dev/null || true
source /opt/ros/humble/setup.bash
set +o nounset 2>/dev/null || true
source "$WS/install/setup.bash"

echo "[start_trace_energy] ━━━ 双画面 ━━━  「Camera /processed_image (raw)」= 原始相机流；「Received Image」= 追踪叠加。无 DISPLAY 时自动关闭两路 OpenCV。" >&2
echo "[start_trace_energy] ━━━ 追踪 ━━━  read_image 只发图；trace_calculator 订阅 processed_image 后画框并发 /servo_control。" >&2
echo "[start_trace_energy] ━━━ 舵机 ━━━  话题 /servo_control ；须串口节点正常；无 DISPLAY 时下面会自动关 OpenCV 以免卡住舵机。" >&2
echo "[start_trace_energy] ━━━ 重要 ━━━  本终端要保持运行 launch。另开终端再执行: ros2 topic echo /servo_control" >&2
echo "[start_trace_energy]           若只 echo 不启动 launch，会一直无输出（--once 在等消息）。" >&2
echo "[start_trace_energy] ━━━ 调试图 ━━━ 处理后话题 /trace_debug_image；若在 conda(base) 下 rqt 报 rclpy 错，请:" >&2
echo "[start_trace_energy]           conda deactivate 后开 rqt，或: bash \"$WS/tools/view_trace_debug_image.sh\"" >&2

EXTRA=()
if [[ -n "$CAMERA_DEVICE" ]]; then
  EXTRA+=(camera_device:="$CAMERA_DEVICE" strict_external_camera:=true)
fi
EXTRA+=(servo_port:="$SERVO_PORT")

# 无 DISPLAY 时强制关 OpenCV：否则 imshow/waitKey 可能阻塞整帧，舵机永远不 publish（已把 TX 挪到 imshow 前仍建议关 GUI）
: "${SHOW_VIS_WINDOW:=}"
if [[ -z "$SHOW_VIS_WINDOW" ]] && [[ -z "${DISPLAY:-}" ]]; then
  SHOW_VIS_WINDOW=false
  echo "[start_trace_energy] 未设置 DISPLAY，已自动 SHOW_VIS_WINDOW=false（仅无画面，舵机仍会发）。" >&2
fi
if [[ "$SHOW_VIS_WINDOW" == "false" ]] || [[ "$SHOW_VIS_WINDOW" == "0" ]]; then
  EXTRA+=(show_vis_window:=false show_image_view:=false)
fi

exec ros2 launch trace_energy trace_start.launch.py \
  energy_mode:=large \
  control_enabled:=true \
  track_color_square_only:=true \
  large_square_roi_frac:=0.77 \
  suppress_bright_lights:=false \
  lab_preprocess_blur_ksize:=15 \
  lab_mask_morph_ksize:=5 \
  vis_input_gamma:=0.76 \
  center_circularity_min:=0.74 \
  large_lab_min_area_seg:=50.0 \
  center_circle_area_ratio_min:=0.8 \
  center_rect_area_ratio_max:=0.9 \
  center_max_dist_ratio:=0.38 \
  large_sector_min_area:=220.0 \
  large_sector_max_area_ratio:=0.52 \
  square_approx_poly_eps:=0.024 \
  large_use_hough_center:=false \
  lab_mask_canny_bridge:=false \
  debug_overlay_hough_circles:=false \
  debug_overlay_min_area_rect:=false \
  large_flower_geometry_sector_fallback:=false \
  large_sector_lab_misclass_fallback:=false \
  show_vis_window:=true \
  launch_aim_panel:=true \
  aim_workflow_enabled:=false \
  init_pitch_deg:=340.0 \
  init_yaw_deg:=170.0 \
  startup_hold_frames:=28 \
  large_lab_thresh:=71.5 \
  large_kp:=0.026 \
  large_kd:=0.022 \
  large_err_lp_beta:=0.38 \
  large_deadband_px:=2 \
  large_max_delta_deg:=1.05 \
  large_yaw_gain:=1.48 \
  large_yaw_kp_mult:=1.85 \
  large_yaw_min_step_deg:=0.58 \
  large_near_tgt_yaw_kp_scale:=0.94 \
  large_square_search_yaw_amp_deg:=32.0 \
  servo_pitch_travel_half_span_deg:=40.0 \
  servo_yaw_travel_half_span_deg:=40.0 \
  large_settle_px:=8.0 \
  large_settle_frames:=4 \
  laser_center_ratio_x:=0.5 \
  laser_center_ratio_y:=0.5 \
  laser_aim_offset_x_px:=0.0 \
  laser_aim_offset_y_px:=12.0 \
  large_deadband_yaw_px:=1 \
  servo_id_pitch:=8 \
  servo_id_yaw:=11 \
  swap_pitch_yaw_channels:=false \
  "${EXTRA[@]}" \
  "$@"
