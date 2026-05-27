from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Vision：仅 hsv_blob 色块方块追踪（便于 PID 标定）
    input_topic = LaunchConfiguration("input_topic")
    blob_topic = LaunchConfiguration("blob_topic")
    publish_debug_mask = LaunchConfiguration("publish_debug_mask")
    debug_mask_topic = LaunchConfiguration("debug_mask_topic")
    publish_debug_overlay = LaunchConfiguration("publish_debug_overlay")
    debug_overlay_topic = LaunchConfiguration("debug_overlay_topic")

    hough_blur_ksize = LaunchConfiguration("hough_blur_ksize")
    hough_dp = LaunchConfiguration("hough_dp")
    hough_min_dist_ratio = LaunchConfiguration("hough_min_dist_ratio")
    hough_param1 = LaunchConfiguration("hough_param1")
    hough_param2 = LaunchConfiguration("hough_param2")
    circle_min_area = LaunchConfiguration("circle_min_area")
    circle_max_area_ratio = LaunchConfiguration("circle_max_area_ratio")
    prefer_center = LaunchConfiguration("prefer_center")
    center_bias = LaunchConfiguration("center_bias")
    circle_min_radius_px = LaunchConfiguration("circle_min_radius_px")
    circle_max_radius_ratio = LaunchConfiguration("circle_max_radius_ratio")
    circle_prefer_last_enable = LaunchConfiguration("circle_prefer_last_enable")
    circle_prefer_last_bias = LaunchConfiguration("circle_prefer_last_bias")
    circle_hold_frames = LaunchConfiguration("circle_hold_frames")

    vision_mode = LaunchConfiguration("vision_mode")
    hsv_color_id = LaunchConfiguration("hsv_color_id")
    hsv_h_min = LaunchConfiguration("hsv_h_min")
    hsv_h_max = LaunchConfiguration("hsv_h_max")
    hsv_s_min = LaunchConfiguration("hsv_s_min")
    hsv_s_max = LaunchConfiguration("hsv_s_max")
    hsv_v_min = LaunchConfiguration("hsv_v_min")
    hsv_v_max = LaunchConfiguration("hsv_v_max")
    blob_morph_open = LaunchConfiguration("blob_morph_open")
    blob_morph_close = LaunchConfiguration("blob_morph_close")
    hsv_id_dist_max = LaunchConfiguration("hsv_id_dist_max")
    blob_square_aspect_max = LaunchConfiguration("blob_square_aspect_max")

    # 舵机话题 PD（需另起 servo_uart_node 或 stm32_simple_track_full.launch.py）
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    servo_id_pitch = LaunchConfiguration("servo_id_pitch")
    servo_id_yaw = LaunchConfiguration("servo_id_yaw")
    servo_output_topic = LaunchConfiguration("servo_output_topic")
    init_pitch_deg = LaunchConfiguration("init_pitch_deg")
    init_yaw_deg = LaunchConfiguration("init_yaw_deg")
    servo_pitch_travel_pos_deg = LaunchConfiguration("servo_pitch_travel_pos_deg")
    servo_pitch_travel_neg_deg = LaunchConfiguration("servo_pitch_travel_neg_deg")
    servo_yaw_travel_pos_deg = LaunchConfiguration("servo_yaw_travel_pos_deg")
    servo_yaw_travel_neg_deg = LaunchConfiguration("servo_yaw_travel_neg_deg")
    kp_yaw = LaunchConfiguration("kp_yaw")
    kd_yaw = LaunchConfiguration("kd_yaw")
    kp_pitch = LaunchConfiguration("kp_pitch")
    kd_pitch = LaunchConfiguration("kd_pitch")
    deadband_px = LaunchConfiguration("deadband_px")
    max_step_deg = LaunchConfiguration("max_step_deg")
    lost_hold_frames = LaunchConfiguration("lost_hold_frames")
    pitch_dir_sign = LaunchConfiguration("pitch_dir_sign")
    yaw_dir_sign = LaunchConfiguration("yaw_dir_sign")
    aim_offset_x_px = LaunchConfiguration("aim_offset_x_px")
    aim_offset_y_px = LaunchConfiguration("aim_offset_y_px")
    control_enabled = LaunchConfiguration("control_enabled")
    publish_track_status = LaunchConfiguration("publish_track_status")
    track_status_topic = LaunchConfiguration("track_status_topic")

    vision_node = Node(
        package="stm32_simple_track",
        executable="simple_blob_vision",
        output="both",
        parameters=[
            {
                "input_topic": input_topic,
                "output_topic": blob_topic,
                "publish_debug_mask": publish_debug_mask,
                "debug_mask_topic": debug_mask_topic,
                "publish_debug_overlay": publish_debug_overlay,
                "debug_overlay_topic": debug_overlay_topic,
                "hough_blur_ksize": hough_blur_ksize,
                "hough_dp": hough_dp,
                "hough_min_dist_ratio": hough_min_dist_ratio,
                "hough_param1": hough_param1,
                "hough_param2": hough_param2,
                "circle_min_area": circle_min_area,
                "circle_max_area_ratio": circle_max_area_ratio,
                "prefer_center": prefer_center,
                "center_bias": center_bias,
                "circle_min_radius_px": circle_min_radius_px,
                "circle_max_radius_ratio": circle_max_radius_ratio,
                "circle_prefer_last_enable": circle_prefer_last_enable,
                "circle_prefer_last_bias": circle_prefer_last_bias,
                "circle_hold_frames": circle_hold_frames,
                "vision_mode": vision_mode,
                "hsv_color_id": hsv_color_id,
                "hsv_h_min": hsv_h_min,
                "hsv_h_max": hsv_h_max,
                "hsv_s_min": hsv_s_min,
                "hsv_s_max": hsv_s_max,
                "hsv_v_min": hsv_v_min,
                "hsv_v_max": hsv_v_max,
                "blob_morph_open": blob_morph_open,
                "blob_morph_close": blob_morph_close,
                "hsv_id_dist_max": hsv_id_dist_max,
                "blob_square_aspect_max": blob_square_aspect_max,
            }
        ],
    )

    ctrl_node = Node(
        package="stm32_simple_track",
        executable="simple_pd_servo",
        output="both",
        parameters=[
            {
                "input_topic": blob_topic,
                "control_rate_hz": control_rate_hz,
                "servo_id_pitch": servo_id_pitch,
                "servo_id_yaw": servo_id_yaw,
                "servo_output_topic": servo_output_topic,
                "init_pitch_deg": init_pitch_deg,
                "init_yaw_deg": init_yaw_deg,
                "servo_pitch_travel_pos_deg": servo_pitch_travel_pos_deg,
                "servo_pitch_travel_neg_deg": servo_pitch_travel_neg_deg,
                "servo_yaw_travel_pos_deg": servo_yaw_travel_pos_deg,
                "servo_yaw_travel_neg_deg": servo_yaw_travel_neg_deg,
                "kp_yaw": kp_yaw,
                "kd_yaw": kd_yaw,
                "kp_pitch": kp_pitch,
                "kd_pitch": kd_pitch,
                "deadband_px": deadband_px,
                "max_step_deg": max_step_deg,
                "lost_hold_frames": lost_hold_frames,
                "pitch_dir_sign": pitch_dir_sign,
                "yaw_dir_sign": yaw_dir_sign,
                "aim_offset_x_px": aim_offset_x_px,
                "aim_offset_y_px": aim_offset_y_px,
                "control_enabled": control_enabled,
                "publish_track_status": publish_track_status,
                "track_status_topic": track_status_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/processed_image"),
            DeclareLaunchArgument("blob_topic", default_value="/blob_xy"),
            DeclareLaunchArgument("publish_debug_mask", default_value="false"),
            DeclareLaunchArgument("debug_mask_topic", default_value="/blob_mask"),
            DeclareLaunchArgument("publish_debug_overlay", default_value="true"),
            DeclareLaunchArgument("debug_overlay_topic", default_value="/simple_track/debug_image"),
            DeclareLaunchArgument("hough_blur_ksize", default_value="9"),
            DeclareLaunchArgument("hough_dp", default_value="1.2"),
            DeclareLaunchArgument("hough_min_dist_ratio", default_value="0.10"),
            DeclareLaunchArgument("hough_param1", default_value="120.0"),
            DeclareLaunchArgument("hough_param2", default_value="22.0"),
            DeclareLaunchArgument("circle_min_area", default_value="120.0"),
            DeclareLaunchArgument("circle_max_area_ratio", default_value="0.80"),
            DeclareLaunchArgument("prefer_center", default_value="true"),
            DeclareLaunchArgument("center_bias", default_value="0.002"),
            DeclareLaunchArgument("circle_min_radius_px", default_value="8.0"),
            DeclareLaunchArgument("circle_max_radius_ratio", default_value="0.45"),
            DeclareLaunchArgument("circle_prefer_last_enable", default_value="true"),
            DeclareLaunchArgument("circle_prefer_last_bias", default_value="0.006"),
            DeclareLaunchArgument("circle_hold_frames", default_value="2"),
            DeclareLaunchArgument(
                "vision_mode",
                default_value="hsv_blob",
                description="仅支持 hsv_blob：按 hsv_color_id / hsv_* 追踪方块",
            ),
            DeclareLaunchArgument(
                "hsv_color_id",
                default_value="5",
                description="颜色 id 参照 调试参数/offline_trace.py；默认 5=红色",
            ),
            DeclareLaunchArgument("hsv_h_min", default_value="0"),
            DeclareLaunchArgument("hsv_h_max", default_value="180"),
            DeclareLaunchArgument("hsv_s_min", default_value="80"),
            DeclareLaunchArgument("hsv_s_max", default_value="255"),
            DeclareLaunchArgument("hsv_v_min", default_value="80"),
            DeclareLaunchArgument("hsv_v_max", default_value="255"),
            DeclareLaunchArgument("blob_morph_open", default_value="3"),
            DeclareLaunchArgument("blob_morph_close", default_value="5"),
            DeclareLaunchArgument("hsv_id_dist_max", default_value="5200.0"),
            DeclareLaunchArgument(
                "blob_square_aspect_max",
                default_value="1.45",
                description="外接矩形长宽比上限，接近 1 更像方块",
            ),
            DeclareLaunchArgument("control_rate_hz", default_value="25.0"),
            DeclareLaunchArgument("servo_id_pitch", default_value="11"),
            DeclareLaunchArgument("servo_id_yaw", default_value="8"),
            DeclareLaunchArgument("servo_output_topic", default_value="servo_control"),
            DeclareLaunchArgument("init_pitch_deg", default_value="335.0"),
            DeclareLaunchArgument("init_yaw_deg", default_value="340.0"),
            DeclareLaunchArgument("servo_pitch_travel_pos_deg", default_value="10.0"),
            DeclareLaunchArgument("servo_pitch_travel_neg_deg", default_value="10.0"),
            DeclareLaunchArgument("servo_yaw_travel_pos_deg", default_value="20.0"),
            DeclareLaunchArgument("servo_yaw_travel_neg_deg", default_value="20.0"),
            DeclareLaunchArgument("kp_yaw", default_value="0.010"),
            DeclareLaunchArgument("kd_yaw", default_value="0.006"),
            DeclareLaunchArgument("kp_pitch", default_value="0.010"),
            DeclareLaunchArgument("kd_pitch", default_value="0.006"),
            DeclareLaunchArgument("deadband_px", default_value="4.0"),
            DeclareLaunchArgument("max_step_deg", default_value="0.8"),
            DeclareLaunchArgument("lost_hold_frames", default_value="2"),
            DeclareLaunchArgument("pitch_dir_sign", default_value="1"),
            DeclareLaunchArgument("yaw_dir_sign", default_value="1"),
            DeclareLaunchArgument("aim_offset_x_px", default_value="0.0"),
            DeclareLaunchArgument("aim_offset_y_px", default_value="0.0"),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            DeclareLaunchArgument("publish_track_status", default_value="true"),
            DeclareLaunchArgument("track_status_topic", default_value="/simple_track/status"),
            vision_node,
            ctrl_node,
        ]
    )
