import launch # 基础launch模块
import launch_ros # ROS2 launch 扩展模块
from launch.actions import ExecuteProcess  # 执行系统命令
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration  # 启动参数替换

def generate_launch_description():
    energy_mode = LaunchConfiguration("energy_mode")
    control_enabled = LaunchConfiguration("control_enabled")
    laser_center_ratio_x = LaunchConfiguration("laser_center_ratio_x")
    laser_center_ratio_y = LaunchConfiguration("laser_center_ratio_y")
    laser_aim_offset_x_px = LaunchConfiguration("laser_aim_offset_x_px")
    laser_aim_offset_y_px = LaunchConfiguration("laser_aim_offset_y_px")
    large_deadband_yaw_px = LaunchConfiguration("large_deadband_yaw_px")
    strict_external_camera = LaunchConfiguration("strict_external_camera")
    camera_device = LaunchConfiguration("camera_device")
    brightness_adjust_enabled = LaunchConfiguration("brightness_adjust_enabled")
    brightness_scale = LaunchConfiguration("brightness_scale")
    publish_period_ms = LaunchConfiguration("publish_period_ms")
    input_gamma = LaunchConfiguration("input_gamma")
    clahe_l_clip_limit = LaunchConfiguration("clahe_l_clip_limit")
    gray_world_wb_enabled = LaunchConfiguration("gray_world_wb_enabled")
    gray_world_wb_strength = LaunchConfiguration("gray_world_wb_strength")
    camera_v4l_auto_exposure = LaunchConfiguration("camera_v4l_auto_exposure")
    camera_v4l_set_exposure = LaunchConfiguration("camera_v4l_set_exposure")
    camera_v4l_exposure = LaunchConfiguration("camera_v4l_exposure")
    camera_v4l_auto_wb = LaunchConfiguration("camera_v4l_auto_wb")
    camera_v4l_wb_temperature = LaunchConfiguration("camera_v4l_wb_temperature")
    camera_v4l_gain = LaunchConfiguration("camera_v4l_gain")
    servo_baudrate = LaunchConfiguration("servo_baudrate")
    small_lead_ms = LaunchConfiguration("small_lead_ms")
    small_kp = LaunchConfiguration("small_kp")
    small_kd = LaunchConfiguration("small_kd")
    small_deadband = LaunchConfiguration("small_deadband")
    small_max_delta_deg = LaunchConfiguration("small_max_delta_deg")
    small_settle_px = LaunchConfiguration("small_settle_px")
    small_settle_frames = LaunchConfiguration("small_settle_frames")
    small_min_step_deg = LaunchConfiguration("small_min_step_deg")
    track_same_color_square_only = LaunchConfiguration("track_same_color_square_only")
    track_color_square_only = LaunchConfiguration("track_color_square_only")
    large_reject_purple_hub_disk = LaunchConfiguration("large_reject_purple_hub_disk")
    large_reject_red_blob = LaunchConfiguration("large_reject_red_blob")
    large_square_roi_frac = LaunchConfiguration("large_square_roi_frac")
    large_square_spin_lead = LaunchConfiguration("large_square_spin_lead")
    large_square_predict_coast_frames = LaunchConfiguration("large_square_predict_coast_frames")
    large_square_predict_lead_ms = LaunchConfiguration("large_square_predict_lead_ms")
    large_square_predict_arm_frames = LaunchConfiguration("large_square_predict_arm_frames")
    large_square_predict_vel_beta = LaunchConfiguration("large_square_predict_vel_beta")
    small_center_hub_dist_frac = LaunchConfiguration("small_center_hub_dist_frac")
    small_color_only_after_lock_enable = LaunchConfiguration("small_color_only_after_lock_enable")
    small_color_only_arm_frames = LaunchConfiguration("small_color_only_arm_frames")
    center_color_relock_after_miss_frames = LaunchConfiguration("center_color_relock_after_miss_frames")
    disable_blob_fallback = LaunchConfiguration("disable_blob_fallback")
    small_search_enter_frames = LaunchConfiguration("small_search_enter_frames")
    small_search_yaw_amp_deg = LaunchConfiguration("small_search_yaw_amp_deg")
    small_search_pitch_amp_deg = LaunchConfiguration("small_search_pitch_amp_deg")
    small_search_phase_step = LaunchConfiguration("small_search_phase_step")
    small_search_pitch_phase_step = LaunchConfiguration("small_search_pitch_phase_step")
    small_lab_thresh = LaunchConfiguration("small_lab_thresh")
    large_lab_thresh = LaunchConfiguration("large_lab_thresh")
    large_kp = LaunchConfiguration("large_kp")
    large_kd = LaunchConfiguration("large_kd")
    large_ki_yaw = LaunchConfiguration("large_ki_yaw")
    large_ki_pitch = LaunchConfiguration("large_ki_pitch")
    small_ki_yaw = LaunchConfiguration("small_ki_yaw")
    small_ki_pitch = LaunchConfiguration("small_ki_pitch")
    ctrl_integ_max_px_s = LaunchConfiguration("ctrl_integ_max_px_s")
    trace_proc_width_cap_large = LaunchConfiguration("trace_proc_width_cap_large")
    trace_proc_width_cap_small = LaunchConfiguration("trace_proc_width_cap_small")
    large_err_lp_beta = LaunchConfiguration("large_err_lp_beta")
    large_err_lp_beta_yaw = LaunchConfiguration("large_err_lp_beta_yaw")
    large_deadband_px = LaunchConfiguration("large_deadband_px")
    large_max_delta_deg = LaunchConfiguration("large_max_delta_deg")
    large_settle_px = LaunchConfiguration("large_settle_px")
    large_settle_frames = LaunchConfiguration("large_settle_frames")
    large_yaw_gain = LaunchConfiguration("large_yaw_gain")
    large_yaw_kp_mult = LaunchConfiguration("large_yaw_kp_mult")
    large_yaw_min_step_deg = LaunchConfiguration("large_yaw_min_step_deg")
    large_near_tgt_yaw_kp_scale = LaunchConfiguration("large_near_tgt_yaw_kp_scale")
    large_square_search_yaw_amp_deg = LaunchConfiguration("large_square_search_yaw_amp_deg")
    init_pitch_deg = LaunchConfiguration("init_pitch_deg")
    init_yaw_deg = LaunchConfiguration("init_yaw_deg")
    startup_hold_frames = LaunchConfiguration("startup_hold_frames")
    publish_tracking_debug = LaunchConfiguration("publish_tracking_debug")
    show_vis_window = LaunchConfiguration("show_vis_window")
    runtime_tuning_enable = LaunchConfiguration("runtime_tuning_enable")
    aim_workflow_enabled = LaunchConfiguration("aim_workflow_enabled")
    stable_online_mode = LaunchConfiguration("stable_online_mode")
    stable_target_hold_frames = LaunchConfiguration("stable_target_hold_frames")
    stable_transition_blend_frames = LaunchConfiguration("stable_transition_blend_frames")
    stable_disable_predict = LaunchConfiguration("stable_disable_predict")
    stable_disable_search = LaunchConfiguration("stable_disable_search")
    stable_fixed_dt_s = LaunchConfiguration("stable_fixed_dt_s")
    large_purple_family_match = LaunchConfiguration("large_purple_family_match")
    launch_aim_panel = LaunchConfiguration("launch_aim_panel")
    large_blob_hsv_relabel = LaunchConfiguration("large_blob_hsv_relabel")
    large_blob_hsv_max_dist_sq = LaunchConfiguration("large_blob_hsv_max_dist_sq")
    large_center_shape_first = LaunchConfiguration("large_center_shape_first")
    center_square_area_rel_tol = LaunchConfiguration("center_square_area_rel_tol")
    center_square_diam_side_rel_tol = LaunchConfiguration("center_square_diam_side_rel_tol")
    square_max_dist_center_radius = LaunchConfiguration("square_max_dist_center_radius")
    fallback_blob_max_area_ratio = LaunchConfiguration("fallback_blob_max_area_ratio")
    fallback_blob_min_dist_center_radius = LaunchConfiguration("fallback_blob_min_dist_center_radius")
    fallback_blob_max_dist_center_radius = LaunchConfiguration("fallback_blob_max_dist_center_radius")
    center_circularity_min = LaunchConfiguration("center_circularity_min")
    large_center_hsv_max_dist_sq = LaunchConfiguration("large_center_hsv_max_dist_sq")
    large_square_min_contour_area = LaunchConfiguration("large_square_min_contour_area")
    large_lab_min_area_seg = LaunchConfiguration("large_lab_min_area_seg")
    center_circle_area_ratio_min = LaunchConfiguration("center_circle_area_ratio_min")
    center_rect_area_ratio_max = LaunchConfiguration("center_rect_area_ratio_max")
    center_max_dist_ratio = LaunchConfiguration("center_max_dist_ratio")
    large_sector_min_area = LaunchConfiguration("large_sector_min_area")
    large_sector_max_area_ratio = LaunchConfiguration("large_sector_max_area_ratio")
    square_approx_poly_eps = LaunchConfiguration("square_approx_poly_eps")
    large_flower_geometry_sector_fallback = LaunchConfiguration("large_flower_geometry_sector_fallback")
    large_sector_lab_misclass_fallback = LaunchConfiguration("large_sector_lab_misclass_fallback")
    publish_center_color_id = LaunchConfiguration("publish_center_color_id")
    suppress_bright_lights = LaunchConfiguration("suppress_bright_lights")
    bright_hsv_v_min = LaunchConfiguration("bright_hsv_v_min")
    bright_hsv_s_max = LaunchConfiguration("bright_hsv_s_max")
    bright_bgr_channel_min = LaunchConfiguration("bright_bgr_channel_min")
    bright_mask_dilate = LaunchConfiguration("bright_mask_dilate")
    bright_inpaint_radius = LaunchConfiguration("bright_inpaint_radius")
    bright_inpaint_max_area_frac = LaunchConfiguration("bright_inpaint_max_area_frac")
    center_roi_zoom = LaunchConfiguration("center_roi_zoom")
    swap_pitch_yaw_channels = LaunchConfiguration("swap_pitch_yaw_channels")
    servo_id_pitch = LaunchConfiguration("servo_id_pitch")
    servo_id_yaw = LaunchConfiguration("servo_id_yaw")
    servo_pitch_min_deg = LaunchConfiguration("servo_pitch_min_deg")
    servo_pitch_max_deg = LaunchConfiguration("servo_pitch_max_deg")
    servo_yaw_min_deg = LaunchConfiguration("servo_yaw_min_deg")
    servo_yaw_max_deg = LaunchConfiguration("servo_yaw_max_deg")
    lab_preprocess_blur_ksize = LaunchConfiguration("lab_preprocess_blur_ksize")
    lab_mask_morph_ksize = LaunchConfiguration("lab_mask_morph_ksize")
    vis_input_gamma = LaunchConfiguration("vis_input_gamma")
    vis_clahe_l_clip_limit = LaunchConfiguration("vis_clahe_l_clip_limit")
    lab_mask_canny_bridge = LaunchConfiguration("lab_mask_canny_bridge")
    lab_canny_thresh1 = LaunchConfiguration("lab_canny_thresh1")
    lab_canny_thresh2 = LaunchConfiguration("lab_canny_thresh2")
    lab_canny_dilate_px = LaunchConfiguration("lab_canny_dilate_px")
    lab_canny_bridge_mask_dilate = LaunchConfiguration("lab_canny_bridge_mask_dilate")
    servo_pitch_travel_half_span_deg = LaunchConfiguration("servo_pitch_travel_half_span_deg")
    servo_yaw_travel_half_span_deg = LaunchConfiguration("servo_yaw_travel_half_span_deg")
    servo_pitch_travel_pos_deg = LaunchConfiguration("servo_pitch_travel_pos_deg")
    servo_pitch_travel_neg_deg = LaunchConfiguration("servo_pitch_travel_neg_deg")
    servo_yaw_travel_pos_deg = LaunchConfiguration("servo_yaw_travel_pos_deg")
    servo_yaw_travel_neg_deg = LaunchConfiguration("servo_yaw_travel_neg_deg")
    servo_angle_smooth_beta = LaunchConfiguration("servo_angle_smooth_beta")
    show_image_view = LaunchConfiguration("show_image_view")
    show_trace_debug_image_view = LaunchConfiguration("show_trace_debug_image_view")
    publish_trace_debug_image = LaunchConfiguration("publish_trace_debug_image")
    trace_debug_image_topic = LaunchConfiguration("trace_debug_image_topic")
    debug_overlay_hough_circles = LaunchConfiguration("debug_overlay_hough_circles")
    debug_overlay_min_area_rect = LaunchConfiguration("debug_overlay_min_area_rect")

    webcam_node = launch_ros.actions.Node(
        package='trace_energy',
        executable='read_image',
        parameters=[{
            "strict_external_camera": strict_external_camera,
            "camera_device": camera_device,
            "brightness_adjust_enabled": brightness_adjust_enabled,
            "brightness_scale": brightness_scale,
            "publish_period_ms": publish_period_ms,
            "input_gamma": input_gamma,
            "clahe_l_clip_limit": clahe_l_clip_limit,
            "gray_world_wb_enabled": gray_world_wb_enabled,
            "gray_world_wb_strength": gray_world_wb_strength,
            "camera_v4l_auto_exposure": camera_v4l_auto_exposure,
            "camera_v4l_set_exposure": camera_v4l_set_exposure,
            "camera_v4l_exposure": camera_v4l_exposure,
            "camera_v4l_auto_wb": camera_v4l_auto_wb,
            "camera_v4l_wb_temperature": camera_v4l_wb_temperature,
            "camera_v4l_gain": camera_v4l_gain,
        }],
        output='both',  
    )   

    trace_node = launch_ros.actions.Node(
        package='trace_energy',
        executable='trace_calculator',
        parameters=[{
            "energy_mode": energy_mode,
            "control_enabled": control_enabled,
            "laser_center_ratio_x": laser_center_ratio_x,
            "laser_center_ratio_y": laser_center_ratio_y,
            "laser_aim_offset_x_px": laser_aim_offset_x_px,
            "laser_aim_offset_y_px": laser_aim_offset_y_px,
            "large_deadband_yaw_px": large_deadband_yaw_px,
            "small_lead_ms": small_lead_ms,
            "small_kp": small_kp,
            "small_kd": small_kd,
            "small_ki_yaw": small_ki_yaw,
            "small_ki_pitch": small_ki_pitch,
            "small_deadband": small_deadband,
            "small_max_delta_deg": small_max_delta_deg,
            "small_settle_px": small_settle_px,
            "small_settle_frames": small_settle_frames,
            "small_min_step_deg": small_min_step_deg,
            "track_same_color_square_only": track_same_color_square_only,
            "track_color_square_only": track_color_square_only,
            "large_reject_purple_hub_disk": large_reject_purple_hub_disk,
            "large_reject_red_blob": large_reject_red_blob,
            "large_square_roi_frac": large_square_roi_frac,
            "large_square_spin_lead": large_square_spin_lead,
            "large_square_predict_coast_frames": large_square_predict_coast_frames,
            "large_square_predict_lead_ms": large_square_predict_lead_ms,
            "large_square_predict_arm_frames": large_square_predict_arm_frames,
            "large_square_predict_vel_beta": large_square_predict_vel_beta,
            "small_center_hub_dist_frac": small_center_hub_dist_frac,
            "small_color_only_after_lock_enable": small_color_only_after_lock_enable,
            "small_color_only_arm_frames": small_color_only_arm_frames,
            "center_color_relock_after_miss_frames": center_color_relock_after_miss_frames,
            "disable_blob_fallback": disable_blob_fallback,
            "small_search_enter_frames": small_search_enter_frames,
            "small_search_yaw_amp_deg": small_search_yaw_amp_deg,
            "small_search_pitch_amp_deg": small_search_pitch_amp_deg,
            "small_search_phase_step": small_search_phase_step,
            "small_search_pitch_phase_step": small_search_pitch_phase_step,
            "small_lab_thresh": small_lab_thresh,
            "large_lab_thresh": large_lab_thresh,
            "large_kp": large_kp,
            "large_kd": large_kd,
            "large_ki_yaw": large_ki_yaw,
            "large_ki_pitch": large_ki_pitch,
            "ctrl_integ_max_px_s": ctrl_integ_max_px_s,
            "trace_proc_width_cap_large": trace_proc_width_cap_large,
            "trace_proc_width_cap_small": trace_proc_width_cap_small,
            "large_err_lp_beta": large_err_lp_beta,
            "large_err_lp_beta_yaw": large_err_lp_beta_yaw,
            "large_deadband_px": large_deadband_px,
            "large_max_delta_deg": large_max_delta_deg,
            "large_settle_px": large_settle_px,
            "large_settle_frames": large_settle_frames,
            "large_yaw_gain": large_yaw_gain,
            "large_yaw_kp_mult": large_yaw_kp_mult,
            "large_yaw_min_step_deg": large_yaw_min_step_deg,
            "large_near_tgt_yaw_kp_scale": large_near_tgt_yaw_kp_scale,
            "large_square_search_yaw_amp_deg": large_square_search_yaw_amp_deg,
            "init_pitch_deg": init_pitch_deg,
            "init_yaw_deg": init_yaw_deg,
            "startup_hold_frames": startup_hold_frames,
            "publish_tracking_debug": publish_tracking_debug,
            "show_vis_window": show_vis_window,
            "runtime_tuning_enable": runtime_tuning_enable,
            "publish_trace_debug_image": publish_trace_debug_image,
            "trace_debug_image_topic": trace_debug_image_topic,
            "debug_overlay_hough_circles": debug_overlay_hough_circles,
            "debug_overlay_min_area_rect": debug_overlay_min_area_rect,
            "aim_workflow_enabled": aim_workflow_enabled,
            "stable_online_mode": stable_online_mode,
            "stable_target_hold_frames": stable_target_hold_frames,
            "stable_transition_blend_frames": stable_transition_blend_frames,
            "stable_disable_predict": stable_disable_predict,
            "stable_disable_search": stable_disable_search,
            "stable_fixed_dt_s": stable_fixed_dt_s,
            "large_purple_family_match": large_purple_family_match,
            "large_blob_hsv_relabel": large_blob_hsv_relabel,
            "large_blob_hsv_max_dist_sq": large_blob_hsv_max_dist_sq,
            "large_center_shape_first": large_center_shape_first,
            "center_square_area_rel_tol": center_square_area_rel_tol,
            "center_square_diam_side_rel_tol": center_square_diam_side_rel_tol,
            "square_max_dist_center_radius": square_max_dist_center_radius,
            "fallback_blob_max_area_ratio": fallback_blob_max_area_ratio,
            "fallback_blob_min_dist_center_radius": fallback_blob_min_dist_center_radius,
            "fallback_blob_max_dist_center_radius": fallback_blob_max_dist_center_radius,
            "center_circularity_min": center_circularity_min,
            "large_center_hsv_max_dist_sq": large_center_hsv_max_dist_sq,
            "large_square_min_contour_area": large_square_min_contour_area,
            "large_lab_min_area_seg": large_lab_min_area_seg,
            "center_circle_area_ratio_min": center_circle_area_ratio_min,
            "center_rect_area_ratio_max": center_rect_area_ratio_max,
            "center_max_dist_ratio": center_max_dist_ratio,
            "large_sector_min_area": large_sector_min_area,
            "large_sector_max_area_ratio": large_sector_max_area_ratio,
            "square_approx_poly_eps": square_approx_poly_eps,
            "large_flower_geometry_sector_fallback": large_flower_geometry_sector_fallback,
            "large_sector_lab_misclass_fallback": large_sector_lab_misclass_fallback,
            "publish_center_color_id": publish_center_color_id,
            "suppress_bright_lights": suppress_bright_lights,
            "bright_hsv_v_min": bright_hsv_v_min,
            "bright_hsv_s_max": bright_hsv_s_max,
            "bright_bgr_channel_min": bright_bgr_channel_min,
            "bright_mask_dilate": bright_mask_dilate,
            "bright_inpaint_radius": bright_inpaint_radius,
            "bright_inpaint_max_area_frac": bright_inpaint_max_area_frac,
            "center_roi_zoom": center_roi_zoom,
            "swap_pitch_yaw_channels": swap_pitch_yaw_channels,
            "servo_id_pitch": servo_id_pitch,
            "servo_id_yaw": servo_id_yaw,
            "servo_pitch_min_deg": servo_pitch_min_deg,
            "servo_pitch_max_deg": servo_pitch_max_deg,
            "servo_yaw_min_deg": servo_yaw_min_deg,
            "servo_yaw_max_deg": servo_yaw_max_deg,
            "lab_preprocess_blur_ksize": lab_preprocess_blur_ksize,
            "lab_mask_morph_ksize": lab_mask_morph_ksize,
            "vis_input_gamma": vis_input_gamma,
            "vis_clahe_l_clip_limit": vis_clahe_l_clip_limit,
            "lab_mask_canny_bridge": lab_mask_canny_bridge,
            "lab_canny_thresh1": lab_canny_thresh1,
            "lab_canny_thresh2": lab_canny_thresh2,
            "lab_canny_dilate_px": lab_canny_dilate_px,
            "lab_canny_bridge_mask_dilate": lab_canny_bridge_mask_dilate,
            "servo_pitch_travel_half_span_deg": servo_pitch_travel_half_span_deg,
            "servo_yaw_travel_half_span_deg": servo_yaw_travel_half_span_deg,
            "servo_pitch_travel_pos_deg": servo_pitch_travel_pos_deg,
            "servo_pitch_travel_neg_deg": servo_pitch_travel_neg_deg,
            "servo_yaw_travel_pos_deg": servo_yaw_travel_pos_deg,
            "servo_yaw_travel_neg_deg": servo_yaw_travel_neg_deg,
            "servo_angle_smooth_beta": servo_angle_smooth_beta,
        }],
        output='both',  
    )   

    aim_panel_node = launch_ros.actions.Node(
        package="trace_energy",
        executable="aim_control_panel",
        condition=IfCondition(launch_aim_panel),
        output="screen",
    )

    control_node = launch_ros.actions.Node(
        package='servo_controller',
        executable='servo_uart_node',
        additional_env={
            "SERVO_BAUDRATE": servo_baudrate,
            "SERVO_PORT": LaunchConfiguration("servo_port"),
            "SERVO_ANGLE_MIN": "0",
            "SERVO_ANGLE_MAX": "360",
        },
        remappings=[
            ("servo_control", "/servo_control"),
        ],
        output='both',  
    )   

    # 包内第二路预览：原始 /processed_image（无需 apt 安装 ros-humble-image-view）
    raw_preview_node = launch_ros.actions.Node(
        package="trace_energy",
        executable="processed_image_preview",
        name="processed_image_preview",
        parameters=[{
            "image_topic": "/processed_image",
            "window_name": "Camera /processed_image (raw)",
        }],
        condition=IfCondition(show_image_view),
        output="screen",
    )

    trace_debug_preview_node = launch_ros.actions.Node(
        package="trace_energy",
        executable="processed_image_preview",
        name="trace_debug_image_preview",
        parameters=[{
            "image_topic": trace_debug_image_topic,
            "window_name": "TRACE /trace_debug_image (processed Lab+Hough+track)",
        }],
        condition=IfCondition(show_trace_debug_image_view),
        output="screen",
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "energy_mode",
            default_value="large",
            choices=["small", "large"],
            description="能量机关模式：大能量机关用 large（紫系 id0/3/6）；小能量用 small（蓝 id9）",
        ),
        DeclareLaunchArgument(
            "control_enabled",
            default_value="true",
            choices=["true", "false"],
            description="是否开启舵机控制输出",
        ),
        DeclareLaunchArgument(
            "laser_center_ratio_x",
            default_value="0.5",
            description="激光/准星中心X比例(0~1)",
        ),
        DeclareLaunchArgument(
            "laser_center_ratio_y",
            default_value="0.5",
            description="激光/准星中心Y比例(0~1)",
        ),
        DeclareLaunchArgument(
            "laser_aim_offset_x_px",
            default_value="0.0",
            description="激光相对画面光轴中心的像素偏移 X（右为正）；与 ratio 叠加后参与瞄准误差",
        ),
        DeclareLaunchArgument(
            "laser_aim_offset_y_px",
            default_value="34.0",
            description="红外/激光相对画面光轴中心的像素偏移 Y（下为正；约4cm 在640级画面的粗估值，请实测光斑微调）",
        ),
        DeclareLaunchArgument(
            "large_deadband_yaw_px",
            default_value="1",
            description="大能量+纯色块：水平像素死区上限（与 large_deadband_px 取较小值），略松俯仰、紧偏航便于左右跟靶",
        ),
        DeclareLaunchArgument(
            "strict_external_camera",
            default_value="true",
            choices=["true", "false"],
            description="是否仅允许外接USB相机路径",
        ),
        DeclareLaunchArgument(
            "camera_device",
            default_value="",
            description="可选：强制锁定相机设备路径（如 /dev/v4l/by-id/... 或 /dev/video2）",
        ),
        DeclareLaunchArgument(
            "brightness_adjust_enabled",
            default_value="false",
            choices=["true", "false"],
            description="是否启用软件亮度调整",
        ),
        DeclareLaunchArgument(
            "brightness_scale",
            default_value="1.0",
            description="亮度缩放系数，仅在开启亮度调整时生效",
        ),
        DeclareLaunchArgument(
            "publish_period_ms",
            default_value="10",
            description="read_image 发布 /processed_image 的周期(ms)；增大可降低 CPU/带宽负载",
        ),
        DeclareLaunchArgument(
            "input_gamma",
            default_value="1.0",
            description="采集端伽马(1.0=不变)；略小于1提亮阴影，略大于1压高光",
        ),
        DeclareLaunchArgument(
            "clahe_l_clip_limit",
            default_value="0.0",
            description="采集端 Lab L 通道 CLAHE 对比度限制，0 关闭；现场可试 2.0~4.0",
        ),
        DeclareLaunchArgument(
            "gray_world_wb_enabled",
            default_value="false",
            choices=["true", "false"],
            description="read_image：灰世界白平衡（大光照变化时可试；与 gamma/CLAHE 顺序：WB→gamma→CLAHE）",
        ),
        DeclareLaunchArgument(
            "gray_world_wb_strength",
            default_value="0.72",
            description="灰世界强度 0~1；0 等价于关，1 为完全按通道均值拉齐",
        ),
        DeclareLaunchArgument(
            "camera_v4l_auto_exposure",
            default_value="-1",
            description="read_image V4L：-1 不改；1 常为手动曝光；3 常为自动（以驱动为准）。颜色乱跳可试手动+关 AWB",
        ),
        DeclareLaunchArgument(
            "camera_v4l_set_exposure",
            default_value="false",
            choices=["true", "false"],
            description="是否写入 camera_v4l_exposure（通常需 camera_v4l_auto_exposure:=1）",
        ),
        DeclareLaunchArgument(
            "camera_v4l_exposure",
            default_value="-6.0",
            description="read_image CAP_PROP_EXPOSURE（UVC 常为负值档位，需实测）",
        ),
        DeclareLaunchArgument(
            "camera_v4l_auto_wb",
            default_value="-1",
            description="read_image 自动白平衡：-1 不改；0 关；1 开",
        ),
        DeclareLaunchArgument(
            "camera_v4l_wb_temperature",
            default_value="-1",
            description="read_image 色温 K，>=0 时尝试设置（相机不支持则无效）",
        ),
        DeclareLaunchArgument(
            "camera_v4l_gain",
            default_value="-1.0",
            description="read_image 增益，>=0 设置；-1 不改",
        ),
        DeclareLaunchArgument(
            "servo_baudrate",
            default_value="9600",
            description="舵机串口波特率（与控制板保持一致）",
        ),
        DeclareLaunchArgument(
            "small_lead_ms",
            default_value="28.0",
            description="small 模式目标预测提前量（ms）",
        ),
        DeclareLaunchArgument(
            "small_kp",
            default_value="0.013",
            description="small 模式控制比例系数",
        ),
        DeclareLaunchArgument(
            "small_kd",
            default_value="0.012",
            description="small 模式控制微分系数",
        ),
        DeclareLaunchArgument(
            "small_ki_yaw",
            default_value="0.0",
            description="small 积分项偏航：消除稳态像素偏差；默认 0 保持原 PD；过大易振荡",
        ),
        DeclareLaunchArgument(
            "small_ki_pitch",
            default_value="0.0",
            description="small 积分项俯仰",
        ),
        DeclareLaunchArgument(
            "small_deadband",
            default_value="12",
            description="small 模式控制死区像素",
        ),
        DeclareLaunchArgument(
            "small_max_delta_deg",
            default_value="0.50",
            description="small 模式每帧最大角度步进",
        ),
        DeclareLaunchArgument(
            "small_settle_px",
            default_value="14.0",
            description="small 模式误差进入稳定锁定阈值（px）",
        ),
        DeclareLaunchArgument(
            "small_settle_frames",
            default_value="5",
            description="small 模式进入稳定锁定所需连续帧",
        ),
        DeclareLaunchArgument(
            "small_min_step_deg",
            default_value="0.18",
            description="small 模式最小有效步进角（度）",
        ),
        DeclareLaunchArgument(
            "track_same_color_square_only",
            default_value="false",
            choices=["true", "false"],
            description="small 模式是否优先同色方块作为目标（track_color_square_only=false 时生效）",
        ),
        DeclareLaunchArgument(
            "track_color_square_only",
            default_value="false",
            choices=["true", "false"],
            description="true：只追色块（大能量紫/小能量蓝），光学中心作转盘参考；false：原中心圆+扇区",
        ),
        DeclareLaunchArgument(
            "large_reject_purple_hub_disk",
            default_value="true",
            choices=["true", "false"],
            description="大能量+纯色块：用形状/位置排除光学中心紫色圆盘，减少与扇区紫块误判",
        ),
        DeclareLaunchArgument(
            "large_reject_red_blob",
            default_value="true",
            choices=["true", "false"],
            description="大能量：轮廓平均色像大红/橙红时否决（Lab 易把红判成紫系 id）",
        ),
        DeclareLaunchArgument(
            "large_square_roi_frac",
            default_value="0.70",
            description="大能量+纯色块：中心 ROI 边长占画面比例（与 offline_trace roi_frac 对齐）；过小易裁掉边缘扇区，过大易进杂景",
        ),
        DeclareLaunchArgument(
            "large_square_spin_lead",
            default_value="0.14",
            description="大能量+纯色块：旋转跟踪速度超前系数(0~0.25)，越大越跟手但易抖",
        ),
        DeclareLaunchArgument(
            "large_square_predict_coast_frames",
            default_value="12",
            description="大能量+纯色块：已武装预测后连续丢检测的最大帧数，此窗口内用速度外推/冻结并抑制 SEARCH",
        ),
        DeclareLaunchArgument(
            "large_square_predict_lead_ms",
            default_value="58.0",
            description="大能量+纯色块：丢检测时每帧外推 smooth_target 的时间提前量(毫秒)",
        ),
        DeclareLaunchArgument(
            "large_square_predict_arm_frames",
            default_value="8",
            description="大能量+纯色块：连续有效识别达到该帧数后才允许短时预测续航",
        ),
        DeclareLaunchArgument(
            "large_square_predict_vel_beta",
            default_value="0.42",
            description="大能量+纯色块：像素速度 EMA 系数(0~1)，越大越跟瞬时速度",
        ),
        DeclareLaunchArgument(
            "small_center_hub_dist_frac",
            default_value="0.32",
            description="small 模式：中心圆盘被标成 square 时，允许距画面中心的最大距离(×min边)",
        ),
        DeclareLaunchArgument(
            "small_color_only_after_lock_enable",
            default_value="false",
            choices=["true", "false"],
            description="small 模式：稳定锁定同色后进入仅追同色方块阶段（不再依赖中心圆识别）",
        ),
        DeclareLaunchArgument(
            "small_color_only_arm_frames",
            default_value="6",
            description="small 模式：进入仅追同色方块阶段所需的中心色稳定帧数",
        ),
        DeclareLaunchArgument(
            "center_color_relock_after_miss_frames",
            default_value="10",
            description="已锁中心色后，连续不一致达到该帧数才重锁颜色（避免每帧重判抖动）",
        ),
        DeclareLaunchArgument(
            "disable_blob_fallback",
            default_value="true",
            choices=["true", "false"],
            description="true：禁用同色blob回退，只保留同色四边形主路径",
        ),
        DeclareLaunchArgument(
            "small_search_enter_frames",
            default_value="3",
            description="连续未锁定(无中心或无扇区目标)达到该帧数后进入搜索模式",
        ),
        DeclareLaunchArgument(
            "small_search_yaw_amp_deg",
            default_value="24.0",
            description="搜索模式偏航摆动幅度(度)，中心为 init_yaw",
        ),
        DeclareLaunchArgument(
            "small_search_pitch_amp_deg",
            default_value="12.0",
            description="small 搜索模式俯仰摆动幅度(度)，中心为 init_pitch",
        ),
        DeclareLaunchArgument(
            "small_search_phase_step",
            default_value="0.10",
            description="搜索模式偏航相位每帧增量(越大扫得越快)",
        ),
        DeclareLaunchArgument(
            "small_search_pitch_phase_step",
            default_value="0.035",
            description="搜索模式俯仰相位每帧增量(建议小于偏航，更慢)",
        ),
        DeclareLaunchArgument(
            "small_lab_thresh",
            default_value="74.2",
            description="small 模式(蓝色 id9) Lab 距离阈值，一般 54~62",
        ),
        DeclareLaunchArgument(
            "large_lab_thresh",
            default_value="74.2",
            description="large Lab 阈值（与 offline_trace lab_thresh 对齐）；略放宽利于召回，误检多再略降",
        ),
        DeclareLaunchArgument(
            "large_kp",
            default_value="0.022",
            description="大能量 PD：比例系数（像素误差→度）",
        ),
        DeclareLaunchArgument(
            "large_kd",
            default_value="0.020",
            description="大能量 PD：微分系数",
        ),
        DeclareLaunchArgument(
            "large_ki_yaw",
            default_value="0.0",
            description="大能量积分偏航（像素误差积分×Ki→度）；默认 0；先调好 P/D 再微量加",
        ),
        DeclareLaunchArgument(
            "large_ki_pitch",
            default_value="0.0",
            description="大能量积分俯仰",
        ),
        DeclareLaunchArgument(
            "ctrl_integ_max_px_s",
            default_value="120.0",
            description="PID 积分抗饱和：|∫e·dt| 上限（像素·秒量级，与 Ki 配合调）",
        ),
        DeclareLaunchArgument(
            "trace_proc_width_cap_large",
            default_value="960",
            description="大能量：Lab/跟踪前将图像宽缩放到不超过该值（降分辨率减负）",
        ),
        DeclareLaunchArgument(
            "trace_proc_width_cap_small",
            default_value="960",
            description="小能量：同上宽度上限",
        ),
        DeclareLaunchArgument(
            "large_err_lp_beta",
            default_value="0.38",
            description="大能量误差低通系数（俯仰等；纯色块水平另见 large_err_lp_beta_yaw）",
        ),
        DeclareLaunchArgument(
            "large_err_lp_beta_yaw",
            default_value="0.52",
            description="大能量+纯色块：水平像素误差低通（越大越跟手，易抖）",
        ),
        DeclareLaunchArgument(
            "large_deadband_px",
            default_value="3",
            description="大能量像素死区",
        ),
        DeclareLaunchArgument(
            "large_max_delta_deg",
            default_value="1.05",
            description="大能量每帧舵机角变化上限(度)；配合 travel±20 与小 beta 更平滑",
        ),
        DeclareLaunchArgument(
            "large_settle_px",
            default_value="8.0",
            description="大能量判定稳态的像素误差阈值",
        ),
        DeclareLaunchArgument(
            "large_settle_frames",
            default_value="4",
            description="大能量稳态需连续帧数",
        ),
        DeclareLaunchArgument(
            "large_yaw_gain",
            default_value="1.58",
            description="大能量偏航额外增益（11号水平舵机易偏软）",
        ),
        DeclareLaunchArgument(
            "large_yaw_kp_mult",
            default_value="1.75",
            description="大能量偏航 Kp 相对俯仰的倍率（整度串口+去重时过小 u_yaw 会导致 11 号不转）",
        ),
        DeclareLaunchArgument(
            "large_yaw_min_step_deg",
            default_value="0.52",
            description="大能量偏航最小步进(度)，克服静摩擦/整数角粘滞",
        ),
        DeclareLaunchArgument(
            "large_near_tgt_yaw_kp_scale",
            default_value="0.92",
            description="近距(hit<40px)时偏航 Kp 软化系数；俯仰仍用满幅 Kp/步进上限",
        ),
        DeclareLaunchArgument(
            "large_square_search_yaw_amp_deg",
            default_value="32.0",
            description="大能量纯色块搜索偏航摆幅(度)；不宜超过 travel半宽太多",
        ),
        DeclareLaunchArgument(
            "init_pitch_deg",
            default_value="335.0",
            description="上电俯仰初值(度，发到 servo_id_pitch 默认11号)",
        ),
        DeclareLaunchArgument(
            "init_yaw_deg",
            default_value="340.0",
            description="上电偏航初值(度)；360°舵机可用 0~360",
        ),
        DeclareLaunchArgument(
            "servo_pitch_min_deg",
            default_value="0.0",
            description="俯仰软件下限(度)，360°舵机常用 0；机械限位请实测后收窄",
        ),
        DeclareLaunchArgument(
            "servo_pitch_max_deg",
            default_value="360.0",
            description="俯仰软件上限(度)",
        ),
        DeclareLaunchArgument(
            "servo_yaw_min_deg",
            default_value="0.0",
            description="偏航软件下限(度)",
        ),
        DeclareLaunchArgument(
            "servo_yaw_max_deg",
            default_value="360.0",
            description="偏航软件上限(度)；须与串口节点 SERVO_ANGLE_MAX 一致",
        ),
        DeclareLaunchArgument(
            "lab_preprocess_blur_ksize",
            default_value="15",
            description="Lab 前高斯模糊核(奇数)；与 offline_trace blur_ksize 对齐；0~2 关闭",
        ),
        DeclareLaunchArgument(
            "lab_mask_morph_ksize",
            default_value="5",
            description="每色 mask 闭+开核(奇数)；与 offline_trace mask_morph 对齐",
        ),
        DeclareLaunchArgument(
            "vis_input_gamma",
            default_value="0.76",
            description="trace 内 ROI 后伽马(1.0=不变)；与 offline_trace gamma 对齐",
        ),
        DeclareLaunchArgument(
            "vis_clahe_l_clip_limit",
            default_value="0.00",
            description="trace 内 Lab L 通道 CLAHE，0 关闭；光照变化大时可试 2.0~3.5",
        ),
        DeclareLaunchArgument(
            "lab_mask_canny_bridge",
            default_value="false",
            choices=["true", "false"],
            description="true：Canny 边缘与颜色掩膜按邻域并集，减轻目标/背景 Lab 接近时的断裂",
        ),
        DeclareLaunchArgument(
            "lab_canny_thresh1",
            default_value="80",
            description="Canny 低阈值（与高阈值配合调边缘密度）",
        ),
        DeclareLaunchArgument(
            "lab_canny_thresh2",
            default_value="160",
            description="Canny 高阈值",
        ),
        DeclareLaunchArgument(
            "lab_canny_dilate_px",
            default_value="5",
            description="Canny 结果膨胀核近似直径(奇数)，略大可连断边",
        ),
        DeclareLaunchArgument(
            "lab_canny_bridge_mask_dilate",
            default_value="2",
            description="颜色掩膜膨胀迭代次数，用于与 Canny 求交后再并回 mask",
        ),
        DeclareLaunchArgument(
            "servo_pitch_travel_half_span_deg",
            default_value="0.0",
            description="俯仰相对 init_pitch 的对称±半宽(度)。>0 时覆盖非对称配置",
        ),
        DeclareLaunchArgument(
            "servo_yaw_travel_half_span_deg",
            default_value="0.0",
            description="偏航相对 init_yaw 的对称±半宽(度)。>0 时覆盖非对称配置",
        ),
        DeclareLaunchArgument(
            "servo_pitch_travel_pos_deg",
            default_value="10.0",
            description="俯仰相对 init_pitch 的正向行程(度)",
        ),
        DeclareLaunchArgument(
            "servo_pitch_travel_neg_deg",
            default_value="10.0",
            description="俯仰相对 init_pitch 的负向行程(度)",
        ),
        DeclareLaunchArgument(
            "servo_yaw_travel_pos_deg",
            default_value="20.0",
            description="偏航相对 init_yaw 的正向行程(度)",
        ),
        DeclareLaunchArgument(
            "servo_yaw_travel_neg_deg",
            default_value="20.0",
            description="偏航相对 init_yaw 的负向行程(度)",
        ),
        DeclareLaunchArgument(
            "servo_angle_smooth_beta",
            default_value="0.32",
            description="舵机角一阶低通：越大越跟手，越小越稳",
        ),
        DeclareLaunchArgument(
            "startup_hold_frames",
            default_value="60",
            description="启动时保持初始角度的帧数",
        ),
        DeclareLaunchArgument(
            "publish_tracking_debug",
            default_value="true",
            choices=["true", "false"],
            description="是否发布 /tracking_debug（17 维：含逻辑 pitch/yaw 浮点、整度 cmd、发到 pitch_id/yaw_id 的实际角）",
        ),
        DeclareLaunchArgument(
            "show_vis_window",
            default_value="true",
            choices=["true", "false"],
            description="是否弹出 OpenCV 调试窗口；无桌面/SSH 请设为 false，否则可能因 GTK 崩溃",
        ),
        DeclareLaunchArgument(
            "runtime_tuning_enable",
            default_value="true",
            choices=["true", "false"],
            description="在 OpenCV 调试窗口启用实时控制参数滑条（Trackbar）",
        ),
        DeclareLaunchArgument(
            "servo_port",
            default_value="",
            description="可选：强制指定舵机串口（如 /dev/ttyACM1）",
        ),
        DeclareLaunchArgument(
            "aim_workflow_enabled",
            default_value="false",
            choices=["true", "false"],
            description="true：须先 aim_command=2 锁中心色再 1 开始瞄准，未开始时舵机冻结；false：沿用自动识别",
        ),
        DeclareLaunchArgument(
            "stable_online_mode",
            default_value="true",
            choices=["true", "false"],
            description="稳定在线闭环模式：保留ROS闭环，但减少预测/搜索带来的链路复杂度",
        ),
        DeclareLaunchArgument(
            "stable_target_hold_frames",
            default_value="3",
            description="稳定模式：短时丢检时保持上一目标帧数",
        ),
        DeclareLaunchArgument(
            "stable_transition_blend_frames",
            default_value="5",
            description="稳定模式：LOCK_CENTER<->TRACK_SQUARE切换后的缓启动帧数",
        ),
        DeclareLaunchArgument(
            "stable_disable_predict",
            default_value="true",
            choices=["true", "false"],
            description="稳定模式：是否禁用预测滑行（COAST）",
        ),
        DeclareLaunchArgument(
            "stable_disable_search",
            default_value="true",
            choices=["true", "false"],
            description="稳定模式：是否禁用SEARCH摇摆搜索",
        ),
        DeclareLaunchArgument(
            "stable_fixed_dt_s",
            default_value="0.04",
            description="稳定模式：控制固定步长秒数（建议0.04对应25Hz）",
        ),
        DeclareLaunchArgument(
            "large_purple_family_match",
            default_value="true",
            choices=["true", "false"],
            description="大能量：中心 id 与扇区 id 在 0/3/6 紫系内可互换匹配（抗 Lab 分裂）",
        ),
        DeclareLaunchArgument(
            "launch_aim_panel",
            default_value="true",
            choices=["true", "false"],
            description="是否同时启动 Qt 瞄准控制面板（需桌面 DISPLAY；发布 /aim_command）",
        ),
        DeclareLaunchArgument(
            "large_center_shape_first",
            default_value="true",
            choices=["true", "false"],
            description="大能量：先按形状找中心圆，再对中心轮廓均值 HSV 读色，再追同色扇区",
        ),
        DeclareLaunchArgument(
            "center_square_area_rel_tol",
            default_value="0.45",
            description="中心圆与同色方块面积相对误差上限（|A_sq-A_c|/A_c）",
        ),
        DeclareLaunchArgument(
            "center_square_diam_side_rel_tol",
            default_value="0.35",
            description="中心圆直径与同色方块边长相对误差上限（|D_c-side|/side）",
        ),
        DeclareLaunchArgument(
            "square_max_dist_center_radius",
            default_value="2.8",
            description="同色方块距中心圆最大距离（按中心圆半径倍数）；过大易吃到背景板",
        ),
        DeclareLaunchArgument(
            "fallback_blob_max_area_ratio",
            default_value="0.10",
            description="回退同色blob筛选最大面积占比（抑制大背景同色块）",
        ),
        DeclareLaunchArgument(
            "fallback_blob_min_dist_center_radius",
            default_value="0.35",
            description="回退同色blob距中心圆的最小距离（按中心圆半径倍数）",
        ),
        DeclareLaunchArgument(
            "fallback_blob_max_dist_center_radius",
            default_value="3.0",
            description="回退同色blob距中心圆的最大距离（按中心圆半径倍数）",
        ),
        DeclareLaunchArgument(
            "center_circularity_min",
            default_value="0.73",
            description="中心候选轮廓圆度下限（与 offline_trace center_circ_min 对齐）",
        ),
        DeclareLaunchArgument(
            "large_center_hsv_max_dist_sq",
            default_value="5200.0",
            description="中心圆读色时 HSV 距离平方上限（与 color_table 比，过小会退回 Lab cid）",
        ),
        DeclareLaunchArgument(
            "large_blob_hsv_relabel",
            default_value="true",
            choices=["true", "false"],
            description="大能量+纯色块：连通域均值 BGR→HSV 按 color_table 距离重标 id，减轻 Lab 分裂",
        ),
        DeclareLaunchArgument(
            "large_blob_hsv_max_dist_sq",
            default_value="5200.0",
            description="HSV 距离平方上限，越小越严（误标少但可能不改 id）",
        ),
        DeclareLaunchArgument(
            "large_square_min_contour_area",
            default_value="220.0",
            description="非纯色块 legacy 分支的大能量连通域面积下限；纯色块分支见 large_lab_min_area_seg",
        ),
        DeclareLaunchArgument(
            "large_lab_min_area_seg",
            default_value="50.0",
            description="大能量+纯色块：offline_trace min_area；连通域保留阈值=max(10, 本值×0.45)，中心几何下限=max(25, 本值×0.35)",
        ),
        DeclareLaunchArgument(
            "center_circle_area_ratio_min",
            default_value="0.80",
            description="中心候选：轮廓面积/最小外接圆面积下限（offline_trace circle_area_ratio_min）",
        ),
        DeclareLaunchArgument(
            "center_rect_area_ratio_max",
            default_value="0.90",
            description="中心候选：轮廓面积/最小外接矩形面积上限（offline_trace rect_area_ratio_max）",
        ),
        DeclareLaunchArgument(
            "center_max_dist_ratio",
            default_value="0.38",
            description="中心候选距光学中心上限（×min(宽,高)）；offline_trace center_max_dist_ratio",
        ),
        DeclareLaunchArgument(
            "large_sector_min_area",
            default_value="220.0",
            description="扇区同色块最小面积（offline_trace square_min_area）",
        ),
        DeclareLaunchArgument(
            "large_sector_max_area_ratio",
            default_value="0.52",
            description="扇区块最大面积占画面比例（offline_trace square_max_area_ratio）",
        ),
        DeclareLaunchArgument(
            "square_approx_poly_eps",
            default_value="0.024",
            description="approxPolyDP 的弧长比例系数（offline_trace square_approx_eps）",
        ),
        DeclareLaunchArgument(
            "large_flower_geometry_sector_fallback",
            default_value="false",
            choices=["true", "false"],
            description="多色花瓣训练靶回退（vision_pc 旧逻辑）；默认关，与 offline_trace 单一路径一致",
        ),
        DeclareLaunchArgument(
            "large_sector_lab_misclass_fallback",
            default_value="false",
            choices=["true", "false"],
            description="Lab 误分时的 camera_demo 几何回退；默认关，与 offline_trace 一致",
        ),
        DeclareLaunchArgument(
            "publish_center_color_id",
            default_value="true",
            choices=["true", "false"],
            description="是否每帧发布 /center_color_id（std_msgs/Int32，与画面 OSD center_id 一致；-1 无效）",
        ),
        DeclareLaunchArgument(
            "suppress_bright_lights",
            default_value="false",
            choices=["true", "false"],
            description="vision_pc 无此步；默认关。开启时掩膜过大则自动跳过 inpaint，避免整幅糊死",
        ),
        DeclareLaunchArgument(
            "bright_hsv_v_min",
            default_value="252",
            description="白灯 HSV：V 下限（仅极亮核，避免整屏进掩膜）",
        ),
        DeclareLaunchArgument(
            "bright_hsv_s_max",
            default_value="58",
            description="白灯 HSV：S 上限",
        ),
        DeclareLaunchArgument(
            "bright_bgr_channel_min",
            default_value="244",
            description="白灯 BGR：三通道均≥此值视为高光",
        ),
        DeclareLaunchArgument(
            "bright_mask_dilate",
            default_value="5",
            description="白灯掩膜膨胀核（奇数）",
        ),
        DeclareLaunchArgument(
            "bright_inpaint_radius",
            default_value="5",
            description="inpaint 半径（像素）",
        ),
        DeclareLaunchArgument(
            "bright_inpaint_max_area_frac",
            default_value="0.012",
            description="高光掩膜占画面比例超过此值则不 inpaint（保清晰度）",
        ),
        DeclareLaunchArgument(
            "center_roi_zoom",
            default_value="1.40",
            description="中心数字变焦：>1 先裁更小中心 ROI 再拉回原尺寸再跟踪（建议 1.0~1.35，与 large_square_roi_frac 叠加）",
        ),
        DeclareLaunchArgument(
            "swap_pitch_yaw_channels",
            default_value="false",
            choices=["true", "false"],
            description="false：俯仰→servo_id_pitch(默认8)、偏航→servo_id_yaw(默认11)。true：两者对调（仅当 init_pitch 误动了11号时再开）",
        ),
        DeclareLaunchArgument(
            "servo_id_pitch",
            default_value="11",
            description="俯仰舵机 ID（须与串口节点协议一致；常见 8 或 1）",
        ),
        DeclareLaunchArgument(
            "servo_id_yaw",
            default_value="8",
            description="偏航舵机 ID（须与串口节点一致；常见 11 或 10）",
        ),
        DeclareLaunchArgument(
            "show_image_view",
            default_value="false",
            choices=["true", "false"],
            description="是否订阅 /processed_image 开窗口（相机直出；调试用可开 true）",
        ),
        DeclareLaunchArgument(
            "show_trace_debug_image_view",
            default_value="true",
            choices=["true", "false"],
            description="是否订阅 trace 节点发布的处理后调试图（Lab+霍夫圆+minAreaRect+跟踪 OSD，非 raw）",
        ),
        DeclareLaunchArgument(
            "publish_trace_debug_image",
            default_value="true",
            choices=["true", "false"],
            description="trace_calculator 是否每帧发布处理后 BGR8 调试图",
        ),
        DeclareLaunchArgument(
            "trace_debug_image_topic",
            default_value="/trace_debug_image",
            description="处理后调试图话题名（image_transport raw）",
        ),
        DeclareLaunchArgument(
            "debug_overlay_hough_circles",
            default_value="false",
            choices=["true", "false"],
            description="在调试图上叠霍夫圆（绿色）；默认关，与 offline 一致不参与可视化",
        ),
        DeclareLaunchArgument(
            "debug_overlay_min_area_rect",
            default_value="false",
            choices=["true", "false"],
            description="在调试图上叠连通域 minAreaRect（青色）；默认关",
        ),
        webcam_node,
        raw_preview_node,
        trace_debug_preview_node,
        trace_node,
        aim_panel_node,
        control_node
    ])

