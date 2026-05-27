# 完整链路：USB 相机(read_image) → /processed_image → pidtest → /servo_control → 串口舵机(servo_uart)
# 追踪画面仅 pidtest「Square Tracker」：中心十字 + 目标矩形（不再默认启动 processed_image_preview 重复订阅预览）
# pidtest_node 显式传入 DISPLAY / GDK_BACKEND（Wayland 下建议 x11）
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _opencv_gui_env():
    """OpenCV 窗口：DISPLAY / Wayland 下 GDK 走 XWayland。"""
    env = {}
    env['DISPLAY'] = os.environ.get('DISPLAY', ':0')
    env['GDK_BACKEND'] = os.environ.get('GDK_BACKEND', 'x11')
    env['QT_QPA_PLATFORM'] = os.environ.get('QT_QPA_PLATFORM', 'xcb')
    return env


def generate_launch_description():
    camera_device = LaunchConfiguration('camera_device')
    strict_external_camera = LaunchConfiguration('strict_external_camera')
    servo_port = LaunchConfiguration('servo_port')
    servo_baudrate = LaunchConfiguration('servo_baudrate')
    launch_rqt_reconfigure = LaunchConfiguration('launch_rqt_reconfigure')
    show_camera_preview = LaunchConfiguration('show_camera_preview')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value='',
            description='强制指定摄像头设备（如 /dev/video2）；留空则优先 /dev/v4l/by-id/usb-* 外接 USB',
        ),
        DeclareLaunchArgument(
            'strict_external_camera',
            default_value='true',
            description='true：仅 USB by-id 与显式路径，不用笔记本内置摄像头；false 才回退 /dev/video*',
        ),
        DeclareLaunchArgument(
            'servo_port',
            default_value='/dev/ttyACM0',
            description='舵机控制板串口（ls /dev/ttyACM* 核对）',
        ),
        DeclareLaunchArgument(
            'servo_baudrate',
            default_value='9600',
            description='与舵机板一致的波特率',
        ),
        DeclareLaunchArgument(
            'launch_rqt_reconfigure',
            default_value='false',
            description='true 时约 2s 后启动 rqt_reconfigure（与 OpenCV 滑块二选一即可）',
        ),
        DeclareLaunchArgument(
            'show_camera_preview',
            default_value='false',
            description='true 时额外启动 processed_image_preview（再订阅 /processed_image 仅 raw 预览，一般不需要）',
        ),
        LogInfo(msg='[pidtest_full_tune] servo_uart → read_image(USB优先) → /processed_image → pidtest → /servo_control'),
        LogInfo(msg='[pidtest_full_tune] 图形：Square Tracker（十字+目标框）+ PID 滑块；请在桌面终端启动'),
        Node(
            package='servo_controller',
            executable='servo_uart_node',
            name='servo_uart_node',
            output='screen',
            emulate_tty=True,
            additional_env={
                'SERVO_BAUDRATE': servo_baudrate,
                'SERVO_PORT': servo_port,
                'SERVO_ANGLE_MIN': '0',
                'SERVO_ANGLE_MAX': '360',
            },
            remappings=[('servo_control', '/servo_control')],
        ),
        Node(
            package='trace_energy',
            executable='read_image',
            name='read_image',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'strict_external_camera': strict_external_camera,
                'camera_device': camera_device,
                'publish_period_ms': 10,
                'brightness_adjust_enabled': False,
                'brightness_scale': 1.0,
                'input_gamma': 1.0,
                'clahe_l_clip_limit': 0.0,
                'gray_world_wb_enabled': False,
                'gray_world_wb_strength': 0.72,
            }],
        ),
        Node(
            package='pidtest',
            executable='pidtest_node',
            name='pidtest_node',
            output='screen',
            emulate_tty=True,
            additional_env=_opencv_gui_env(),
            parameters=[{
                'image_topic': '/processed_image',
                # 带识别框/追踪信息的画面（与 /processed_image 原始预览不同）
                'show_vis_window': True,
                'pid_tune_sliders': True,
                'target_color_id': 0,
                'init_pitch_deg': 349.0,
                'init_yaw_deg': 175.0,
                'servo_pitch_travel_half_span_deg': 15.0,
                'servo_yaw_travel_half_span_deg': 15.0,
                'servo_yaw_travel_pos_deg': 16.0,
                'servo_yaw_travel_neg_deg': 7.0,
                # 小能量：恢复上一版跟手，但压制突跳
                'max_delta_deg': 1.45,
                'yaw_gain': 2.75,
                'kp': 0.026,
                'predict_lead_ms': 68.0,
                'servo_angle_smooth_beta': 0.86,
                'pitch_control_dir_sign': -1,
                'pitch_kp_mult': 1.35,
                'pitch_min_step_deg': 0.65,
                'pitch_min_step_err_extra_px': 1.0,
                'servo_cmd_max_step_pitch_deg': 2.5,
                'servo_cmd_max_step_yaw_deg': 2.5,
                'deadband_px': 2,
                'deadband_yaw_px': 2,
                'settle_hold_enable': False,
                # 小能量已知转速预测（默认 60°/s，方向不对时把 dir_sign 改为 -1）
                'small_known_speed_predict_enable': True,
                'small_known_speed_deg_s': 60.0,
                'small_known_speed_dir_sign': 1,
                'small_known_speed_blend': 0.45,
                'track_fallback_to_largest_color_blob': True,
                'publish_trace_debug_image': True,
            }],
        ),
        GroupAction(
            condition=IfCondition(show_camera_preview),
            actions=[
                Node(
                    package='trace_energy',
                    executable='processed_image_preview',
                    name='processed_image_preview',
                    output='screen',
                    emulate_tty=True,
                    additional_env=_opencv_gui_env(),
                    parameters=[{
                        'image_topic': '/processed_image',
                        'window_name': 'USB Camera (/processed_image)',
                    }],
                ),
            ],
        ),
        GroupAction(
            condition=IfCondition(launch_rqt_reconfigure),
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                        Node(
                            package='rqt_reconfigure',
                            executable='rqt_reconfigure',
                            name='rqt_reconfigure',
                            arguments=['/pidtest_node'],
                            output='screen',
                            emulate_tty=True,
                        ),
                    ],
                ),
            ],
        ),
    ])
