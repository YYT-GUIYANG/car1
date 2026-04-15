# USB 相机 + 舵机串口 + pidtest；可选 rqt_reconfigure
# pidtest 默认：OpenCV PID 滑块开、Square Tracker 关（与 pidtest_full_tune.launch.py 一致）
# 与 trace_start 一致：为 servo_uart 设置 SERVO_ANGLE_MAX=360
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _opencv_gui_env():
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

    declare_camera_device = DeclareLaunchArgument(
        'camera_device',
        default_value='',
        description='强制摄像头路径；留空则优先 /dev/v4l/by-id/usb-*',
    )
    declare_strict = DeclareLaunchArgument(
        'strict_external_camera',
        default_value='true',
        description='true：仅 USB 外接（by-id），false：可回退内置 /dev/video0',
    )
    declare_servo_port = DeclareLaunchArgument(
        'servo_port',
        default_value='/dev/ttyACM0',
        description='舵机控制板串口（与 ls /dev/ttyACM* 一致）',
    )
    declare_servo_baud = DeclareLaunchArgument(
        'servo_baudrate',
        default_value='9600',
        description='串口波特率，须与控制板一致',
    )
    declare_launch_rqt = DeclareLaunchArgument(
        'launch_rqt_reconfigure',
        default_value='false',
        description='true 时约 2s 后启动 rqt_reconfigure',
    )
    declare_show_preview = DeclareLaunchArgument(
        'show_camera_preview',
        default_value='false',
        description='true 时额外启动 processed_image_preview（默认关，追踪画面见 pidtest Square Tracker）',
    )

    servo_uart_node = Node(
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
        remappings=[
            ('servo_control', '/servo_control'),
        ],
    )

    read_image_node = Node(
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
    )

    pidtest_node = Node(
        package='pidtest',
        executable='pidtest_node',
        name='pidtest_node',
        output='screen',
        emulate_tty=True,
        additional_env=_opencv_gui_env(),
        parameters=[{
            'image_topic': '/processed_image',
            'show_vis_window': True,
            'pid_tune_sliders': True,
            'target_color_id': 0,
            'track_fallback_to_largest_color_blob': True,
            'publish_trace_debug_image': True,
        }],
    )

    camera_preview_node = Node(
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
    )

    rqt_reconf = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        arguments=['/pidtest_node'],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_camera_device,
        declare_strict,
        declare_servo_port,
        declare_servo_baud,
        declare_launch_rqt,
        declare_show_preview,
        LogInfo(msg='启动舵机串口节点 servo_uart_node（订阅 /servo_control）…'),
        LogInfo(msg='若串口失败请检查 servo_port；360° 舵机须保持 SERVO_ANGLE_MAX=360。'),
        LogInfo(msg='read_image（USB 优先）→ /processed_image → pidtest；可选相机预览窗'),
        servo_uart_node,
        read_image_node,
        pidtest_node,
        GroupAction(
            condition=IfCondition(show_camera_preview),
            actions=[camera_preview_node],
        ),
        GroupAction(
            condition=IfCondition(launch_rqt_reconfigure),
            actions=[TimerAction(period=2.0, actions=[rqt_reconf])],
        ),
    ])
