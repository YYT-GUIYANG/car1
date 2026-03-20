import launch # 基础launch模块
import launch_ros # ROS2 launch 扩展模块
from launch.actions import ExecuteProcess  # 执行系统命令
from launch.substitutions import LaunchConfiguration  # 启动参数替换

def generate_launch_description():
    webcam_node = launch_ros.actions.Node(
        package='trace_energy',
        executable='read_image',
        output='both',  
    )   

    trace_node = launch_ros.actions.Node(
        package='trace_energy',
        executable='trace_calculator',
        output='both',  
    )   

    control_node = launch_ros.actions.Node(
        package='servo_controller',
        executable='servo_uart_node',
        output='both',  
    )   

    return launch.LaunchDescription([
        webcam_node, 
        trace_node,
        control_node
    ])

