import launch # 基础launch模块
import launch_ros # ROS2 launch 扩展模块
from launch.actions import ExecuteProcess  # 执行系统命令
from launch.substitutions import LaunchConfiguration  # 启动参数替换

def generate_launch_description():
    energy_node = launch_ros.actions.Node(
        package='trace_energy',
        executable='energy_control',
        output='both',  
    )   

    control_node = launch_ros.actions.Node(
        package='servo_controller',
        executable='servo_uart_node',
        output='both',  
    )   

    return launch.LaunchDescription([
        energy_node,
        control_node
    ])

