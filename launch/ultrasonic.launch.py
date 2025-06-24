from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('ultrasonic_bridge')
    rviz_config = os.path.join(pkg_dir, 'config', 'ultra.rviz')

    return LaunchDescription([
        # Your ultrasonic_bridge Python node
        Node(
            package='ultrasonic_bridge',
            executable='ultrasonic_bridge',
            name='ultrasonic_bridge_node',
            output='screen'
        ),

        # RViz2 with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
