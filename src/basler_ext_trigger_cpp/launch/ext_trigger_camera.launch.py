from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('basler_ext_trigger_cpp')
    params = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    return LaunchDescription([
        Node(
            package='basler_ext_trigger_cpp',
            executable='ext_trigger_node',
            name='basler_ext_trigger_cpp_node',
            output='screen',
            parameters=[params],
        )
    ])
