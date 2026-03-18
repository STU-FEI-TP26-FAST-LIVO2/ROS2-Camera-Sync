from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_config = get_package_share_directory('hesai_ros_driver') + '/config/config.yaml'

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config,
        description='Path to Hesai YAML config file',
    )

    driver_node = Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': LaunchConfiguration('config_path')}],
    )

    return LaunchDescription([
        config_path_arg,
        driver_node,
    ])
