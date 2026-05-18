import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('basler_ext_trigger_cpp')
    params = os.path.join(pkg_share, 'config', 'camera_params.yaml')
    cyclone_xml = os.path.join(pkg_share, 'config', 'cyclonedds.xml')

    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    pylon_lib = '/opt/pylon/lib'
    new_ld = f"{pylon_lib}:{existing_ld}" if existing_ld else pylon_lib

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + cyclone_xml),

        # Pylon runtime pre Basler
        SetEnvironmentVariable('PYLON_ROOT', '/opt/pylon'),
        SetEnvironmentVariable('LD_LIBRARY_PATH', new_ld),

        Node(
            package='basler_ext_trigger_cpp',
            executable='ext_trigger_node',
            name='basler_ext_trigger_cpp_node',
            output='screen',
            parameters=[params],
        ),
    ])