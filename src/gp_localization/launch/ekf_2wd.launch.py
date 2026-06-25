import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('gp_localization'),
        'config',
        'ekf_2wd.yaml',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    return LaunchDescription([
        ekf_node,
    ])
