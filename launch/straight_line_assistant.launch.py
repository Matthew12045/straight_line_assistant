import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('straight_line_assistant'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='straight_line_assistant',
            executable='straight_line_assistant_node',
            name='straight_line_assistant',
            output='screen',
            parameters=[config],
        )
    ])
