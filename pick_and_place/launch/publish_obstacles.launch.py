from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('pick_and_place')

    obstacles_yaml = os.path.join(
        pkg_share,
        'config',
        'obstacles.yaml'
    )

    obstacle_publisher_node = Node(
        package='pick_and_place',
        executable='obstacle_publisher',
        name='obstacle_publisher',
        output='screen',
        parameters=[
            {
                'yaml_file': obstacles_yaml
            }
        ]
    )

    return LaunchDescription([
        obstacle_publisher_node
    ])
