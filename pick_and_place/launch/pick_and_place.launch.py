from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('pick_and_place')

    params_file = os.path.join(
        pkg_share,
        'config',
        'configuration.yaml'
    )

    pick_and_place_node = Node(
        package='pick_and_place',
        executable='pick_and_place_node',
        name='pick_and_place_node',
        output='screen',
        parameters=[params_file]
    )

    publish_obstacles_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share,
                'launch',
                'publish_obstacles.launch.py'
            )
        )
    )

    return LaunchDescription([
        pick_and_place_node,
        publish_obstacles_launch
    ])
