# Copyright 2026, IKERLAN S. Coop.
# Copyright 2026, Industrial Machinery Export Bilbao S.L.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
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

    setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('setup_launch'),
                'launch',
                'setup_launch.launch.py'
            )
        )
    )

    pick_and_place_gui_node = Node(
        package='pick_and_place_gui',
        executable='pick_place_gui',
        name='pick_place_gui',
        output='screen',
    )

    # Launch vision pipeline
    vision_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vision_pipeline"),
                    "launch",
                    "vision_node.launch.py",
                ]
            )
        )
    )

    return LaunchDescription([
        pick_and_place_node,
        publish_obstacles_launch,
        pick_and_place_gui_node,
        setup_launch,
        vision_pipeline_launch
    ])
