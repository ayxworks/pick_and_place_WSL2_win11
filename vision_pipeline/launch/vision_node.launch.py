from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    pkg_share = get_package_share_directory('vision_pipeline')

    params_file = os.path.join(
        pkg_share,
        'config',
        'configuration.yaml'
    )

    # Crear nodo
    vision_pipeline_node = Node(
        package='vision_pipeline',
        executable='vision_node',
        name='pose_estimator_service',
        output='screen',
        parameters=[params_file]
    )
    
    return LaunchDescription([
        vision_pipeline_node
    ])