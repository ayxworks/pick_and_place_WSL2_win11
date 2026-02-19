from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_share = get_package_share_directory('vision_pipeline')

    params_file = os.path.join(
        pkg_share,
        'config',
        'configuration.yaml'
    )

    # Declare launch argument
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulated RGB and depth images instead of camera topics'
    )

    use_sim = LaunchConfiguration('use_sim')

    # Create the vision pipeline node
    vision_pipeline_node = Node(
        package='vision_pipeline',
        executable='vision_node',
        name='pose_estimator_service',
        output='screen',
        parameters=[
            params_file,
            {'use_sim': use_sim}
        ]
    )

    sim_camera_frames_node = Node(
        package='vision_pipeline',
        executable='publish_sim_camera_frames',
        name='sim_camera_frames_publisher',
        output='screen',
        condition=IfCondition(use_sim)
    )
    
    return LaunchDescription([
        use_sim_arg,
        vision_pipeline_node,
        sim_camera_frames_node
    ])