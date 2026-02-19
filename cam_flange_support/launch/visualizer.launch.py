import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution 
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

THIS_PKG:str = "cam_flange_support"

def launch_setup(context, *args, **kwargs):
    
    # Robot state publisher (/robot_description topic)
    package_directory = get_package_share_directory(THIS_PKG)
    robot_desc_path = package_directory + '/' + 'urdf/debug.urdf.xacro'
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            robot_desc_path,
            ' ',
        ]
    ) 

    robot_description = {'robot_description': robot_description_content}  
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[
            robot_description,
            ]
    )

    rviz_config_dir = os.path.join(get_package_share_directory(THIS_PKG), 
                                    'rviz', 
                                    'config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        emulate_tty=True,
        parameters=[],
        arguments = ['-d', rviz_config_dir],
    ) 
    
    nodes = [
        robot_state_publisher_node,
        rviz_node,
    ]

    return nodes


def generate_launch_description() -> LaunchDescription:

    declared_arguments = []

    nodes = [OpaqueFunction(function=launch_setup)]

    return LaunchDescription(nodes + declared_arguments)