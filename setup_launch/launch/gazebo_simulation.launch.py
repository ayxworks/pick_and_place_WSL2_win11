import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import load_python_launch_file_as_module
from launch_ros.substitutions import FindPackageShare

def get_xacro_file_content(
    xacro_file,
    arguments={}):
    command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' '
    ]
    if arguments and isinstance(arguments, dict):
        for key, val in arguments.items():
            command.extend([
                '{}:='.format(key),
                val,
                ' '
            ])
    return Command(command)

def launch_setup(context, *args, **kwargs):
    
    # Set Gazebo environment variables
    package_description_list = [
        "setup_description",
        "ur_description",
        "robotiq_description",
        ]
        
    for package_description in package_description_list:
    
        package_directory = get_package_share_directory(package_description)
        install_dir_path = (get_package_prefix(package_description) + "/share")
        robot_meshes_path = os.path.join(package_directory, "meshes")
        gazebo_resource_paths = [install_dir_path, robot_meshes_path]

        if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
            for resource_path in gazebo_resource_paths:
                if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                    os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
        else:
            os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))

    # Launch Gazebo world
    path_to_sdf = os.path.join("setup_description",
                               "gazebo",
                               "worlds", 
                               "empty_world.sdf") 

    declare_world_arg = DeclareLaunchArgument("world",
                                              default_value=["-r ", path_to_sdf],
                                              description="SDF World File")  
    world_config = LaunchConfiguration("world")
    
    # Declare GazeboSim Launch
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])),
                launch_arguments={"gz_args": world_config}.items(),
    )
    
    # Spawn Robot
    spawn_coords = [0.0, 0.0, 0.0] # (x,y,z) [m]
    declare_spawn_x = DeclareLaunchArgument("x", 
                                            default_value=str(spawn_coords[0]),
                                            description="Model Spawn X Axis Value")  
    declare_spawn_y = DeclareLaunchArgument("y", 
                                            default_value=str(spawn_coords[1]),
                                            description="Model Spawn Y Axis Value")  
    declare_spawn_z = DeclareLaunchArgument("z", 
                                            default_value=str(spawn_coords[2]),
                                            description="Model Spawn Z Axis Value")
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", "my_robot",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
    
    # Robot state publisher (/robot_description topic)
    package_directory = get_package_share_directory('setup_description')
    
    # Get Gazebo <-> ROS2 bridge
    bridge_args = ["/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
                    "/tf"   + "@tf2_msgs/msg/TFMessage"  + "[ignition.msgs.Pose_V",]

    gzsim_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=bridge_args,
        output="screen",
    )

    nodes = [
        declare_world_arg,
        gz_sim,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        gz_spawn_entity,
        gzsim_ros_bridge,
    ]

    return nodes

def generate_launch_description() -> LaunchDescription:
    
    declared_arguments = []

    # Create and Return the Launch Description Object #
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])