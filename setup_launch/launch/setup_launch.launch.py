import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    
    robot_ip = LaunchConfiguration("robot_ip")
    use_sim = LaunchConfiguration("use_sim", default="false")
    controllers_file = LaunchConfiguration("controllers_file", default="ur_controllers.yaml")

    if use_sim.perform(context) == "true":
        controllers_file = str(controllers_file.perform(context)).replace(".yaml", "_gazebo.yaml")

    # Launch del driver del UR
    ur_control_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("setup_launch"),
                            "launch",
                            "ur_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ur_type": "ur10e",
                    "robot_ip": robot_ip,
                    "launch_rviz": "false",
                    "use_fake_hardware": "false",
                    "initial_joint_controller": "scaled_joint_trajectory_controller",
                    "activate_joint_controller": "true",
                    "description_package": "setup_description",
                    "description_file": "robot.urdf.xacro",
                    "use_tool_communication": "true",
                    "gripper_com_port": "/tmp/ttyUR",
                    "tool_voltage": "24",
                    "tool_parity": "0",
                    "tool_baud_rate": "115200",
                    "tool_stop_bits": "1",
                    "tool_rx_idle_chars": "1.5",
                    "tool_tx_idle_chars": "3.5",
                    "tool_device_name": "/tmp/ttyUR",
                    "use_sim": use_sim,
                    "controllers_file": controllers_file,
                }.items(),
            )
    
    # Launch de MoveIt
    moveit_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("setup_moveit_config"),
                            "launch",
                            "ur_moveit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ur_type": "ur10e",
                    "launch_rviz": "true",
                    "robot_ip": robot_ip,
                    "description_package": "setup_description",
                    "description_file": "robot.urdf.xacro",
                    "moveit_config_package": "setup_moveit_config",
                    "moveit_config_file": "ur.srdf.xacro",
                    "gripper_com_port": "/tmp/ttyUR",
                    "use_sim_time": use_sim,
                }.items(),
            )
    
    nodes = [
        ur_control_launch, 
        moveit_launch,
    ]
    
    return nodes

def generate_launch_description():

    
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.16.7.75",
            description="IP del robot UR10e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo Ignition simulation.',
            choices=["true", "false"],
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
