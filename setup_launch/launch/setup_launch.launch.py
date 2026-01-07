from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")
    use_rviz = LaunchConfiguration("use_rviz")

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.16.7.75",
            description="IP del robot UR10e",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Lanzar RViz con MoveIt",
        ),
    ]

    # Socat node to create /tmp/ttyUR
    socat_node = ExecuteProcess(
        cmd=[
            'socat',
            'pty,link=/tmp/ttyUR,raw,ignoreeof,waitslave',
            'tcp:172.16.7.75:54321'
        ],
        output='screen'
    )
    

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
                    "reverse_ip": "172.16.7.25",
                    "tool_voltage": "24",
                    "tool_device_name": "/tmp/ttyUR",
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
                    "use_rviz": use_rviz,
                    "launch_rviz": use_rviz,
                    "robot_ip": robot_ip,
                    "description_package": "setup_description",
                    "description_file": "robot.urdf.xacro",
                    "moveit_config_package": "setup_moveit_config",
                    "moveit_config_file": "ur.srdf.xacro",
                    "gripper_com_port": "/tmp/ttyUR"
                }.items(),
            )

    return LaunchDescription(
        declared_arguments +
        [
            socat_node,
            ur_control_launch,
            moveit_launch,
        ]
    )
