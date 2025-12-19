from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # Launch del driver del UR
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
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
            "use_tool_communication": "true"
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
            "gripper_comm_port": "/tmp/ttyUR"
        }.items(),
    )

    return LaunchDescription(
        declared_arguments +
        [
            ur_control_launch,
            moveit_launch,
        ]
    )
