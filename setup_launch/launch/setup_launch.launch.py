from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.16.7.75",
            description="IP del robot UR10e",
        )
    ]

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
        }.items(),
    )

    # Launch de RealSense D435
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ]
            )
        ),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "depth_module.profile": "640x480x30",
            "rgb_camera.profile": "640x480x30",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "publish_tf": "true",
        }.items(),
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

    return LaunchDescription(
        declared_arguments
        + [
            # ur_control_launch,
            # moveit_launch,
            realsense_launch,
            vision_pipeline_launch
        ]
    )
