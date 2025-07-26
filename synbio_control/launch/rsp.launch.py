import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")

    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    left_kinematics_parameters_file = LaunchConfiguration("left_kinematics_parameters_file")
    right_kinematics_parameters_file = LaunchConfiguration("right_kinematics_parameters_file")


    # Load description with necessary parameters
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("synbio_control"),
                    "urdf",
                    "synbio_env_controlled.urdf.xacro",
                ]
            ),
            " ",
            "left_robot_ip:=",
            left_robot_ip,
            " ",
            "right_robot_ip:=",
            right_robot_ip,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "left_kinematics_parameters_file:=",
            left_kinematics_parameters_file,
            " ",
            "right_kinematics_parameters_file:=",
            right_kinematics_parameters_file,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_robot_ip", 
            default_value="0.0.0.0",
            description="IP address by which the left robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_robot_ip", description="IP address by which the right robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("synbio_control"),
                    "config",
                    "left_robot_calibration.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("synbio_control"),
                    "config",
                    "right_robot_calibration.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("synbio_control"),
        "config",
        "controller_manager.yaml",
    )


    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )
