from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # target_frame = "camera_link"
    ur_type = LaunchConfiguration("ur_type")
    description_package = FindPackageShare("workspace_test")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "synbio_env.urdf.xacro"]
    )

    rvizconfig_file = PathJoinSubstitution([description_package, "rviz", "workspace_analysis.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", description_file, " ", "ur_type:=", ur_type]), value_type=str
    )

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("ros2_control_demo_example_1"),
    #         "config",
    #         "rrbot_controllers.yaml",
    #     ]
    # )

    robot_state_publisher_node_1 = Node(
        # namespace="",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # parameters=[robot_controllers],
        output="both",
    )

    # robot_state_publisher_node_2 = Node(
    #     namespace="robot_2",
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": robot_description}],
    # )

    # tf_listener_node = Node(
    #     package="synbio_test",
    #     executable="tf_listener",
    #     parameters=[{"target_frame": target_frame}]
    # )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["forward_position_controller", "--param-file", robot_controllers]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            description="Typo/series of used UR robot.",
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
    ]

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node_1,
            # robot_state_publisher_node_2,
            # tf_listener_node,
            rviz_node,
        ]
    )
