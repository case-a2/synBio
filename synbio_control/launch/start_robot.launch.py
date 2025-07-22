from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup():
    # Initialize Arguments
    # right_ur_type = LaunchConfiguration("right_ur_type")
    left_ur_type = LaunchConfiguration("left_ur_type")

    # right_robot_ip = LaunchConfiguration("right_robot_ip")
    left_robot_ip = LaunchConfiguration("left_robot_ip")

    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    description_launchfile = LaunchConfiguration("description_launchfile")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")

    # Robot specific arguments
    # right_use_mock_hardware = LaunchConfiguration("right_use_mock_hardware")
    # right_mock_sensor_commands = LaunchConfiguration("right_mock_sensor_commands")
    # right_initial_joint_controller = LaunchConfiguration("right_initial_joint_controller")
    # right_activate_joint_controller = LaunchConfiguration("right_activate_joint_controller")
    # right_launch_dashboard_client = LaunchConfiguration("right_launch_dashboard_client")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_fake_sensor_commands = LaunchConfiguration("use_fake_sensor_commands")
    left_initial_joint_controller = LaunchConfiguration("left_initial_joint_controller")
    left_activate_joint_controller = LaunchConfiguration("left_activate_joint_controller")
    left_launch_dashboard_client = LaunchConfiguration("left_launch_dashboard_client")

    # Single controller manager comprising of controllers for both arms
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
            # We use the tf_prefix as substitution in there, so that's why we keep it as an
            # argument for this launchfile
        ],
        output="screen",
        # Added remapping for control node to find robot_description
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # right_dashboard_client_node = Node(
    #     package="ur_robot_driver",
    #     condition=IfCondition(right_launch_dashboard_client) and UnlessCondition(right_use_mock_hardware),
    #     executable="dashboard_client",
    #     name="dashboard_client",
    #     namespace="right",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[{"robot_ip": right_robot_ip}],
    # )

    left_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(left_launch_dashboard_client) and UnlessCondition(use_fake_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        # namespace="left",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": left_robot_ip}],
    )

    # right_urscript_interface = Node(
    #     package="ur_robot_driver",
    #     executable="urscript_interface",
    #     namespace="right",
    #     parameters=[{"robot_ip": right_robot_ip}],
    #     output="screen",
    #     condition=UnlessCondition(right_use_mock_hardware),
    # )

    left_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        # namespace="left",
        parameters=[{"robot_ip": left_robot_ip}],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    # right_controller_stopper_node = Node(
    #     package="ur_robot_driver",
    #     executable="controller_stopper_node",
    #     namespace="right",
    #     name="controller_stopper",
    #     output="screen",
    #     emulate_tty=True,
    #     condition=UnlessCondition(right_use_mock_hardware),
    #     parameters=[
    #         {"headless_mode": headless_mode},
    #         {"joint_controller_active": right_activate_joint_controller},
    #         {
    #             "consistent_controllers": [
    #                 "joint_state_broadcaster",
    #                 "right_io_and_status_controller",
    #                 "left_io_and_status_controller",
    #                 "right_force_torque_sensor_broadcaster",
    #                 "left_force_torque_sensor_broadcaster",
    #                 "right_speed_scaling_state_broadcaster",
    #                 "left_speed_scaling_state_broadcaster",
    #             ]
    #         },
    #     ],
    # )

    left_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        # namespace="left",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": left_activate_joint_controller},
            {
                "consistent_controllers": [
                    "left_joint_state_broadcaster",
                    # "right_io_and_status_controller",
                    "left_io_and_status_controller",
                    # "right_force_torque_sensor_broadcaster",
                    "left_force_torque_sensor_broadcaster",
                    # "right_speed_scaling_state_broadcaster",
                    "left_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "left_joint_state_broadcaster",
        # "right_io_and_status_controller",
        "left_io_and_status_controller",
        # "right_speed_scaling_state_broadcaster",
        "left_speed_scaling_state_broadcaster",
        # "right_force_torque_sensor_broadcaster",
        "left_force_torque_sensor_broadcaster",
    ]
    controllers_inactive = [
        # "right_forward_position_controller",
        "left_forward_position_controller",
    ]

    controller_spawners = [controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    # There may be other controllers of the joints, but this is the initially-started one
    # right_initial_joint_controller_spawner_started = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         right_initial_joint_controller,
    #         "-c",
    #         "/controller_manager",
    #         "--controller-manager-timeout",
    #         controller_spawner_timeout,
    #     ],
    #     condition=IfCondition(right_activate_joint_controller),
    # )
    left_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            left_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(left_activate_joint_controller),
    )
    # right_initial_joint_controller_spawner_stopped = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         right_initial_joint_controller,
    #         "-c",
    #         "/controller_manager",
    #         "--controller-manager-timeout",
    #         controller_spawner_timeout,
    #         "--inactive",
    #     ],
    #     condition=UnlessCondition(right_activate_joint_controller),
    # )
    left_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            left_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(left_activate_joint_controller),
    )

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            # "right_robot_ip": right_robot_ip,
            "left_robot_ip": left_robot_ip,
            # "right_ur_type": right_ur_type,
            "left_ur_type": left_ur_type,
        }.items(),
    )

    nodes_to_start = [
        control_node,
        # right_dashboard_client_node,
        left_dashboard_client_node,
        # right_controller_stopper_node,
        left_controller_stopper_node,
        # right_urscript_interface,
        left_urscript_interface,
        rsp,
        rviz_node,
        # right_initial_joint_controller_spawner_stopped,
        left_initial_joint_controller_spawner_stopped,
        # right_initial_joint_controller_spawner_started,
        left_initial_joint_controller_spawner_started,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "right_ur_type",
    #         description="Type/series of used UR robot.",
    #         choices=[
    #             "ur3",
    #             "ur3e",
    #             "ur5",
    #             "ur5e",
    #             "ur10",
    #             "ur10e",
    #             "ur16e",
    #             "ur20",
    #             "ur30",
    #         ],
    #         default_value="ur3e",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur_type",
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
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "right_robot_ip",
    #         default_value="192.168.0.101",
    #         description="IP address by which right can be reached.",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_robot_ip",
            description="IP address by which left can be reached.",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("synbio_control"),
                    "config",
                    "controller_manager.yaml",
                ]
            ),
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("synbio_control"),
                    "launch",
                    "rsp.launch.py",
                ]
            ),
            description="Launchfile (absolute path) providing the description. "
            "The launchfile has to start a robot_state_publisher node that "
            "publishes the description topic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start left with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for right's sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control for both arms.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "right_initial_joint_controller",
    #         default_value="right_scaled_joint_trajectory_controller",
    #         description="Initially loaded robot controller for the right robot arm.",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_initial_joint_controller",
            default_value="left_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the left robot arm.",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "right_activate_joint_controller",
    #         default_value="true",
    #         description="Activate loaded joint controller for the right robot arm.",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the left robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("workspace_test"), "rviz", "workspace_analysis.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "right_launch_dashboard_client",
    #         default_value="true",
    #         description="Launch Dashboard Client?",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("synbio_control"),
                        "config",
                    ]
                ),
                "/",
                "update_rate.yaml",
            ],
        )
    )
    return LaunchDescription(declared_arguments + launch_setup())