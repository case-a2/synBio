from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # get package share directories
    robot_share = get_package_share_directory("ur_description")
    tcp_share = get_package_share_directory("ros_tcp_endpoint")
    realsense_share = get_package_share_directory("realsense2_camera")
    synbio_share = get_package_share_directory("synbio_test")

    rviz_config_file = os.path.join(synbio_share, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_robot = DeclareLaunchArgument(
        name='robot',
        default_value='True'
    )

    show_rs = DeclareLaunchArgument(
        name='show_rs',
        default_value='True'
    )


    ur3_node = Node(
        package='ur_description',
        exec_name='view_ur.launch.py',
        name=''
    )

    robot_state_publisher_node = Node(
        condition = UnlessCondition(show_robot),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_'}
        ]        
    )

    joint_state_publisher_node = Node(
        condition = UnlessCondition(show_robot),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'    
    )

    joint_state_publisher_gui_node = Node(
        condition = UnlessCondition(show_robot),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'    
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

