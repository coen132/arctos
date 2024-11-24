import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, FindPackageShare

def generate_launch_description():
    arctos_description_dir = get_package_share_directory('arctos_description')
    arctos_hardware_interface_dir = get_package_share_directory('arctos_hardware_interface')

    # Launch the robot state publisher
    urdf_file = os.path.join(arctos_description_dir, 'urdf', 'arctos.urdf')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )

    # Launch the joint state publisher (to publish joint states)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # Define the robot controller configuration (arctos_controller.yaml)
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("arctos_description"),
            "config",
            "arctos_controller.yaml",  # The file where your controllers are configured
        ]
    )

    # Start the ros2_control node with the controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    # Start the controller spawner for the joint_state_broadcaster
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        robot_controller_spawner,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
