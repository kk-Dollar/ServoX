import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt configurations from our renamed package
    moveit_config = MoveItConfigsBuilder("servox", package_name="servox_moveit_config").to_moveit_configs()

    # Get URDF content
    robot_description = moveit_config.robot_description

    # Get controllers configuration path
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("servox_bringup"), "config", "ros2_controllers.yaml"]
    )

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 2. Controller Manager (ros2_control_node)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    # 3. Spawners for controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # 4. MoveIt 2 - move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("servox_moveit_config"), "launch", "move_group.launch.py"])
        ),
    )

    # 5. MoveIt 2 - RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("servox_bringup"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            ros2_control_node,
            # Use TimerActions to delay the spawners a bit to ensure controller_manager is heavily booted
            TimerAction(period=1.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=2.0, actions=[arm_controller_spawner]),
            TimerAction(period=2.0, actions=[gripper_controller_spawner]),
            move_group_launch,
            rviz_node,
        ]
    )
