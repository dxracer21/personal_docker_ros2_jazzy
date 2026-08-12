#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    init_position_file = LaunchConfiguration('init_position_file')

    below_offset = LaunchConfiguration('below_offset')
    workspace_marker = LaunchConfiguration('workspace_marker')
    workspace_reach = LaunchConfiguration('workspace_reach')
    workspace_alpha = LaunchConfiguration('workspace_alpha')

    omy_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omy_f3m.launch.py',
            ])
        ),
        launch_arguments={
            'start_rviz': start_rviz,
            'prefix': prefix,
            'use_sim': use_sim,
            'use_mock_hardware': use_mock_hardware,
            'mock_sensor_commands': mock_sensor_commands,
            'init_position': init_position,
            'ros2_control_type': ros2_control_type,
            'init_position_file': init_position_file,
        }.items(),
    )

    uav_interactive_control = Node(
        package='uav_description',
        executable='uav_interactive_control.py',
        name='uav_interactive_control',
        output='screen',
        parameters=[{
            'gazebo_enabled': False,
            'pose_topic': '/uav/pose',
            'world_frame': 'world',
        }],
    )

    uav_visual_marker = Node(
        package='uav_description',
        executable='uav_visual_marker.py',
        name='uav_visual_marker',
        output='screen',
        parameters=[{
            'pose_topic': '/uav/pose',
            'marker_topic': '/uav/visual_marker',
            'world_frame': 'world',
        }],
    )

    omy_uav_target_pose = Node(
        package='uav_description',
        executable='omy_uav_target_pose.py',
        name='omy_uav_target_pose',
        output='screen',
        parameters=[{
            'uav_pose_topic': '/uav/pose',
            'target_pose_topic': '/omy/target_pose',
            'target_marker_topic': '/omy/target_marker',
            'below_offset': below_offset,
        }],
    )

    omy_workspace_marker = Node(
        package='uav_description',
        executable='omy_workspace_marker.py',
        name='omy_workspace_marker',
        output='screen',
        parameters=[{
            'reach_radius': workspace_reach,
            'alpha': workspace_alpha,
        }],
        condition=IfCondition(workspace_marker),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz', default_value='false', description='Whether to execute ROBOTIS bringup RViz.'
        ),
        DeclareLaunchArgument(
            'prefix', default_value='', description='Prefix of the joint and link names.'
        ),
        DeclareLaunchArgument(
            'use_sim', default_value='false', description='Start robot in Gazebo simulation.'
        ),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='false', description='Use mock hardware mirroring command.'
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands', default_value='false', description='Enable mock sensor commands.'
        ),
        DeclareLaunchArgument(
            'init_position', default_value='true', description='Whether to launch the init_position node.'
        ),
        DeclareLaunchArgument(
            'ros2_control_type', default_value='omy_f3m_position', description='Type of ros2_control.'
        ),
        DeclareLaunchArgument(
            'init_position_file', default_value='initial_positions.yaml', description='Path to the initial position file.'
        ),
        DeclareLaunchArgument(
            'below_offset', default_value='0.3', description='Target point offset below the UAV along UAV local -Z axis, in meters.'
        ),
        DeclareLaunchArgument(
            'workspace_marker', default_value='true', description='true: publish OMY workspace marker on /omy/workspace_marker.'
        ),
        DeclareLaunchArgument(
            'workspace_reach', default_value='1.1', description='OMY workspace sphere radius in meters.'
        ),
        DeclareLaunchArgument(
            'workspace_alpha', default_value='0.18', description='OMY workspace sphere alpha.'
        ),
        omy_bringup,
        uav_interactive_control,
        uav_visual_marker,
        omy_uav_target_pose,
        omy_workspace_marker,
    ])
