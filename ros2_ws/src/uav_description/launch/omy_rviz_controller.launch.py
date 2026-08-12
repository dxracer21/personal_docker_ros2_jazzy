#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    init_position = LaunchConfiguration('init_position')

    omy_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omy_f3m.launch.py',
            ])
        ),
        launch_arguments={
            'start_rviz': rviz,
            'use_sim': 'false',
            'use_mock_hardware': use_mock_hardware,
            'mock_sensor_commands': mock_sensor_commands,
            'init_position': init_position,
            'ros2_control_type': 'omy_f3m_position',
            'init_position_file': 'initial_positions.yaml',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('init_position', default_value='false'),
        omy_bringup,
    ])
