#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf',
            'omy_f3m',
            'omy_f3m.urdf.xacro',
        ]),
        ' use_sim:=false',
        ' use_mock_hardware:=true',
        ' mock_sensor_commands:=false',
        ' config_type:=omy_f3m',
        ' ros2_control_type:=omy_f3m_position',
        ' use_gripper:=true',
    ])

    tracker_config = PathJoinSubstitution([
        FindPackageShare('omy_realtime_tracking'),
        'config',
        'jacobian_tracking.yaml',
    ])

    tracker = Node(
        package='omy_realtime_tracking',
        executable='jacobian_tracker',
        name='jacobian_tracker',
        output='screen',
        parameters=[
            tracker_config,
            {'robot_description': robot_description},
        ],
    )

    return LaunchDescription([tracker])
