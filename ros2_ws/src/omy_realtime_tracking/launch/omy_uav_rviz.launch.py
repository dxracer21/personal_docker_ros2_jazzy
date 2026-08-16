#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('omy_realtime_tracking'),
        'rviz',
        'omy_uav_realtime.rviz',
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='omy_uav_realtime_rviz',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([rviz])
