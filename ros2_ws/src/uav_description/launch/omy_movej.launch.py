#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    execute = LaunchConfiguration('execute')
    tracking_start = LaunchConfiguration('tracking_start')
    plan_period = LaunchConfiguration('plan_period')
    trajectory_duration = LaunchConfiguration('trajectory_duration')

    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='true'),
        DeclareLaunchArgument('tracking_start', default_value='true'),
        DeclareLaunchArgument('plan_period', default_value='0.8'),
        DeclareLaunchArgument('trajectory_duration', default_value='4.0'),
        Node(
            package='uav_description',
            executable='omy_movej_movel_tracker.py',
            name='omy_movej',
            output='screen',
            parameters=[{
                'motion_mode': 'movej',
                'start_enabled': tracking_start,
                'execute': execute,
                'target_pose_topic': '/omy/target_pose',
                'joint_state_topic': '/joint_states',
                'controller_action': '/arm_controller/follow_joint_trajectory',
                'plan_period': plan_period,
                'trajectory_duration': trajectory_duration,
            }],
        ),
    ])
