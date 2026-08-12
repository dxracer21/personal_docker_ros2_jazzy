#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    below_offset = LaunchConfiguration('below_offset')
    workspace_marker = LaunchConfiguration('workspace_marker')
    workspace_reach = LaunchConfiguration('workspace_reach')
    workspace_alpha = LaunchConfiguration('workspace_alpha')

    uav_description_path = get_package_share_directory('uav_description')

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

    rviz_config = os.path.join(uav_description_path, 'rviz', 'uav_omy.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='uav_omy_target_pose_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='true: open UAV/OMY interaction RViz config. Use false when MoveIt RViz is already open.',
        ),
        DeclareLaunchArgument(
            'below_offset',
            default_value='0.3',
            description='Target point offset below the UAV along UAV local -Z axis, in meters.',
        ),
        DeclareLaunchArgument(
            'workspace_marker',
            default_value='true',
            description='true: publish OMY workspace marker on /omy/workspace_marker.',
        ),
        DeclareLaunchArgument(
            'workspace_reach',
            default_value='1.1',
            description='OMY workspace sphere radius in meters.',
        ),
        DeclareLaunchArgument(
            'workspace_alpha',
            default_value='0.18',
            description='OMY workspace sphere alpha.',
        ),
        uav_interactive_control,
        uav_visual_marker,
        omy_uav_target_pose,
        omy_workspace_marker,
        rviz_node,
    ])
