#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    motion_mode = LaunchConfiguration('motion_mode')
    execute = LaunchConfiguration('execute')
    tracking_start = LaunchConfiguration('tracking_start')
    keyboard = LaunchConfiguration('keyboard')
    rviz = LaunchConfiguration('rviz')
    display_gripper = LaunchConfiguration('display_gripper')
    initial_joint_positions = LaunchConfiguration('initial_joint_positions')
    workspace_reach = LaunchConfiguration('workspace_reach')
    workspace_alpha = LaunchConfiguration('workspace_alpha')
    trajectory_duration = LaunchConfiguration('trajectory_duration')
    movel_translation_step = LaunchConfiguration('movel_translation_step')
    movel_rotation_step = LaunchConfiguration('movel_rotation_step')
    max_joint_step = LaunchConfiguration('max_joint_step')

    package_path = get_package_share_directory('uav_description')
    description_path = get_package_share_directory('open_manipulator_description')
    xacro_file = os.path.join(
        description_path, 'urdf', 'omy_f3m', 'omy_f3m.urdf.xacro'
    )
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' use_sim:=true config_type:=omy_f3m_sim use_gripper:=',
            display_gripper,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'motion_mode', default_value='movej', choices=['movej', 'movel'],
            description='movej: OMPL joint-space plan, movel: Cartesian waypoint IK',
        ),
        DeclareLaunchArgument(
            'execute', default_value='false', choices=['false'],
            description='RViz safety lock: this launch only previews trajectories',
        ),
        DeclareLaunchArgument('tracking_start', default_value='false'),
        DeclareLaunchArgument('keyboard', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('display_gripper', default_value='true'),
        DeclareLaunchArgument('workspace_reach', default_value='1.1'),
        DeclareLaunchArgument('workspace_alpha', default_value='0.18'),
        DeclareLaunchArgument('trajectory_duration', default_value='4.0'),
        DeclareLaunchArgument('movel_translation_step', default_value='0.01'),
        DeclareLaunchArgument('movel_rotation_step', default_value='0.05'),
        DeclareLaunchArgument('max_joint_step', default_value='0.20'),
        DeclareLaunchArgument(
            'initial_joint_positions',
            default_value='0.0,-1.3950,2.3698,-1.0527,1.5707963267948966,0.0',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='omy_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        ),
        Node(
            package='uav_description',
            executable='omy_joint_state_preview.py',
            name='omy_joint_state_preview',
            output='screen',
            parameters=[{
                'initial_joint_positions': initial_joint_positions,
                'preview_trajectory_topic': '/omy_motion/preview_trajectory',
            }],
        ),
        Node(
            package='uav_description',
            executable='omy_movej_movel_tracker.py',
            name='omy_movej_movel_tracker',
            output='screen',
            parameters=[{
                'motion_mode': motion_mode,
                'start_enabled': tracking_start,
                'execute': execute,
                'trajectory_duration': trajectory_duration,
                'movel_translation_step': movel_translation_step,
                'movel_rotation_step': movel_rotation_step,
                'max_joint_step': max_joint_step,
            }],
        ),
        Node(
            package='uav_description',
            executable='omy_preview_keyboard_control.py',
            name='omy_motion_keyboard_control',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'start_enabled': tracking_start,
                'set_enabled_service': '/omy_motion/set_enabled',
                'reset_service': '/omy_motion/reset',
            }],
            condition=IfCondition(keyboard),
        ),
        Node(
            package='uav_description',
            executable='uav_interactive_control.py',
            name='uav_interactive_control',
            output='screen',
            parameters=[{
                'gazebo_enabled': False,
                'pose_topic': '/uav/pose',
                'world_frame': 'world',
            }],
        ),
        Node(
            package='uav_description',
            executable='uav_visual_marker.py',
            name='uav_visual_marker',
            output='screen',
            parameters=[{
                'pose_topic': '/uav/pose',
                'marker_topic': '/uav/visual_marker',
                'world_frame': 'world',
            }],
        ),
        Node(
            package='uav_description',
            executable='omy_uav_target_pose.py',
            name='omy_uav_target_pose',
            output='screen',
            parameters=[{'below_offset': 0.3}],
        ),
        Node(
            package='uav_description',
            executable='omy_workspace_marker.py',
            name='omy_workspace_marker',
            output='screen',
            parameters=[{
                'reach_radius': workspace_reach,
                'alpha': workspace_alpha,
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='uav_omy_rviz',
            output='screen',
            arguments=['-d', os.path.join(package_path, 'rviz', 'uav_omy.rviz')],
            condition=IfCondition(rviz),
        ),
    ])
