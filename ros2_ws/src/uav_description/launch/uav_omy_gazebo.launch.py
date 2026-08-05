#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    realtime_pose = LaunchConfiguration('realtime_pose')
    use_sim = LaunchConfiguration('use_sim')

    uav_description_path = get_package_share_directory('uav_description')
    omy_description_path = get_package_share_directory(
        'open_manipulator_description'
    )
    omy_bringup_path = get_package_share_directory(
        'open_manipulator_bringup'
    )
    realsense_description_path = get_package_share_directory(
        'realsense2_description'
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(omy_bringup_path, 'worlds'),
            ':' + str(Path(omy_description_path).parent.resolve()),
            ':' + str(
                Path(realsense_description_path).parent.resolve()
            ),
        ],
    )

    uav_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                uav_description_path,
                'launch',
                'uav_gazebo.launch.py',
            )
        ),
        launch_arguments={
            'realtime_pose': realtime_pose,
        }.items(),
    )

    omy_xacro = os.path.join(
        omy_description_path,
        'urdf',
        'omy_f3m',
        'omy_f3m.urdf.xacro',
    )
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            omy_xacro,
            ' use_sim:=',
            use_sim,
            ' config_type:=omy_f3m_sim',
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='omy_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_omy = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_omy_f3m',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-topic', 'robot_description',
            '-name', 'omy_f3m',
            '-x', '-1.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='omy_joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='omy_arm_controller_spawner',
        output='screen',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='omy_gripper_controller_spawner',
        output='screen',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager',
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='simulation_clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'realtime_pose',
            default_value='false',
            description='true: control the UAV continuously from RViz',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            choices=['true'],
            description=(
                'Simulation-only safety option. This launch never connects '
                'to real OMY hardware.'
            ),
        ),
        gazebo_resource_path,
        uav_gazebo,
        robot_state_publisher,
        clock_bridge,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_omy,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[arm_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller,
                on_exit=[gripper_controller],
            )
        ),
        TimerAction(period=4.0, actions=[spawn_omy]),
    ])
