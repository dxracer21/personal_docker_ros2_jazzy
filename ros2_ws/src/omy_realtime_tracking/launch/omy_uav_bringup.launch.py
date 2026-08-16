#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = Command([
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

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omy_f3m',
        'hardware_controller_manager.yaml',
    ])

    initial_positions_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omy_f3m',
        'initial_positions.yaml',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            'joint_state_broadcaster',
        ],
        output='both',
        parameters=[{'robot_description': urdf_file}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': False}],
    )

    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        output='both',
        parameters=[initial_positions_file],
    )

    delay_initial_pose_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner,
            on_exit=[joint_trajectory_executor],
        )
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
            'below_offset': 0.3,
        }],
    )

    omy_workspace_marker = Node(
        package='uav_description',
        executable='omy_workspace_marker.py',
        name='omy_workspace_marker',
        output='screen',
        parameters=[{
            'reach_radius': 1.1,
            'alpha': 0.18,
        }],
    )

    return LaunchDescription([
        control_node,
        controller_spawner,
        robot_state_publisher,
        delay_initial_pose_after_controllers,
        uav_interactive_control,
        uav_visual_marker,
        omy_uav_target_pose,
        omy_workspace_marker,
    ])
