#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    display_gripper = LaunchConfiguration('display_gripper')
    rviz = LaunchConfiguration('rviz')
    use_sim = LaunchConfiguration('use_sim')
    workspace_reach = LaunchConfiguration('workspace_reach')
    workspace_alpha = LaunchConfiguration('workspace_alpha')
    tracking_mode = LaunchConfiguration('tracking_mode')
    preview_orientation_weight = LaunchConfiguration('preview_orientation_weight')
    tracking = LaunchConfiguration('tracking')
    tracking_start = LaunchConfiguration('tracking_start')
    keyboard = LaunchConfiguration('keyboard')
    effective_tracking_start = PythonExpression([
        "'true' if '", tracking, "' == 'true' or '",
        tracking_start, "' == 'true' else 'false'",
    ])
    preview_mode = PythonExpression(["'", tracking_mode, "' == 'preview'"])
    keyboard_preview_mode = PythonExpression(["'", tracking_mode, "' == 'preview' and '", keyboard, "' == 'true'"])
    moveit_mode = PythonExpression(["'", tracking_mode, "' == 'moveit'"])
    moveit_execute = LaunchConfiguration('moveit_execute')
    moveit_preview_joint_states = LaunchConfiguration('moveit_preview_joint_states')
    workspace_policy = LaunchConfiguration('workspace_policy')
    max_joint_delta = LaunchConfiguration('max_joint_delta')
    max_total_joint_delta = LaunchConfiguration('max_total_joint_delta')
    max_abs_joint_position = LaunchConfiguration('max_abs_joint_position')
    use_seeded_ik_goal = LaunchConfiguration('use_seeded_ik_goal')
    seeded_ik_timeout = LaunchConfiguration('seeded_ik_timeout')
    initial_joint_positions = LaunchConfiguration('initial_joint_positions')
    uav_description_path = get_package_share_directory('uav_description')
    omy_description_path = get_package_share_directory('open_manipulator_description')

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
            ' use_gripper:=',
            display_gripper,
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
            'use_sim_time': False,
        }],
    )

    omy_joint_state_preview = Node(
        package='uav_description',
        executable='omy_joint_state_preview.py',
        name='omy_joint_state_preview',
        output='screen',
        parameters=[{
            'initial_joint_positions': initial_joint_positions,
        }],
        condition=IfCondition(moveit_mode),
    )

    omy_moveit_target_tracker = Node(
        package='uav_description',
        executable='omy_moveit_target_tracker.py',
        name='omy_moveit_target_tracker',
        output='screen',
        parameters=[{
            'start_enabled': effective_tracking_start,
            'execute': moveit_execute,
            'preview_joint_states': moveit_preview_joint_states,
            'workspace_radius': workspace_reach,
            'workspace_policy': workspace_policy,
            'max_joint_delta': max_joint_delta,
            'max_total_joint_delta': max_total_joint_delta,
            'max_abs_joint_position': max_abs_joint_position,
            'use_seeded_ik_goal': use_seeded_ik_goal,
            'seeded_ik_timeout': seeded_ik_timeout,
        }],
        condition=IfCondition(moveit_mode),
    )

    omy_preview_target_follow = Node(
        package='uav_description',
        executable='omy_target_follow_joint_state.py',
        name='omy_target_follow_joint_state',
        output='screen',
        parameters=[{
            'start_enabled': effective_tracking_start,
            'initial_joint_positions': initial_joint_positions,
            'orientation_weight': preview_orientation_weight,
        }],
        condition=IfCondition(preview_mode),
    )

    omy_preview_keyboard_control = Node(
        package='uav_description',
        executable='omy_preview_keyboard_control.py',
        name='omy_preview_keyboard_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'start_enabled': effective_tracking_start,
        }],
        condition=IfCondition(keyboard_preview_mode),
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
            'below_offset': 0.3,
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
    )

    rviz_config = os.path.join(uav_description_path, 'rviz', 'uav_omy.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='uav_omy_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            choices=['true'],
            description='Simulation-only safety option. This launch never connects to real OMY hardware.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='true: open RViz with target-following OMY RobotModel and UAV marker displays',
        ),
        DeclareLaunchArgument(
            'display_gripper',
            default_value='true',
            description='true: include the OMY gripper/end-unit visual',
        ),
        DeclareLaunchArgument(
            'workspace_reach',
            default_value='1.1',
            description='OMY workspace sphere radius in meters',
        ),
        DeclareLaunchArgument(
            'workspace_alpha',
            default_value='0.18',
            description='Initial OMY workspace sphere alpha',
        ),
        DeclareLaunchArgument(
            'tracking_mode',
            default_value='preview',
            choices=['preview', 'moveit'],
            description='preview: smooth Jacobian joint-state follower, moveit: exact MoveIt IK/planning',
        ),
        DeclareLaunchArgument(
            'preview_orientation_weight',
            default_value='0.05',
            description='Orientation weight for preview tracking',
        ),
        DeclareLaunchArgument(
            'tracking',
            default_value='false',
            description='true: start the selected target tracking mode immediately',
        ),
        DeclareLaunchArgument(
            'tracking_start',
            default_value='false',
            description='Legacy alias for tracking:=true',
        ),
        DeclareLaunchArgument(
            'keyboard',
            default_value='true',
            description='true: enable terminal keyboard controls in preview mode',
        ),
        DeclareLaunchArgument(
            'moveit_execute',
            default_value='false',
            description='true: execute MoveIt trajectories on configured controllers',
        ),
        DeclareLaunchArgument(
            'moveit_preview_joint_states',
            default_value='true',
            description='true: publish planned trajectory points to /joint_states for RViz preview',
        ),
        DeclareLaunchArgument(
            'initial_joint_positions',
            default_value='0.0,-1.3950,2.3698,-1.0527,1.5707963267948966,0.0',
            description='Comma-separated OMY initial arm joints plus optional gripper position in radians',
        ),
        DeclareLaunchArgument(
            'workspace_policy',
            default_value='hold',
            choices=['hold', 'clamp'],
            description='hold: reject out-of-workspace targets, clamp: project them onto workspace radius',
        ),
        DeclareLaunchArgument(
            'max_joint_delta',
            default_value='2.0',
            description='Reject planned trajectories with any single joint moving more than this many radians',
        ),
        DeclareLaunchArgument(
            'max_total_joint_delta',
            default_value='4.5',
            description='Reject planned trajectories whose total joint motion exceeds this many radians',
        ),
        DeclareLaunchArgument(
            'max_abs_joint_position',
            default_value='3.2',
            description='Reject planned trajectories ending outside this absolute joint position bound',
        ),
        DeclareLaunchArgument(
            'use_seeded_ik_goal',
            default_value='true',
            description='true: solve target pose IK from current joint seed before planning',
        ),
        DeclareLaunchArgument(
            'seeded_ik_timeout',
            default_value='2.0',
            description='Seeded IK timeout in seconds',
        ),
        robot_state_publisher,
        omy_joint_state_preview,
        omy_moveit_target_tracker,
        omy_preview_target_follow,
        omy_preview_keyboard_control,
        uav_interactive_control,
        uav_visual_marker,
        omy_uav_target_pose,
        omy_workspace_marker,
        rviz_node,
    ])
