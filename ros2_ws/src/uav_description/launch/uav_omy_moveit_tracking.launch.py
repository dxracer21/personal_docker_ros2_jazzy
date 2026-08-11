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
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo = LaunchConfiguration('gazebo')
    realtime_pose = LaunchConfiguration('realtime_pose')
    run_omy_test = LaunchConfiguration('run_omy_test')
    run_gripper = LaunchConfiguration('run_gripper')
    use_gripper = LaunchConfiguration('use_gripper')
    display_gripper = LaunchConfiguration('display_gripper')
    rviz = LaunchConfiguration('rviz')
    use_sim = LaunchConfiguration('use_sim')
    workspace_reach = LaunchConfiguration('workspace_reach')
    workspace_alpha = LaunchConfiguration('workspace_alpha')
    follow_orientation_weight = LaunchConfiguration('follow_orientation_weight')
    tracking = LaunchConfiguration('tracking')
    tracking_start = LaunchConfiguration('tracking_start')
    effective_tracking_start = PythonExpression([
        "'true' if '", tracking, "' == 'true' or '",
        tracking_start, "' == 'true' else 'false'",
    ])
    moveit_execute = LaunchConfiguration('moveit_execute')
    moveit_preview_joint_states = LaunchConfiguration('moveit_preview_joint_states')
    workspace_policy = LaunchConfiguration('workspace_policy')
    max_joint_delta = LaunchConfiguration('max_joint_delta')
    max_total_joint_delta = LaunchConfiguration('max_total_joint_delta')
    max_abs_joint_position = LaunchConfiguration('max_abs_joint_position')
    use_seeded_ik_goal = LaunchConfiguration('use_seeded_ik_goal')
    seeded_ik_timeout = LaunchConfiguration('seeded_ik_timeout')
    initial_joint_positions = LaunchConfiguration('initial_joint_positions')
    effective_use_gripper = PythonExpression([
        "'true' if (",
        "'", gazebo, "' == 'false' and '",
        display_gripper, "' == 'true') or (",
        "'", gazebo, "' == 'true' and '",
        use_gripper, "' == 'true') else 'false'",
    ])

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
        condition=IfCondition(gazebo),
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
            'show_rviz': 'false',
        }.items(),
        condition=IfCondition(gazebo),
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
            ' use_gripper:=',
            effective_use_gripper,
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
            'use_sim_time': gazebo,
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
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        condition=IfCondition(gazebo),
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
        condition=IfCondition(gazebo),
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
        condition=IfCondition(gazebo),
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
        condition=IfCondition(PythonExpression([
            "'", gazebo, "' == 'true' and '",
            run_gripper, "' == 'true' and '",
            use_gripper, "' == 'true'",
        ])),
    )

    rviz_config = os.path.join(
        uav_description_path,
        'rviz',
        'uav_omy.rviz',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='simulation_clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        condition=IfCondition(gazebo),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='uav_omy_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
    )

    rviz_only_interactive_control = Node(
        package='uav_description',
        executable='uav_interactive_control.py',
        name='uav_interactive_control',
        output='screen',
        parameters=[{
            'gazebo_enabled': False,
            'pose_topic': '/uav/pose',
            'world_frame': 'world',
        }],
        condition=UnlessCondition(gazebo),
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
        condition=UnlessCondition(gazebo),
    )

    omy_joint_state_preview = Node(
        package='uav_description',
        executable='omy_joint_state_preview.py',
        name='omy_joint_state_preview',
        output='screen',
        parameters=[{
            'initial_joint_positions': initial_joint_positions,
        }],
        condition=UnlessCondition(gazebo),
    )

    rviz_only_joint_state = Node(
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
        condition=UnlessCondition(gazebo),
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

    omy_uav_follow_test = Node(
        package='uav_description',
        executable='omy_uav_follow_test.py',
        name='omy_uav_follow_test',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", gazebo, "' == 'true' and '",
            run_omy_test, "' == 'true'",
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo',
            default_value='false',
            description='true: run Gazebo, spawn UAV/OMY, and use Gazebo pose feedback',
        ),
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
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='true: open RViz with target-following OMY RobotModel and UAV marker displays',
        ),
        DeclareLaunchArgument(
            'display_gripper',
            default_value='true',
            description='true: include the OMY gripper/end-unit visual in RViz-only mode',
        ),
        DeclareLaunchArgument(
            'use_gripper',
            default_value='false',
            description='true: include the OMY gripper/end-unit in Gazebo mode',
        ),
        DeclareLaunchArgument(
            'run_gripper',
            default_value='false',
            description='true: spawn the OMY gripper controller; requires use_gripper:=true',
        ),
        DeclareLaunchArgument(
            'run_omy_test',
            default_value='false',
            description='true: send small test goals to the OMY arm controller',
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
            'follow_orientation_weight',
            default_value='0.05',
            description='Target-follow preview orientation weight; higher values favor EE orientation',
        ),
        DeclareLaunchArgument(
            'tracking',
            default_value='false',
            description='true: start MoveIt target tracking immediately',
        ),
        DeclareLaunchArgument(
            'tracking_start',
            default_value='false',
            description='Legacy alias for tracking:=true',
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
        gazebo_resource_path,
        uav_gazebo,
        robot_state_publisher,
        omy_joint_state_preview,
        rviz_only_joint_state,
        rviz_only_interactive_control,
        uav_visual_marker,
        omy_uav_target_pose,
        omy_workspace_marker,
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
                on_exit=[gripper_controller, omy_uav_follow_test],
            )
        ),
        TimerAction(period=10.0, actions=[spawn_omy], condition=IfCondition(gazebo)),
        TimerAction(period=2.0, actions=[rviz_node], condition=UnlessCondition(gazebo)),
        TimerAction(period=14.0, actions=[rviz_node], condition=IfCondition(gazebo)),
    ])
