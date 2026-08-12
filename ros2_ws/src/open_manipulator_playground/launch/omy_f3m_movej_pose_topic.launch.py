#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    planning_group = LaunchConfiguration("planning_group")
    planning_time = LaunchConfiguration("planning_time")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="omy_f3m", package_name="open_manipulator_moveit_config")
        .robot_description_semantic(Path("config") / "omy_f3m" / "omy_f3m.srdf")
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .joint_limits(Path("config") / "omy_f3m" / "joint_limits.yaml")
        .trajectory_execution(Path("config") / "omy_f3m" / "moveit_controllers.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "target_pose_topic",
            default_value="/omy/target_pose",
            description="PoseStamped topic to command OMY F3M MoveJ pose targets.",
        ),
        DeclareLaunchArgument(
            "planning_group",
            default_value="arm",
            description="MoveIt planning group.",
        ),
        DeclareLaunchArgument(
            "planning_time",
            default_value="10.0",
            description="MoveIt planning time in seconds.",
        ),
        Node(
            package="open_manipulator_playground",
            executable="omy_f3m_movej_pose_topic",
            name="omy_f3m_movej_pose_topic",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "target_pose_topic": target_pose_topic,
                    "planning_group": planning_group,
                    "planning_time": planning_time,
                },
            ],
        ),
    ])
