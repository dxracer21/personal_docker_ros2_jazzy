#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    omy_description_path = get_package_share_directory('open_manipulator_description')
    moveit_config_path = get_package_share_directory('uav_omy_moveit_config')
    omy_xacro = os.path.join(
        omy_description_path,
        'urdf',
        'omy_f3m',
        'omy_f3m.urdf.xacro',
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' ',
                omy_xacro,
                ' use_sim:=true config_type:=omy_f3m_sim use_gripper:=false',
            ]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        'robot_description_semantic': open(
            os.path.join(moveit_config_path, 'config', 'omy_f3m.srdf'),
            encoding='utf-8',
        ).read()
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            load_yaml('uav_omy_moveit_config', 'config/kinematics.yaml'),
            load_yaml('uav_omy_moveit_config', 'config/joint_limits.yaml'),
            load_yaml('uav_omy_moveit_config', 'config/ompl_planning.yaml'),
            load_yaml('uav_omy_moveit_config', 'config/moveit_controllers.yaml'),
        ],
    )

    return LaunchDescription([move_group])
