from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    ])

    uav_urdf = PathJoinSubstitution([
        FindPackageShare('uav_description'),
        'urdf',
        'uav_ver62.urdf',
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            # -r을 넣지 않아 일시정지 상태로 시작
            'gz_args': 'empty.sdf',
        }.items(),
    )

    spawn_uav = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-file', uav_urdf,
            '-name', 'uav_ver62',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
    )

    return LaunchDescription([
        gazebo,
        TimerAction(
            period=3.0,
            actions=[spawn_uav],
        ),
    ])
