from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realtime_pose = LaunchConfiguration('realtime_pose')
    show_rviz = LaunchConfiguration('show_rviz')

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
    ])
    uav_urdf = PathJoinSubstitution([
        FindPackageShare('uav_description'), 'urdf', 'uav_ver62.urdf',
    ])
    gui_config = PathJoinSubstitution([
        FindPackageShare('uav_description'), 'gui', 'uav_gui.config',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('uav_description'), 'rviz', 'uav_interactive.rviz',
    ])
    rviz_condition = IfCondition(PythonExpression([
        "'", realtime_pose, "' == 'true' and '", show_rviz, "' == 'true'",
    ]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'gz_args': ['-r empty.sdf --gui-config ', gui_config],
            'on_exit_shutdown': 'true',
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
            '-x', '0.0', '-y', '0.0', '-z', '1.0',
        ],
    )

    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_pose_bridge',
        output='screen',
        arguments=[
            '/model/uav_ver62/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
    )
    pose_output = Node(
        package='uav_description',
        executable='uav_pose_publisher.py',
        name='uav_pose_publisher',
        output='screen',
        parameters=[{
            'input_topic': '/model/uav_ver62/pose',
            'world_frame': 'world',
        }],
    )

    set_pose_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_set_pose_bridge',
        output='screen',
        arguments=[
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        condition=IfCondition(realtime_pose),
    )
    interactive_control = Node(
        package='uav_description',
        executable='uav_interactive_control.py',
        name='uav_interactive_control',
        output='screen',
        condition=IfCondition(realtime_pose),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='uav_interactive_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        condition=rviz_condition,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'realtime_pose',
            default_value='false',
            description=(
                'false: Gazebo gizmo commits on mouse release; '
                'true: RViz marker streams pose while dragging'
            ),
        ),
        DeclareLaunchArgument(
            'show_rviz',
            default_value='true',
            description='true: open the standalone UAV RViz view in realtime mode',
        ),
        gazebo,
        pose_bridge,
        pose_output,
        set_pose_service_bridge,
        rviz,
        TimerAction(period=8.0, actions=[spawn_uav]),
        TimerAction(period=8.5, actions=[interactive_control]),
    ])
