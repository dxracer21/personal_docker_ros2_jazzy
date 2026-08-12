cmake_minimum_required(VERSION 3.8)
project(uav_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    gui
    rviz
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/uav_pose_publisher.py
    scripts/uav_interactive_control.py
    scripts/uav_visual_marker.py
    scripts/omy_uav_follow_test.py
    scripts/omy_uav_target_pose.py
    scripts/omy_target_follow_joint_state.py
    scripts/omy_preview_keyboard_control.py
    scripts/omy_moveit_target_tracker.py
    scripts/omy_movej_movel_tracker.py
    scripts/omy_joint_state_preview.py
    scripts/omy_zero_joint_state.py
    scripts/omy_workspace_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
