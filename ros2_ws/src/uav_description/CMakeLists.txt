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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
