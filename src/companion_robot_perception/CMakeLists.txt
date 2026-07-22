cmake_minimum_required(VERSION 3.8)
project(companion_robot_perception)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/dock_marker_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_marker_detector
)

ament_package()
