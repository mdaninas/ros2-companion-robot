cmake_minimum_required(VERSION 3.8)
project(companion_robot_behaviors)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/waypoint_patrol.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_patrol
)

install(
  PROGRAMS scripts/auto_docking.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME auto_docking
)

ament_package()
