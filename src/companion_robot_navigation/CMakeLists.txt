cmake_minimum_required(VERSION 3.8)
project(companion_robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/directional_safety_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME directional_safety_filter
)

install(
  PROGRAMS scripts/velocity_command_mux.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME velocity_command_mux
)

ament_package()
