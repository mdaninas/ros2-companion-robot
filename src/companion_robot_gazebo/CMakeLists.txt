cmake_minimum_required(VERSION 3.8)
project(companion_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/wasd_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wasd_teleop
)

ament_package()
