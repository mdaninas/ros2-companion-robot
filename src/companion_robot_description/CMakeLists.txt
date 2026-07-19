cmake_minimum_required(VERSION 3.8)
project(companion_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

install(
  DIRECTORY rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
