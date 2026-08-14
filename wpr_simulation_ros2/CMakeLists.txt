cmake_minimum_required(VERSION 3.8)
project(wpr_simulation_ros2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
