cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0054 NEW)
project(jetbot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

