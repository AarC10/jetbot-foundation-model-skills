cmake_minimum_required(VERSION 3.10)
project(praxis_ros)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GenerateSkill.srv"
  "srv/ExecuteSkill.srv"
)

install(PROGRAMS
  praxis_ros/skill_node.py
  praxis_ros/praxis_cli_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
