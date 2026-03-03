cmake_minimum_required(VERSION 3.8)
project(praxis_ros)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GenerateSkill.srv"
  "srv/ExecuteSkill.srv"
)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  praxis_ros/skill_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
