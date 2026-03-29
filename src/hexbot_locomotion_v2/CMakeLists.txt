cmake_minimum_required(VERSION 3.8)
project(hexbot_locomotion_v2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
)

install(
  PROGRAMS src/hexbot_locomotion_v2_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
