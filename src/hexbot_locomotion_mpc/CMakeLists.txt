cmake_minimum_required(VERSION 3.8)
project(hexbot_locomotion_mpc)

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
  PROGRAMS
    src/hexbot_locomotion_mpc_bridge.py
    src/check_hexbot_mpc_readiness.py
    src/check_hexbot_runtime_topics.py
    src/merge_patent_closed_loop_evidence.py
    src/probe_hexbot_closed_loop.py
    src/export_hexbot_ocs2_seed.py
    scripts/bootstrap_mpc_sources.sh
    scripts/run_patent_closed_loop_case.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
