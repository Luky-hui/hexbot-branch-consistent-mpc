#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws"
LOG_DIR="${WORKSPACE_ROOT}/log_humble"
PROBE_JSON="${LOG_DIR}/patent_closed_loop_probe_latest.json"
TARGET_YAML="${LOG_DIR}/patent_closed_loop_target_latest.yaml"
MODE_YAML="${LOG_DIR}/patent_closed_loop_mode_latest.yaml"
SOLVER_YAML="${LOG_DIR}/patent_closed_loop_solver_latest.yaml"
SUMMARY_JSON="${LOG_DIR}/patent_closed_loop_summary_latest.json"
ISAAC_RUNTIME_CONTROL="${WORKSPACE_ROOT}/src/hexbot_description_ros2/isaac/isaac_runtime_control.py"
ISAAC_CLEAN_START="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/ensure_isaac_clean_start.py"
REFERENCE_PRIMER="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/prime_hexbot_reference_joint_command.py"
ISAAC_PREFLIGHT_JSON="${LOG_DIR}/patent_closed_loop_isaac_preflight_latest.json"
REFERENCE_FILE="${WORKSPACE_ROOT}/src/ocs2_hexbot_legged_robot/config/command/reference.info"
TASK_FILE="${WORKSPACE_ROOT}/src/ocs2_hexbot_legged_robot/config/mpc/task.info"
PRIME_DURATION_SEC="10"
PRIME_MAX_FINAL_ERROR_RAD="0.45"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --task-file)
      TASK_FILE="$2"
      shift 2
      ;;
    *)
      break
      ;;
  esac
done

mkdir -p "${LOG_DIR}"
rm -f "${PROBE_JSON}" "${TARGET_YAML}" "${MODE_YAML}" "${SOLVER_YAML}" "${SUMMARY_JSON}"

set +u
source /opt/ros/humble/setup.bash
source /home/u-zhuang/ros2_ws/install/setup.bash
if [[ -f "${WORKSPACE_ROOT}/install_humble/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/install_humble/setup.bash"
fi
set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

python3 "${ISAAC_RUNTIME_CONTROL}" reload-and-play > /dev/null 2>&1 || true
python3 "${REFERENCE_PRIMER}" \
  --use-sim-time \
  --reference-file "${REFERENCE_FILE}" \
  --duration "${PRIME_DURATION_SEC}" \
  --max-final-error-rad "${PRIME_MAX_FINAL_ERROR_RAD}" >/dev/null
python3 "${ISAAC_CLEAN_START}" \
  --duration 2.0 \
  --reference-file "${REFERENCE_FILE}" \
  --max-joint-position-error-rad "${PRIME_MAX_FINAL_ERROR_RAD}" \
  > "${ISAAC_PREFLIGHT_JSON}"

python3 "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/check_hexbot_runtime_topics.py" \
  --duration 2.0

printf 'root\n' | sudo -S docker exec -i ocs2_hexbot_jazzy bash -lc \
  "sleep 2; source /opt/ros/jazzy/setup.bash && \
   source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_jazzy/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
   timeout 12s ros2 topic echo --once /hexbot_mpc_target" > "${TARGET_YAML}" &
target_pid=$!

printf 'root\n' | sudo -S docker exec -i ocs2_hexbot_jazzy bash -lc \
  "sleep 2; source /opt/ros/jazzy/setup.bash && \
   source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_jazzy/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
   timeout 12s ros2 topic echo --once /hexbot_mpc_mode_schedule" > "${MODE_YAML}" &
mode_pid=$!

printf 'root\n' | sudo -S docker exec -i ocs2_hexbot_jazzy bash -lc \
  "sleep 2; source /opt/ros/jazzy/setup.bash && \
   source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_jazzy/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
   timeout 12s ros2 topic echo --once /hexbot_mpc_solver_diagnostics" > "${SOLVER_YAML}" &
solver_pid=$!

python3 "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/probe_hexbot_closed_loop.py" \
  --use-sim-time \
  --duration 5.0 \
  --linear-x 0.03 \
  --angular-z 0.0 \
  "$@" \
  --output "${PROBE_JSON}"

wait "${target_pid}" "${mode_pid}" "${solver_pid}"

python3 "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/merge_patent_closed_loop_evidence.py" \
  --probe-json "${PROBE_JSON}" \
  --target-yaml "${TARGET_YAML}" \
  --mode-yaml "${MODE_YAML}" \
  --solver-yaml "${SOLVER_YAML}" \
  --output "${SUMMARY_JSON}"
