#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws"
HOST_SETUP="/opt/ros/humble/setup.bash"
HOST_WS_SETUP="/home/u-zhuang/ros2_ws/install/setup.bash"
HOST_RUNTIME_SETUP="${WORKSPACE_ROOT}/install_humble/setup.bash"
JAZZY_SETUP="/opt/ros/jazzy/setup.bash"
JAZZY_WS_SETUP="${WORKSPACE_ROOT}/install_jazzy/setup.bash"
CONTAINER_NAME="ocs2_hexbot_jazzy"
DEFAULT_REFERENCE_FILE="${WORKSPACE_ROOT}/src/ocs2_hexbot_legged_robot/config/command/reference.info"
DEFAULT_TASK_FILE="${WORKSPACE_ROOT}/src/ocs2_hexbot_legged_robot/config/mpc/task.info"
BRIDGE_CONFIG="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/config/hexbot_mpc_default.yaml"
LOG_DIR="${WORKSPACE_ROOT}/log_humble/idle_reference_probe"
ISAAC_RUNTIME_CONTROL="${WORKSPACE_ROOT}/src/hexbot_description_ros2/isaac/isaac_runtime_control.py"
ISAAC_CLEAN_START="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/ensure_isaac_clean_start.py"
REFERENCE_PRIMER="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/prime_hexbot_reference_joint_command.py"
POSTURE_PROBE="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/probe_hexbot_posture.py"
TOPIC_CAPTURE="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/capture_hexbot_ocs2_topics.py"
PRECOMP_SUMMARY="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/summarize_hexbot_precomputation_log.py"
SOLVER_TRACE_SUMMARY="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/summarize_hexbot_solver_trace.py"

REFERENCE_FILE="${DEFAULT_REFERENCE_FILE}"
TASK_FILE="${DEFAULT_TASK_FILE}"
DURATION_SEC="12"
LABEL=""
LINEAR_X="0.0"
ANGULAR_Z="0.0"
PROBE_DURATION_SEC="0"
PRIME_DURATION_SEC="10"
PRIME_MAX_FINAL_ERROR_RAD="0.45"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --reference-file)
      REFERENCE_FILE="$2"
      shift 2
      ;;
    --task-file)
      TASK_FILE="$2"
      shift 2
      ;;
    --duration)
      DURATION_SEC="$2"
      shift 2
      ;;
    --label)
      LABEL="$2"
      shift 2
      ;;
    --linear-x)
      LINEAR_X="$2"
      shift 2
      ;;
    --angular-z)
      ANGULAR_Z="$2"
      shift 2
      ;;
    --probe-duration)
      PROBE_DURATION_SEC="$2"
      shift 2
      ;;
    --prime-duration)
      PRIME_DURATION_SEC="$2"
      shift 2
      ;;
    --prime-max-final-error-rad)
      PRIME_MAX_FINAL_ERROR_RAD="$2"
      shift 2
      ;;
    *)
      echo "[error] unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

mkdir -p "${LOG_DIR}"
if [[ -z "${LABEL}" ]]; then
  LABEL="$(basename "${REFERENCE_FILE}" .info)"
fi
RUN_TAG="${LABEL}_$(date +%Y%m%d_%H%M%S)"
CONTAINER_LOG="${LOG_DIR}/${RUN_TAG}_container.log"
BRIDGE_LOG="${LOG_DIR}/${RUN_TAG}_bridge.log"
SUMMARY_JSON="${LOG_DIR}/${RUN_TAG}_summary.json"
PROBE_JSON="${LOG_DIR}/${RUN_TAG}_probe.json"
POSTURE_JSON="${LOG_DIR}/${RUN_TAG}_posture.json"
TOPIC_CAPTURE_JSON="${LOG_DIR}/${RUN_TAG}_ocs2_topics.json"
TOPIC_CAPTURE_LOG="${LOG_DIR}/${RUN_TAG}_ocs2_topics.log"
RUNTIME_TOPICS_JSON="${LOG_DIR}/${RUN_TAG}_runtime_topics.json"
PRECOMP_JSON="${LOG_DIR}/${RUN_TAG}_precomp.json"
SOLVER_TRACE_JSON="${LOG_DIR}/${RUN_TAG}_solver_trace.json"
ISAAC_PREFLIGHT_JSON="${LOG_DIR}/${RUN_TAG}_isaac_preflight.json"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

if python3 - "${PROBE_DURATION_SEC}" "${LINEAR_X}" "${ANGULAR_Z}" <<'PY'
import sys
probe_duration = float(sys.argv[1])
linear_x = float(sys.argv[2])
angular_z = float(sys.argv[3])
should_warn = probe_duration > 0.0 and (abs(linear_x) > 1e-9 or abs(angular_z) > 1e-9)
raise SystemExit(0 if should_warn else 1)
PY
then
  echo "[warn] current OCS2 Hexbot command probe is still experimental." >&2
  echo "[warn] it is suitable for closed-loop evidence capture, not for stable walking validation yet." >&2
fi

POSTURE_DURATION_SEC="$(python3 - "${PROBE_DURATION_SEC}" "${DURATION_SEC}" <<'PY'
import sys
probe_duration = float(sys.argv[1])
duration = float(sys.argv[2])
if probe_duration > 0.0:
    print(probe_duration)
else:
    print(min(max(duration, 1.0), 5.0))
PY
)"

cleanup() {
  pkill -f 'hexbot_locomotion_mpc_bridge.py|ros2 launch hexbot_locomotion_mpc' >/dev/null 2>&1 || true
  printf 'root\n' | sudo -S -k docker exec -i "${CONTAINER_NAME}" bash -lc \
    "pkill -f 'legged_robot_sqp_mpc|hexbot_runtime_mrt|hexbot_cmd_vel_target|hexbot_auto_gait_command|robot_state_publisher|ros2 launch ocs2_hexbot_legged_robot_ros hexbot_sqp.launch.py' || true" \
    >/dev/null 2>&1 || true
  python3 "${ISAAC_RUNTIME_CONTROL}" stop >/dev/null 2>&1 || true
}

trap cleanup EXIT

cleanup

set +u
source "${HOST_SETUP}"
source "${HOST_WS_SETUP}"
source "${HOST_RUNTIME_SETUP}"
set -u

python3 "${ISAAC_RUNTIME_CONTROL}" stop > /dev/null 2>&1 || true
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

timeout "$((DURATION_SEC + 6))s" ros2 launch hexbot_locomotion_mpc hexbot_locomotion_mpc.launch.py \
  use_sim_time:=true \
  geometry_config_path:="${BRIDGE_CONFIG}" \
  >"${BRIDGE_LOG}" 2>&1 &
bridge_pid=$!

sleep 2

runtime_topics_ready=0
for attempt in 1 2 3; do
  if python3 "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/check_hexbot_runtime_topics.py" \
    --duration 2.0 \
    > "${RUNTIME_TOPICS_JSON}"; then
    runtime_topics_ready=1
    break
  fi
  sleep 1
done

if [[ "${runtime_topics_ready}" != "1" ]]; then
  cat "${RUNTIME_TOPICS_JSON}" >&2 || true
  exit 1
fi

printf 'root\n' | sudo -S -k docker exec -i "${CONTAINER_NAME}" bash -lc \
  "source '${JAZZY_SETUP}' && \
   source '${JAZZY_WS_SETUP}' && \
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
   timeout ${DURATION_SEC}s ros2 launch ocs2_hexbot_legged_robot_ros hexbot_sqp.launch.py \
     use_sim_time:=true \
     taskFile:='${TASK_FILE}' \
     referenceFile:='${REFERENCE_FILE}'" \
  >"${CONTAINER_LOG}" 2>&1 &
backend_pid=$!

printf 'root\n' | sudo -S -k docker exec -i "${CONTAINER_NAME}" bash -lc \
  "source '${JAZZY_SETUP}' && \
   source '${JAZZY_WS_SETUP}' && \
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
   python3 '${TOPIC_CAPTURE}' \
     --duration '${DURATION_SEC}' \
     --output '${TOPIC_CAPTURE_JSON}'" \
  >"${TOPIC_CAPTURE_LOG}" 2>&1 &
topic_capture_pid=$!

sleep 4

if python3 - "${PROBE_DURATION_SEC}" "${LINEAR_X}" "${ANGULAR_Z}" <<'PY'
import sys
probe_duration = float(sys.argv[1])
linear_x = float(sys.argv[2])
angular_z = float(sys.argv[3])
should_probe = probe_duration > 0.0 and (abs(linear_x) > 1e-9 or abs(angular_z) > 1e-9)
raise SystemExit(0 if should_probe else 1)
PY
then
  python3 "${POSTURE_PROBE}" \
    --use-sim-time \
    --duration "${POSTURE_DURATION_SEC}" \
    --output "${POSTURE_JSON}" >/dev/null &
  posture_pid=$!
  python3 "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/probe_hexbot_closed_loop.py" \
    --use-sim-time \
    --duration "${PROBE_DURATION_SEC}" \
    --linear-x "${LINEAR_X}" \
    --angular-z "${ANGULAR_Z}" \
    --reference-file "${REFERENCE_FILE}" \
    --output "${PROBE_JSON}" >/dev/null
  wait "${posture_pid}" || true
else
  python3 "${POSTURE_PROBE}" \
    --use-sim-time \
    --duration "${POSTURE_DURATION_SEC}" \
    --output "${POSTURE_JSON}" >/dev/null || true
fi

wait "${backend_pid}" || true
wait "${bridge_pid}" || true
wait "${topic_capture_pid}" || true

python3 "${PRECOMP_SUMMARY}" \
  --log "${CONTAINER_LOG}" \
  --output "${PRECOMP_JSON}" >/dev/null || true

python3 "${SOLVER_TRACE_SUMMARY}" \
  --log "${CONTAINER_LOG}" \
  --output "${SOLVER_TRACE_JSON}" >/dev/null || true

python3 - "${CONTAINER_LOG}" "${SUMMARY_JSON}" "${REFERENCE_FILE}" "${DURATION_SEC}" "${PROBE_JSON}" "${POSTURE_JSON}" "${LINEAR_X}" "${ANGULAR_Z}" "${PROBE_DURATION_SEC}" "${TOPIC_CAPTURE_JSON}" "${RUNTIME_TOPICS_JSON}" "${PRECOMP_JSON}" "${SOLVER_TRACE_JSON}" <<'PY'
import json
import pathlib
import sys

log_path = pathlib.Path(sys.argv[1])
summary_path = pathlib.Path(sys.argv[2])
reference_file = sys.argv[3]
duration_sec = float(sys.argv[4])
probe_path = pathlib.Path(sys.argv[5])
posture_path = pathlib.Path(sys.argv[6])
linear_x = float(sys.argv[7])
angular_z = float(sys.argv[8])
probe_duration_sec = float(sys.argv[9])
topic_capture_path = pathlib.Path(sys.argv[10])
runtime_topics_path = pathlib.Path(sys.argv[11])
precomp_path = pathlib.Path(sys.argv[12])
solver_trace_path = pathlib.Path(sys.argv[13])
text = log_path.read_text(encoding="utf-8", errors="replace")

def count(pattern: str) -> int:
    return text.count(pattern)

probe = None
if probe_path.exists():
    probe = json.loads(probe_path.read_text(encoding="utf-8"))
posture = None
if posture_path.exists():
    posture = json.loads(posture_path.read_text(encoding="utf-8"))
topic_capture = None
if topic_capture_path.exists():
    topic_capture = json.loads(topic_capture_path.read_text(encoding="utf-8"))
runtime_topics = None
if runtime_topics_path.exists():
    runtime_topics = json.loads(runtime_topics_path.read_text(encoding="utf-8"))
precomp = None
if precomp_path.exists():
    precomp = json.loads(precomp_path.read_text(encoding="utf-8"))
solver_trace = None
if solver_trace_path.exists():
    solver_trace = json.loads(solver_trace_path.read_text(encoding="utf-8"))

summary = {
    "reference_file": reference_file,
    "duration_sec": duration_sec,
    "log_file": str(log_path),
    "cmd_vel": {
        "linear_x": linear_x,
        "angular_z": angular_z,
        "probe_duration_sec": probe_duration_sec,
    },
    "counts": {
        "initial_policy_received": count("Initial MPC policy received. Runtime MRT loop is active."),
        "reset_request_sent": count("Reset request sent to MPC node. Waiting for acknowledgement."),
        "reject_backend_joint_command": count("Rejecting backend joint command because the normalized output exceeded runtime safety thresholds."),
        "runtime_observations_stale": count("Runtime observations are stale."),
    },
}
if probe is not None:
    summary["probe"] = {
        "ready": probe.get("ready"),
        "failures": probe.get("failures"),
        "motion": probe.get("motion"),
        "command_state_tracking": probe.get("command_state_tracking"),
        "command_vs_actual_joint_state": probe.get("command_vs_actual_joint_state"),
        "backend_vs_runtime_joint_command": probe.get("backend_vs_runtime_joint_command"),
        "solver_diagnostics": probe.get("solver_diagnostics"),
    }
if posture is not None:
    summary["posture"] = posture
if topic_capture is not None:
    summary["ocs2_topics"] = topic_capture
if runtime_topics is not None:
    summary["runtime_topics"] = runtime_topics
if precomp is not None:
    summary["precomputation"] = precomp
if solver_trace is not None:
    summary["solver_trace"] = solver_trace
summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY
