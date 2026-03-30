#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUNNER_PY="$SCRIPT_DIR/nav_goal_acceptance_runner.py"
JOINT_SETTLE_PY="$SCRIPT_DIR/wait_joint_state_convergence.py"
DEFAULT_CFG="$PKG_DIR/config/hexbot_v2_stage2_explicit_current_candidate.yaml"
JOINT_CONFIG_PATH="$PKG_DIR/config/joint_name_list.yaml"

PRESET=""
OUTPUT_ROOT=""
GEOMETRY_CFG="$DEFAULT_CFG"
USE_SIM_TIME="true"
GOAL_FRAME="map"
ROBOT_FRAME="base_link"
ODOM_TOPIC="/odom"
ACTION_NAME="navigate_to_pose"
MODE="tripod"
LOCOMOTION_STARTUP_MODE="fixed_stand"
STARTUP_READY_PATTERN="Tripod gait startup ready:"
JOINT_STATE_TOPIC="/joint_states"
JOINT_SETTLE_MAX_DIFF_RAD="0.10"
JOINT_SETTLE_TIMEOUT_SEC="8.0"
JOINT_SETTLE_STABLE_SAMPLES="5"
EXPECTED_NAV_XY_GOAL_TOLERANCE="0.05"
EXPECTED_NAV_YAW_GOAL_TOLERANCE="0.10"
EXPECTED_NAV_PLANNER_TOLERANCE="0.05"

LAUNCH_PID=""
LAUNCH_LOG=""

usage() {
  cat <<EOF
Usage:
  bash $0 --preset PRESET [--geometry-cfg PATH] [--output-root PATH] [--no-sim-time]

Presets:
  nav_fwd_0p5
  nav_fwd_1p0
  nav_turn15_fwd_0p5
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --preset)
      PRESET="$2"
      shift 2
      ;;
    --geometry-cfg)
      GEOMETRY_CFG="$2"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --no-sim-time)
      USE_SIM_TIME="false"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$PRESET" ]]; then
  echo "Missing required --preset." >&2
  usage
  exit 1
fi

if [[ -z "$OUTPUT_ROOT" ]]; then
  OUTPUT_ROOT="$PKG_DIR/nav_results/$(date +%F_%H%M%S)_${PRESET}"
fi

JSON_DIR="$OUTPUT_ROOT/json"
LOG_DIR="$OUTPUT_ROOT/logs"
RESULTS_MD="$OUTPUT_ROOT/result.md"
RUN_JSON="$JSON_DIR/${PRESET}.json"
RUN_LOG="$LOG_DIR/${PRESET}.log"
mkdir -p "$JSON_DIR" "$LOG_DIR"

for required_file in "$RUNNER_PY" "$JOINT_SETTLE_PY" "$GEOMETRY_CFG" "$JOINT_CONFIG_PATH"; do
  if [[ ! -f "$required_file" ]]; then
    echo "Missing required file: $required_file" >&2
    exit 1
  fi
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not in PATH. Source /opt/ros/humble/setup.bash and install_humble/setup.bash first." >&2
  exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is not in PATH." >&2
  exit 1
fi

kill_locomotion_residue() {
  local pattern="hexbot_locomotion_v2_node.py"
  local matched
  matched="$(pgrep -af "$pattern" || true)"
  if [[ -z "$matched" ]]; then
    return 0
  fi

  echo "Cleaning lingering locomotion node processes:" >&2
  printf '%s\n' "$matched" >&2

  pkill -INT -f "$pattern" 2>/dev/null || true
  for _ in {1..10}; do
    if ! pgrep -af "$pattern" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done

  pkill -TERM -f "$pattern" 2>/dev/null || true
  for _ in {1..10}; do
    if ! pgrep -af "$pattern" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done

  pkill -KILL -f "$pattern" 2>/dev/null || true
}

cleanup_launch() {
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Stopping locomotion launch (pid=$LAUNCH_PID)"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        break
      fi
      sleep 0.5
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  kill_locomotion_residue
  LAUNCH_PID=""
  LAUNCH_LOG=""
}

trap cleanup_launch EXIT INT TERM

publisher_count() {
  local topic="$1"
  local count
  count="$(
    ros2 topic info "$topic" 2>/dev/null | awk -F': ' '/Publisher count/ {print $2; exit}'
  )"
  if [[ -z "$count" ]]; then
    echo "0"
  else
    echo "$count"
  fi
}

wait_for_publishers() {
  local topic="$1"
  local min_count="$2"
  local timeout_sec="$3"
  local label="$4"
  local start_sec
  start_sec="$(date +%s)"
  while true; do
    local count
    count="$(publisher_count "$topic")"
    if [[ "$count" =~ ^[0-9]+$ ]] && (( count >= min_count )); then
      echo "$label ready on $topic with $count publisher(s)"
      return 0
    fi
    if (( "$(date +%s)" - start_sec >= timeout_sec )); then
      echo "Timed out waiting for at least $min_count publisher(s) on $topic for $label" >&2
      return 1
    fi
    sleep 1
  done
}

read_numeric_param() {
  local node_name="$1"
  local param_name="$2"
  local output
  output="$(ros2 param get "$node_name" "$param_name" 2>/dev/null || true)"
  if [[ -z "$output" ]]; then
    return 1
  fi
  local value
  value="$(printf '%s\n' "$output" | sed -nE 's/.*: ([+-]?[0-9]+(\.[0-9]+)?([eE][+-]?[0-9]+)?).*/\1/p' | tail -n 1)"
  if [[ -z "$value" ]]; then
    return 1
  fi
  printf '%s\n' "$value"
}

verify_numeric_param() {
  local node_name="$1"
  local param_name="$2"
  local expected_value="$3"
  local actual_value
  actual_value="$(read_numeric_param "$node_name" "$param_name")" || {
    echo "Failed to read Nav2 param $node_name::$param_name" >&2
    return 1
  }
  python3 - "$actual_value" "$expected_value" <<'PY'
import math
import sys
actual = float(sys.argv[1])
expected = float(sys.argv[2])
raise SystemExit(0 if math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-6) else 1)
PY
}

verify_nav2_runtime_params() {
  echo "Verifying live Nav2 goal tolerances..."
  local failed=0

  verify_numeric_param "/controller_server" "general_goal_checker.xy_goal_tolerance" "$EXPECTED_NAV_XY_GOAL_TOLERANCE" || {
    echo "Nav2 runtime mismatch: /controller_server general_goal_checker.xy_goal_tolerance is not $EXPECTED_NAV_XY_GOAL_TOLERANCE" >&2
    failed=1
  }
  verify_numeric_param "/controller_server" "general_goal_checker.yaw_goal_tolerance" "$EXPECTED_NAV_YAW_GOAL_TOLERANCE" || {
    echo "Nav2 runtime mismatch: /controller_server general_goal_checker.yaw_goal_tolerance is not $EXPECTED_NAV_YAW_GOAL_TOLERANCE" >&2
    failed=1
  }
  verify_numeric_param "/controller_server" "FollowPath.xy_goal_tolerance" "$EXPECTED_NAV_XY_GOAL_TOLERANCE" || {
    echo "Nav2 runtime mismatch: /controller_server FollowPath.xy_goal_tolerance is not $EXPECTED_NAV_XY_GOAL_TOLERANCE" >&2
    failed=1
  }
  verify_numeric_param "/planner_server" "GridBased.tolerance" "$EXPECTED_NAV_PLANNER_TOLERANCE" || {
    echo "Nav2 runtime mismatch: /planner_server GridBased.tolerance is not $EXPECTED_NAV_PLANNER_TOLERANCE" >&2
    failed=1
  }

  if (( failed != 0 )); then
    cat >&2 <<EOF
Refusing to continue because live Nav2 parameters do not match the tightened
hexapod acceptance tolerances.

Do not rely on 'pkill -f nav2' alone. The active navigation processes are
typically named:
  controller_server
  smoother_server
  planner_server
  behavior_server
  bt_navigator
  waypoint_follower
  velocity_smoother
  lifecycle_manager

Restart Nav2 cleanly, then rerun this script.
EOF
    return 1
  fi
}

dump_topic_publishers() {
  local topic="$1"
  echo "Publisher inspection for $topic:" >&2
  ros2 topic info "$topic" -v >&2 || true
}

wait_for_log_pattern() {
  local log_path="$1"
  local pattern="$2"
  local timeout_sec="$3"
  local label="$4"
  local start_sec
  start_sec="$(date +%s)"
  while true; do
    if [[ -f "$log_path" ]] && grep -q "$pattern" "$log_path" 2>/dev/null; then
      echo "$label observed in log"
      return 0
    fi
    if (( "$(date +%s)" - start_sec >= timeout_sec )); then
      echo "Timed out waiting for $label in $log_path" >&2
      return 1
    fi
    sleep 1
  done
}

start_locomotion() {
  cleanup_launch
  if ! wait_for_exact_publishers "/joint_command" 0 15 "joint command topic clear" 2; then
    echo "Refusing to start locomotion because /joint_command is still busy after cleanup." >&2
    dump_topic_publishers "/joint_command"
    return 1
  fi

  LAUNCH_LOG="$LOG_DIR/locomotion_${PRESET}.log"
  echo
  echo "Starting locomotion with geometry config: $GEOMETRY_CFG"
  ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py \
    use_sim_time:="$USE_SIM_TIME" \
    mode:="$MODE" \
    locomotion_startup_mode:="$LOCOMOTION_STARTUP_MODE" \
    geometry_config_path:="$GEOMETRY_CFG" \
    >"$LAUNCH_LOG" 2>&1 &
  LAUNCH_PID="$!"

  wait_for_exact_publishers "/joint_command" 1 20 "locomotion publisher" 3 || {
    echo "Locomotion launch failed publisher-count validation. Log tail:" >&2
    dump_topic_publishers "/joint_command"
    tail -n 80 "$LAUNCH_LOG" >&2 || true
    return 1
  }
  wait_for_log_pattern "$LAUNCH_LOG" "$STARTUP_READY_PATTERN" 15 "startup ready" || {
    echo "Locomotion launch reached publisher-ready state but not startup-ready. Log tail:" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    return 1
  }
  echo "Locomotion launch log: $LAUNCH_LOG"
}

wait_for_joint_settle() {
  echo "[$PRESET] Waiting for joint-state settle against $(basename "$GEOMETRY_CFG") (pre-run)..."
  local cmd=(
    python3 "$JOINT_SETTLE_PY"
    --joint-config "$JOINT_CONFIG_PATH"
    --geometry-config "$GEOMETRY_CFG"
    --joint-state-topic "$JOINT_STATE_TOPIC"
    --max-joint-diff-rad "$JOINT_SETTLE_MAX_DIFF_RAD"
    --stable-samples "$JOINT_SETTLE_STABLE_SAMPLES"
    --timeout-sec "$JOINT_SETTLE_TIMEOUT_SEC"
  )
  if [[ "$USE_SIM_TIME" == "true" ]]; then
    cmd+=(--use-sim-time)
  fi
  "${cmd[@]}"
}

wait_for_exact_publishers() {
  local topic="$1"
  local exact_count="$2"
  local timeout_sec="$3"
  local label="$4"
  local stable_required="${5:-3}"
  local stable_count=0
  local start_sec
  start_sec="$(date +%s)"
  while true; do
    local count
    count="$(publisher_count "$topic")"
    if [[ "$count" =~ ^[0-9]+$ ]] && (( count == exact_count )); then
      stable_count=$((stable_count + 1))
      if (( stable_count >= stable_required )); then
        echo "$label ready on $topic with exact publisher count $count"
        return 0
      fi
    else
      stable_count=0
    fi
    if (( "$(date +%s)" - start_sec >= timeout_sec )); then
      echo "Timed out waiting for exact publisher count $exact_count on $topic for $label" >&2
      return 1
    fi
    sleep 1
  done
}

echo "Output root: $OUTPUT_ROOT"
echo "Preset: $PRESET"
echo "Geometry config: $GEOMETRY_CFG"
echo "Startup mode: $LOCOMOTION_STARTUP_MODE"
echo "Goal frame: $GOAL_FRAME"
echo "Robot frame: $ROBOT_FRAME"
echo
echo "Prerequisites:"
echo "  1. Isaac Sim and the ROS bridge are already running."
echo "  2. RTAB-Map and Nav2 are already running."
echo "  3. No other locomotion controller publishes /joint_command."
echo "  4. This script will launch locomotion fresh with:"
echo "     $GEOMETRY_CFG"
echo
read -r -p "Press Enter once /clock + /odom + /map should be live..."

wait_for_publishers "/clock" 1 20 "clock"
wait_for_publishers "$ODOM_TOPIC" 1 20 "odometry"
wait_for_publishers "/map" 1 20 "map"
verify_nav2_runtime_params

echo
echo "[$PRESET] Reset robot position and yaw now."
echo "[$PRESET] Press Enter after reset; the script will send one navigation goal."
read -r

start_locomotion
wait_for_joint_settle

PY_ARGS=(
  "$RUNNER_PY"
  --preset "$PRESET"
  --output "$RUN_JSON"
  --result-md "$RESULTS_MD"
  --geometry-config-path "$GEOMETRY_CFG"
  --goal-frame "$GOAL_FRAME"
  --robot-frame "$ROBOT_FRAME"
  --odom-topic "$ODOM_TOPIC"
  --action-name "$ACTION_NAME"
)

if [[ "$USE_SIM_TIME" == "true" ]]; then
  PY_ARGS+=(--use-sim-time)
fi

echo "[$PRESET] Running navigation acceptance..."
python3 "${PY_ARGS[@]}" | tee "$RUN_LOG"

echo
echo "Done."
echo "Result: $RESULTS_MD"
echo "Log:    $RUN_LOG"
echo "Locomotion log: $LAUNCH_LOG"
echo "JSON:   $RUN_JSON"
