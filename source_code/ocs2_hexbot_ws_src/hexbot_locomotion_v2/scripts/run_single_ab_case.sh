#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUNNER_PY="$SCRIPT_DIR/odom_ab_runner.py"
JOINT_SETTLE_PY="$SCRIPT_DIR/wait_joint_state_convergence.py"
BASELINE_CFG="$PKG_DIR/config/hexbot_v2.yaml"
DEFAULT_STAGE1_CFG="$PKG_DIR/config/hexbot_v2_stage2_explicit_current_candidate.yaml"
JOINT_CONFIG_PATH="$PKG_DIR/config/joint_name_list.yaml"

PRESET=""
OUTPUT_ROOT=""
STAGE1_CFG="$DEFAULT_STAGE1_CFG"
USE_SIM_TIME="true"
CMD_TOPIC="/cmd_vel"
ODOM_TOPIC="/odom"
JOINT_STATE_TOPIC="/joint_states"
MODE="tripod"
DURATION_SEC="10"
LOCOMOTION_STARTUP_MODE="fixed_stand"
STARTUP_READY_PATTERN="Tripod gait startup ready:"
JOINT_SETTLE_MAX_DIFF_RAD="0.10"
JOINT_SETTLE_TIMEOUT_SEC="8.0"
JOINT_SETTLE_STABLE_SAMPLES="5"

usage() {
  cat <<EOF
Usage:
  bash $0 --preset PRESET [--stage1-cfg PATH] [--output-root PATH] [--no-sim-time]

Presets:
  baseline_fwd_0p10
  baseline_fwd_0p15
  baseline_turn_0p35
  stage1_fwd_0p10
  stage1_fwd_0p15
  stage1_turn_0p35
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --preset)
      PRESET="$2"
      shift 2
      ;;
    --stage1-cfg)
      STAGE1_CFG="$2"
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
  OUTPUT_ROOT="$PKG_DIR/ab_results/$(date +%F_%H%M%S)_${PRESET}"
fi

JSON_DIR="$OUTPUT_ROOT/json"
LOG_DIR="$OUTPUT_ROOT/logs"
RESULTS_MD="$OUTPUT_ROOT/result.md"
mkdir -p "$JSON_DIR" "$LOG_DIR"

for required_file in "$RUNNER_PY" "$JOINT_SETTLE_PY" "$BASELINE_CFG" "$STAGE1_CFG" "$JOINT_CONFIG_PATH"; do
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

CONFIG_NAME=""
CASE_LABEL=""
LINEAR_X="0.0"
LINEAR_Y="0.0"
ANGULAR_Z="0.0"
GEOMETRY_CFG="$BASELINE_CFG"
LAUNCH_CFG_PATH=""

case "$PRESET" in
  baseline_fwd_0p10)
    CONFIG_NAME="baseline"
    CASE_LABEL="vx=0.10 for 10s"
    LINEAR_X="0.10"
    ;;
  baseline_fwd_0p15)
    CONFIG_NAME="baseline"
    CASE_LABEL="vx=0.15 for 10s"
    LINEAR_X="0.15"
    ;;
  baseline_turn_0p35)
    CONFIG_NAME="baseline"
    CASE_LABEL="wz=0.35 for 10s"
    ANGULAR_Z="0.35"
    ;;
  stage1_fwd_0p10)
    CONFIG_NAME="stage1"
    CASE_LABEL="vx=0.10 for 10s"
    LINEAR_X="0.10"
    GEOMETRY_CFG="$STAGE1_CFG"
    LAUNCH_CFG_PATH="$STAGE1_CFG"
    ;;
  stage1_fwd_0p15)
    CONFIG_NAME="stage1"
    CASE_LABEL="vx=0.15 for 10s"
    LINEAR_X="0.15"
    GEOMETRY_CFG="$STAGE1_CFG"
    LAUNCH_CFG_PATH="$STAGE1_CFG"
    ;;
  stage1_turn_0p35)
    CONFIG_NAME="stage1"
    CASE_LABEL="wz=0.35 for 10s"
    ANGULAR_Z="0.35"
    GEOMETRY_CFG="$STAGE1_CFG"
    LAUNCH_CFG_PATH="$STAGE1_CFG"
    ;;
  *)
    echo "Unknown preset: $PRESET" >&2
    usage
    exit 1
    ;;
esac

LAUNCH_PID=""
LAUNCH_LOG=""

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
      echo "Timed out waiting for $label on $topic" >&2
      return 1
    fi
    sleep 1
  done
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
    echo "Refusing to start $CONFIG_NAME because /joint_command is still busy after cleanup." >&2
    dump_topic_publishers "/joint_command"
    return 1
  fi

  LAUNCH_LOG="$LOG_DIR/locomotion_${PRESET}.log"
  echo
  echo "Starting locomotion config: $CONFIG_NAME"
  if [[ -n "$LAUNCH_CFG_PATH" ]]; then
    ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py \
      use_sim_time:="$USE_SIM_TIME" \
      mode:="$MODE" \
      locomotion_startup_mode:="$LOCOMOTION_STARTUP_MODE" \
      geometry_config_path:="$LAUNCH_CFG_PATH" \
      >"$LAUNCH_LOG" 2>&1 &
  else
    ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py \
      use_sim_time:="$USE_SIM_TIME" \
      mode:="$MODE" \
      locomotion_startup_mode:="$LOCOMOTION_STARTUP_MODE" \
      >"$LAUNCH_LOG" 2>&1 &
  fi
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
  local phase_label="$1"
  echo "[$PRESET] Waiting for joint-state settle against $(basename "$GEOMETRY_CFG") ($phase_label)..."
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

normalize_yes_no() {
  local raw_value="${1:-}"
  local default_value="${2:-no}"
  local lowered
  lowered="$(printf '%s' "$raw_value" | tr '[:upper:]' '[:lower:]')"
  case "$lowered" in
    y|yes) echo "yes" ;;
    n|no) echo "no" ;;
    "") echo "$default_value" ;;
    *) echo "$default_value" ;;
  esac
}

echo "Output root: $OUTPUT_ROOT"
echo "Preset: $PRESET"
echo "Config: $CONFIG_NAME"
echo "Geometry config for settle gate: $GEOMETRY_CFG"
echo "Startup mode: $LOCOMOTION_STARTUP_MODE"
echo "Joint settle gate: topic=$JOINT_STATE_TOPIC, max_diff_rad=$JOINT_SETTLE_MAX_DIFF_RAD, stable_samples=$JOINT_SETTLE_STABLE_SAMPLES, timeout_sec=$JOINT_SETTLE_TIMEOUT_SEC"
echo
echo "Prerequisites:"
echo "  1. Isaac Sim and the ROS bridge are already running."
echo "  2. Nav2 is not running."
echo "  3. No other locomotion controller publishes /joint_command."
echo
read -r -p "Press Enter once Isaac Sim is up and /clock + /odom should be live..."

if [[ "$USE_SIM_TIME" == "true" ]]; then
  wait_for_publishers "/clock" 1 20 "clock"
fi
wait_for_publishers "$ODOM_TOPIC" 1 20 "odometry"

echo
echo "[$PRESET] Reset robot position and yaw now."
echo "[$PRESET] Target motion: $CASE_LABEL"
read -r -p "Press Enter to start this run..."

start_locomotion
wait_for_joint_settle "pre-run"

OUTPUT_JSON="$JSON_DIR/${PRESET}.json"
echo "[$PRESET] Running..."
runner_cmd=(
  python3 "$RUNNER_PY"
  --duration "$DURATION_SEC"
  --cmd-topic "$CMD_TOPIC"
  --odom-topic "$ODOM_TOPIC"
  --output "$OUTPUT_JSON"
  --linear-x "$LINEAR_X"
  --linear-y "$LINEAR_Y"
  --angular-z "$ANGULAR_Z"
)
if [[ "$USE_SIM_TIME" == "true" ]]; then
  runner_cmd+=(--use-sim-time)
fi
"${runner_cmd[@]}"

measured_still_sec="$(
  python3 - "$OUTPUT_JSON" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as stream:
    data = json.load(stream)
print(f"{float(data.get('max_still_sec', 0.0)):.3f}")
PY
)"

measured_pause_default="no"
if python3 - "$OUTPUT_JSON" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as stream:
    data = json.load(stream)
raise SystemExit(0 if float(data.get("max_still_sec", 0.0)) >= 2.0 else 1)
PY
then
  measured_pause_default="yes"
fi

read -r -p "[$PRESET] Obvious trip/drag? [y/N]: " trip_drag_raw
trip_drag="$(normalize_yes_no "$trip_drag_raw" "no")"

read -r -p "[$PRESET] Pause >2s? [Enter=use measured ${measured_pause_default}, y/N]: " pause_raw
pause_gt_2s="$(normalize_yes_no "$pause_raw" "$measured_pause_default")"

cat >"$RESULTS_MD" <<EOF
# Single A/B Case Result

- preset: $PRESET
- config: $CONFIG_NAME
- cmd: $CASE_LABEL
- trip_drag: $trip_drag
- pause_gt_2s: $pause_gt_2s
- json: $OUTPUT_JSON
- log: $LAUNCH_LOG
- measured_max_still_sec: $measured_still_sec
EOF

echo "[$PRESET] Saved: $OUTPUT_JSON"
echo "[$PRESET] Measured max_still_sec=$measured_still_sec"
echo
echo "Done."
echo "Result: $RESULTS_MD"
echo "Log:    $LAUNCH_LOG"
echo "JSON:   $OUTPUT_JSON"
