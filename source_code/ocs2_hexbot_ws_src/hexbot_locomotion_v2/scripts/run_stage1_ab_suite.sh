#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUNNER_PY="$SCRIPT_DIR/odom_ab_runner.py"
JOINT_SETTLE_PY="$SCRIPT_DIR/wait_joint_state_convergence.py"
DEFAULT_STAGE1_CFG="$PKG_DIR/config/hexbot_v2_stage2_explicit_current_candidate.yaml"
BASELINE_CFG="$PKG_DIR/config/hexbot_v2.yaml"
JOINT_CONFIG_PATH="$PKG_DIR/config/joint_name_list.yaml"

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
  bash $0 [--output-root PATH] [--stage1-cfg PATH] [--no-sim-time] [--startup-mode MODE]

Environment assumptions:
  - Isaac Sim and the ROS bridge are already running
  - Nav2 is not running
  - this shell has already sourced ROS 2 and the workspace install_humble/setup.bash

Outputs:
  - JSON files under the chosen output directory
  - markdown summary table at <output-root>/results.md
  - locomotion launch logs under <output-root>/logs/
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --stage1-cfg)
      STAGE1_CFG="$2"
      shift 2
      ;;
    --no-sim-time)
      USE_SIM_TIME="false"
      shift
      ;;
    --startup-mode)
      LOCOMOTION_STARTUP_MODE="$2"
      shift 2
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

if [[ -z "$OUTPUT_ROOT" ]]; then
  OUTPUT_ROOT="$PKG_DIR/ab_results/$(date +%F_%H%M%S)"
fi

JSON_DIR="$OUTPUT_ROOT/json"
LOG_DIR="$OUTPUT_ROOT/logs"
OBSERVATIONS_TSV="$OUTPUT_ROOT/observations.tsv"
RESULTS_MD="$OUTPUT_ROOT/results.md"
mkdir -p "$JSON_DIR" "$LOG_DIR"

if [[ ! -f "$RUNNER_PY" ]]; then
  echo "Missing helper: $RUNNER_PY" >&2
  exit 1
fi
if [[ ! -f "$JOINT_SETTLE_PY" ]]; then
  echo "Missing helper: $JOINT_SETTLE_PY" >&2
  exit 1
fi
if [[ ! -f "$STAGE1_CFG" ]]; then
  echo "Missing stage1 config: $STAGE1_CFG" >&2
  exit 1
fi
if [[ ! -f "$BASELINE_CFG" ]]; then
  echo "Missing baseline config: $BASELINE_CFG" >&2
  exit 1
fi
if [[ ! -f "$JOINT_CONFIG_PATH" ]]; then
  echo "Missing joint config: $JOINT_CONFIG_PATH" >&2
  exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not in PATH. Source /opt/ros/humble/setup.bash and install_humble/setup.bash first." >&2
  exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is not in PATH." >&2
  exit 1
fi

declare -a CASE_KEYS=(
  "baseline_fwd_0p10"
  "baseline_fwd_0p15"
  "baseline_turn_0p35"
  "stage1_fwd_0p10"
  "stage1_fwd_0p15"
  "stage1_turn_0p35"
)
declare -a CASE_CONFIGS=(
  "baseline"
  "baseline"
  "baseline"
  "stage1"
  "stage1"
  "stage1"
)
declare -a CASE_LABELS=(
  "vx=0.10 for 10s"
  "vx=0.15 for 10s"
  "wz=0.35 for 10s"
  "vx=0.10 for 10s"
  "vx=0.15 for 10s"
  "wz=0.35 for 10s"
)
declare -a CASE_LINEAR_X=(
  "0.10"
  "0.15"
  "0.00"
  "0.10"
  "0.15"
  "0.00"
)
declare -a CASE_LINEAR_Y=(
  "0.00"
  "0.00"
  "0.00"
  "0.00"
  "0.00"
  "0.00"
)
declare -a CASE_ANGULAR_Z=(
  "0.00"
  "0.00"
  "0.35"
  "0.00"
  "0.00"
  "0.35"
)

LAUNCH_PID=""
LAUNCH_LOG=""

kill_locomotion_residue() {
  local pattern="hexbot_locomotion_v2_node.py"
  local matched=""
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

assert_single_joint_command_publisher() {
  local count
  count="$(publisher_count "/joint_command")"
  if [[ ! "$count" =~ ^[0-9]+$ ]] || (( count != 1 )); then
    echo "Expected exactly one /joint_command publisher, found: $count" >&2
    echo "Check that no other locomotion controller is still running." >&2
    return 1
  fi
}

start_locomotion() {
  local config_name="$1"
  local config_path="$2"
  local launch_key="${3:-$config_name}"
  cleanup_launch
  if ! wait_for_exact_publishers "/joint_command" 0 15 "joint command topic clear" 2; then
    echo "Refusing to start $config_name because /joint_command is still busy after cleanup." >&2
    dump_topic_publishers "/joint_command"
    return 1
  fi

  LAUNCH_LOG="$LOG_DIR/locomotion_${launch_key}.log"
  echo
  echo "Starting locomotion config: $config_name"
  if [[ -n "$config_path" ]]; then
    ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py \
      use_sim_time:="$USE_SIM_TIME" \
      mode:="$MODE" \
      locomotion_startup_mode:="$LOCOMOTION_STARTUP_MODE" \
      geometry_config_path:="$config_path" \
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

pause_for_reset() {
  local case_key="$1"
  local label="$2"
  echo
  echo "[$case_key] Reset robot position and yaw now."
  echo "[$case_key] Target motion: $label"
  read -r -p "Press Enter to start this run..."
}

wait_for_joint_settle() {
  local case_key="$1"
  local geometry_cfg="$2"
  local phase_label="${3:-settle}"
  echo "[$case_key] Waiting for joint-state settle against $(basename "$geometry_cfg") ($phase_label)..."
  local cmd=(
    python3 "$JOINT_SETTLE_PY"
    --joint-config "$JOINT_CONFIG_PATH"
    --geometry-config "$geometry_cfg"
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

run_case() {
  local index="$1"
  local case_key="${CASE_KEYS[$index]}"
  local config_name="${CASE_CONFIGS[$index]}"
  local label="${CASE_LABELS[$index]}"
  local linear_x="${CASE_LINEAR_X[$index]}"
  local linear_y="${CASE_LINEAR_Y[$index]}"
  local angular_z="${CASE_ANGULAR_Z[$index]}"
  local output_json="$JSON_DIR/${case_key}.json"
  local geometry_cfg="$BASELINE_CFG"
  local launch_cfg_path=""
  if [[ "$config_name" == "stage1" ]]; then
    geometry_cfg="$STAGE1_CFG"
    launch_cfg_path="$STAGE1_CFG"
  fi

  pause_for_reset "$case_key" "$label"
  start_locomotion "$config_name" "$launch_cfg_path" "$case_key"
  wait_for_joint_settle "$case_key" "$geometry_cfg" "pre-run"

  echo "[$case_key] Running..."
  local cmd=(
    python3 "$RUNNER_PY"
    --duration "$DURATION_SEC"
    --cmd-topic "$CMD_TOPIC"
    --odom-topic "$ODOM_TOPIC"
    --output "$output_json"
    --linear-x "$linear_x"
    --linear-y "$linear_y"
    --angular-z "$angular_z"
  )
  if [[ "$USE_SIM_TIME" == "true" ]]; then
    cmd+=(--use-sim-time)
  fi
  "${cmd[@]}"

  local measured_still_sec
  measured_still_sec="$(
    python3 - "$output_json" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as stream:
    data = json.load(stream)
print(f"{float(data.get('max_still_sec', 0.0)):.3f}")
PY
  )"

  local measured_pause_default="no"
  if python3 - "$output_json" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as stream:
    data = json.load(stream)
raise SystemExit(0 if float(data.get("max_still_sec", 0.0)) >= 2.0 else 1)
PY
  then
    measured_pause_default="yes"
  fi

  local trip_drag_raw
  local trip_drag
  read -r -p "[$case_key] Obvious trip/drag? [y/N]: " trip_drag_raw
  trip_drag="$(normalize_yes_no "$trip_drag_raw" "no")"

  local pause_raw
  local pause_gt_2s
  read -r -p "[$case_key] Pause >2s? [Enter=use measured ${measured_pause_default}, y/N]: " pause_raw
  pause_gt_2s="$(normalize_yes_no "$pause_raw" "$measured_pause_default")"

  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$case_key" \
    "$config_name" \
    "$label" \
    "$trip_drag" \
    "$pause_gt_2s" \
    "$output_json" \
    >>"$OBSERVATIONS_TSV"

  echo "[$case_key] Saved: $output_json"
  echo "[$case_key] Measured max_still_sec=$measured_still_sec"
  wait_for_joint_settle "$case_key" "$geometry_cfg" "post-run stand return"
  cleanup_launch
}

generate_summary() {
  python3 - "$OBSERVATIONS_TSV" "$RESULTS_MD" <<'PY'
import json
import sys
from pathlib import Path

observations_path = Path(sys.argv[1])
results_md_path = Path(sys.argv[2])
lines = observations_path.read_text(encoding="utf-8").splitlines()

rows = []
for line in lines[1:]:
    if not line.strip():
        continue
    case_key, config_name, label, trip_drag, pause_gt_2s, json_path = line.split("\t")
    with open(json_path, encoding="utf-8") as stream:
        metrics = json.load(stream)
    rows.append(
        {
            "case_key": case_key,
            "config": config_name,
            "cmd": label,
            "trip_drag": trip_drag,
            "pause_gt_2s": pause_gt_2s,
            "delta_x_m": metrics.get("delta_x_m"),
            "delta_y_m": metrics.get("delta_y_m"),
            "delta_yaw_rad": metrics.get("delta_yaw_rad"),
            "max_still_sec": metrics.get("max_still_sec"),
            "json_path": json_path,
        }
    )

def fmt(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.4f}"

md_lines = [
    "# Stage1 A/B Results",
    "",
    "| config | cmd | trip_drag | pause_gt_2s | delta_x_m | delta_y_m | delta_yaw_rad |",
    "| --- | --- | --- | --- | --- | --- | --- |",
]
for row in rows:
    md_lines.append(
        f"| {row['config']} | {row['cmd']} | {row['trip_drag']} | {row['pause_gt_2s']} | "
        f"{fmt(row['delta_x_m'])} | {fmt(row['delta_y_m'])} | {fmt(row['delta_yaw_rad'])} |"
    )

detail_lines = [
    "",
    "Detailed max still time per run:",
    "",
]
for row in rows:
    detail_lines.append(
        f"- {row['case_key']}: max_still_sec={fmt(row['max_still_sec'])}, json={row['json_path']}"
    )

payload = "\n".join(md_lines + detail_lines) + "\n"
results_md_path.write_text(payload, encoding="utf-8")
print(payload)
PY
}

printf 'case_key\tconfig\tcmd\ttrip_drag\tpause_gt_2s\tjson_path\n' >"$OBSERVATIONS_TSV"

echo "Output root: $OUTPUT_ROOT"
echo "Stage1 config: $STAGE1_CFG"
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

for index in 0 1 2; do
  run_case "$index"
done

echo
echo "Switching from baseline to stage1."
for index in 3 4 5; do
  run_case "$index"
done

cleanup_launch

echo
echo "Generating summary table..."
generate_summary
echo
echo "Done."
echo "Summary: $RESULTS_MD"
echo "Logs:    $LOG_DIR"
echo "JSON:    $JSON_DIR"
