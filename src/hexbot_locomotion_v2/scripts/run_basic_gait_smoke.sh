#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${PKG_DIR}/../.." && pwd)"

JOINT_CONFIG="${WORKSPACE_ROOT}/src/hexbot_description_ros2/config/joint_name_list.yaml"
BASE_CONFIG="${PKG_DIR}/config/hexbot_v2_stage2_explicit_current_candidate.yaml"
WAIT_SCRIPT="${SCRIPT_DIR}/wait_joint_state_convergence.py"
ODOM_RUNNER="${SCRIPT_DIR}/odom_ab_runner.py"

# Some ROS setup scripts still assume a few tracing vars may be unset.
# Temporarily relax nounset while importing those environments.
set +u
source /opt/ros/humble/setup.bash
source /home/u-zhuang/ros2_ws/install/setup.bash
if [[ -f "${WORKSPACE_ROOT}/install_humble/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/install_humble/setup.bash"
fi
set -u

echo "[info] checking stand convergence against ${BASE_CONFIG}"
python3 "${WAIT_SCRIPT}" \
  --joint-config "${JOINT_CONFIG}" \
  --geometry-config "${BASE_CONFIG}" \
  --joint-state-topic /joint_states \
  --use-sim-time \
  --max-joint-diff-rad 0.12 \
  --stable-samples 5 \
  --timeout-sec 4.0

echo "[info] running forward smoke (5s @ 0.03 m/s)"
forward_json="$(
  python3 "${ODOM_RUNNER}" \
    --use-sim-time \
    --duration 5.0 \
    --linear-x 0.03 \
    --angular-z 0.0 \
    --cmd-publish-rate 15.0
)"
echo "${forward_json}"

echo "[info] running turn smoke (5s @ 0.12 rad/s)"
turn_json="$(
  python3 "${ODOM_RUNNER}" \
    --use-sim-time \
    --duration 5.0 \
    --linear-x 0.0 \
    --angular-z 0.12 \
    --cmd-publish-rate 15.0
)"
echo "${turn_json}"

python3 - <<'PY' "${forward_json}" "${turn_json}"
import json
import math
import sys

forward = json.loads(sys.argv[1])
turn = json.loads(sys.argv[2])

summary = {
    "forward_pass": (
        forward["delta_forward_m"] is not None
        and forward["delta_forward_m"] > 0.08
        and abs(forward["delta_lateral_m"]) < 0.03
        and abs(forward["delta_yaw_rad"]) < 0.08
        and forward["max_still_sec"] < 0.25
    ),
    "turn_pass": (
        turn["delta_yaw_rad"] is not None
        and abs(turn["delta_yaw_rad"]) > 0.40
        and math.hypot(turn["delta_x_m"], turn["delta_y_m"]) < 0.06
        and turn["max_still_sec"] < 0.25
    ),
    "forward": forward,
    "turn": turn,
}

print(json.dumps(summary, indent=2, sort_keys=True))
if not summary["forward_pass"] or not summary["turn_pass"]:
    raise SystemExit(1)
PY
