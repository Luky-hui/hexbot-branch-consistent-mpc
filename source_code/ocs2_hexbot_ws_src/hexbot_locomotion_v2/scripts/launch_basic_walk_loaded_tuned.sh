#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws"
GEOMETRY_CONFIG="${WORKSPACE_ROOT}/src/hexbot_locomotion_v2/config/hexbot_v2_stage2_loaded_equilibrium_candidate.yaml"

exec "${WORKSPACE_ROOT}/src/hexbot_locomotion_v2/scripts/launch_basic_walk_validated.sh" \
  geometry_config_path:="${GEOMETRY_CONFIG}" \
  tripod_turn_stride_m:=0.03 \
  "$@"
