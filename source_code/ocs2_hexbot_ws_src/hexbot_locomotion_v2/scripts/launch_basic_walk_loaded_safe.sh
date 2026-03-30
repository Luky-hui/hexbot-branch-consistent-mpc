#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws"
GEOMETRY_CONFIG="${WORKSPACE_ROOT}/src/hexbot_locomotion_v2/config/hexbot_v2_basic_walk_loaded_safe.yaml"

set +u
source /opt/ros/humble/setup.bash
source /home/u-zhuang/ros2_ws/install/setup.bash
if [[ -f "${WORKSPACE_ROOT}/install_humble/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/install_humble/setup.bash"
fi
set -u

exec ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py \
  use_sim_time:=true \
  mode:=tripod \
  locomotion_startup_mode:=fixed_stand \
  geometry_config_path:="${GEOMETRY_CONFIG}" \
  "$@"
