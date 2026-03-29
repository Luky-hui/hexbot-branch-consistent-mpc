#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws"

# ROS setup scripts may touch unset tracing variables.
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
  speed_scale:=0.5 \
  tripod_max_linear_x:=0.05 \
  tripod_max_angular_z:=0.20 \
  tripod_forward_stride_m:=0.04 \
  tripod_turn_stride_m:=0.04 \
  "$@"
