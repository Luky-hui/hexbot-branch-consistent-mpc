#!/usr/bin/env bash
set -euo pipefail

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/humble/setup.bash
set -u

WORKSPACE_ROOT="/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws"
HOST_BUILD_BASE="${WORKSPACE_ROOT}/build_humble"
HOST_INSTALL_BASE="${WORKSPACE_ROOT}/install_humble"
HOST_LOG_BASE="${WORKSPACE_ROOT}/log_humble"

if [[ ! -d "${WORKSPACE_ROOT}" ]]; then
  echo "[error] workspace root not found: ${WORKSPACE_ROOT}" >&2
  exit 1
fi

cd "${WORKSPACE_ROOT}"

echo "[info] building Hexbot host runtime stack in ${WORKSPACE_ROOT}"
echo "[info] using isolated Humble artifacts:"
echo "       build=${HOST_BUILD_BASE}"
echo "       install=${HOST_INSTALL_BASE}"
echo "       log=${HOST_LOG_BASE}"

rm -rf "${HOST_BUILD_BASE}" "${HOST_INSTALL_BASE}" "${HOST_LOG_BASE}"

colcon \
  --log-base "${HOST_LOG_BASE}" \
  build \
  --symlink-install \
  --build-base "${HOST_BUILD_BASE}" \
  --install-base "${HOST_INSTALL_BASE}" \
  --packages-up-to \
    hexbot_description_ros2 \
    hexbot_isaac_runtime \
    hexbot_locomotion_v2 \
    hexbot_locomotion_mpc \
  --cmake-clean-cache

echo "[done] host runtime stack build finished"
echo "[hint] source ${HOST_INSTALL_BASE}/setup.bash"
