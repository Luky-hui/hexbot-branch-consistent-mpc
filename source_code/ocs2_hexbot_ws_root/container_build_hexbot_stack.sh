#!/usr/bin/env bash
set -euo pipefail

if [[ ! -d /opt/ros/jazzy ]]; then
  echo "[error] /opt/ros/jazzy is missing. Run this script inside the Jazzy container." >&2
  exit 1
fi

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

WORKSPACE_ROOT="/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws"
UPSTREAM_OCS2_ROOT="${WORKSPACE_ROOT}/.tmp_upstream/ocs2_ros2/ocs2-ros2"
UPSTREAM_OCS2_ASSETS_ROOT="${UPSTREAM_OCS2_ROOT}/ocs2_robotic_assets"
WORKSPACE_OCS2_ROOT="${WORKSPACE_ROOT}/src/ocs2"
CONTAINER_BUILD_BASE="${WORKSPACE_ROOT}/build_jazzy"
CONTAINER_INSTALL_BASE="${WORKSPACE_ROOT}/install_jazzy"
CONTAINER_LOG_BASE="${WORKSPACE_ROOT}/log_jazzy"
SOURCE_PATHS=(
  "${WORKSPACE_ROOT}/src/ocs2"
  "${WORKSPACE_ROOT}/src/ocs2_hexbot_legged_robot"
  "${WORKSPACE_ROOT}/src/ocs2_hexbot_legged_robot_ros"
  "${WORKSPACE_ROOT}/src/hexbot_description_ros2"
  "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc"
)

if [[ ! -d "${WORKSPACE_ROOT}" ]]; then
  echo "[error] workspace root not found: ${WORKSPACE_ROOT}" >&2
  exit 1
fi

ensure_upstream_ocs2_visible() {
  if [[ -d "${WORKSPACE_OCS2_ROOT}/ocs2_core" ]]; then
    echo "[info] upstream OCS2 sources already visible at ${WORKSPACE_OCS2_ROOT}"
    return
  fi

  if [[ ! -d "${UPSTREAM_OCS2_ROOT}/ocs2_core" ]]; then
    echo "[error] upstream OCS2 source root not found: ${UPSTREAM_OCS2_ROOT}" >&2
    echo "[hint] restore or reclone the upstream ros2 branch under .tmp_upstream first" >&2
    exit 1
  fi

  echo "[info] staging upstream OCS2 sources into workspace src/ocs2"
  rm -rf "${WORKSPACE_OCS2_ROOT}"

  if ln -s "${UPSTREAM_OCS2_ROOT}" "${WORKSPACE_OCS2_ROOT}" 2>/dev/null; then
    echo "[info] created symlink: ${WORKSPACE_OCS2_ROOT} -> ${UPSTREAM_OCS2_ROOT}"
    return
  fi

  mkdir -p "${WORKSPACE_OCS2_ROOT}"
  cp -a "${UPSTREAM_OCS2_ROOT}/." "${WORKSPACE_OCS2_ROOT}/"
  echo "[warn] symlink unavailable; copied upstream OCS2 sources into ${WORKSPACE_OCS2_ROOT}"
}

ensure_upstream_ocs2_assets_visible() {
  if [[ -d "${UPSTREAM_OCS2_ASSETS_ROOT}" ]]; then
    echo "[info] upstream OCS2 robotic assets already visible at ${UPSTREAM_OCS2_ASSETS_ROOT}"
    return
  fi

  if ! command -v git >/dev/null 2>&1; then
    echo "[error] git is required to fetch missing ocs2_robotic_assets" >&2
    exit 1
  fi

  echo "[info] cloning missing upstream dependency: ocs2_robotic_assets"
  git clone --depth 1 \
    https://github.com/leggedrobotics/ocs2_robotic_assets.git \
    "${UPSTREAM_OCS2_ASSETS_ROOT}"
}

cd "${WORKSPACE_ROOT}"
ensure_upstream_ocs2_visible
ensure_upstream_ocs2_assets_visible

echo "[info] building OCS2 Hexbot stack in ${WORKSPACE_ROOT}"
echo "[info] using dependency closure instead of a hand-maintained packages-select list"
echo "[info] using isolated Jazzy artifacts:"
echo "       build=${CONTAINER_BUILD_BASE}"
echo "       install=${CONTAINER_INSTALL_BASE}"
echo "       log=${CONTAINER_LOG_BASE}"
echo "[info] explicit source paths:"
printf '       %s\n' "${SOURCE_PATHS[@]}"

rm -rf "${CONTAINER_BUILD_BASE}" "${CONTAINER_INSTALL_BASE}" "${CONTAINER_LOG_BASE}"

colcon \
  --log-base "${CONTAINER_LOG_BASE}" \
  build \
  --base-paths "${SOURCE_PATHS[@]}" \
  --symlink-install \
  --build-base "${CONTAINER_BUILD_BASE}" \
  --install-base "${CONTAINER_INSTALL_BASE}" \
  --packages-up-to ocs2_hexbot_legged_robot_ros hexbot_locomotion_mpc \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DBUILD_TESTING=OFF \
    -DOCS2_HEXBOT_BUILD_UPSTREAM_TESTS=OFF

echo "[done] build finished"
echo "[hint] source ${CONTAINER_INSTALL_BASE}/setup.bash"
