#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${HEXBOT_JAZZY_CONTAINER_NAME:-ocs2_hexbot_jazzy}"
IMAGE_NAME="${HEXBOT_JAZZY_IMAGE_NAME:-osrf/ros:jazzy-desktop}"
WORKSPACE_HOST_ROOT="/home/u-zhuang/ros2_ws"
MPC_WBC_HOST_ROOT="/home/u-zhuang/ros2_ws/src/liuzu/third_party/mpc_wbc_sources"
DOCKER_BIN="${DOCKER_BIN:-sudo docker}"

container_exists() {
  ${DOCKER_BIN} ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"
}

container_running() {
  ${DOCKER_BIN} ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"
}

if container_running; then
  echo "[info] container already running: ${CONTAINER_NAME}"
  exit 0
fi

if container_exists; then
  echo "[info] starting existing container: ${CONTAINER_NAME}"
  ${DOCKER_BIN} start "${CONTAINER_NAME}" >/dev/null
  echo "[done] container started: ${CONTAINER_NAME}"
  exit 0
fi

MOUNTS=(
  -v "${WORKSPACE_HOST_ROOT}:${WORKSPACE_HOST_ROOT}"
)

if [[ -d "${MPC_WBC_HOST_ROOT}" ]]; then
  MOUNTS+=(
    -v "${MPC_WBC_HOST_ROOT}:/home/u-zhuang/mpc_wbc_sources"
  )
fi

echo "[info] creating detached Jazzy container: ${CONTAINER_NAME}"
${DOCKER_BIN} run -d \
  --name "${CONTAINER_NAME}" \
  --net=host \
  --ipc=host \
  "${MOUNTS[@]}" \
  "${IMAGE_NAME}" \
  bash -lc 'sleep infinity' >/dev/null

echo "[done] container ready: ${CONTAINER_NAME}"
echo "[hint] exec into it with: bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_exec_hexbot_jazzy.sh"
