#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${HEXBOT_JAZZY_CONTAINER_NAME:-ocs2_hexbot_jazzy}"
DOCKER_BIN="${DOCKER_BIN:-sudo docker}"

if ! ${DOCKER_BIN} ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "[error] container is not running: ${CONTAINER_NAME}" >&2
  echo "[hint] first run: bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_run_hexbot_jazzy.sh" >&2
  exit 1
fi

if [[ $# -eq 0 ]]; then
  exec ${DOCKER_BIN} exec -it "${CONTAINER_NAME}" bash
fi

exec ${DOCKER_BIN} exec -it "${CONTAINER_NAME}" "$@"
