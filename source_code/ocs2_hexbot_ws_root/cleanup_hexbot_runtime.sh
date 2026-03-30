#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${HEXBOT_JAZZY_CONTAINER_NAME:-ocs2_hexbot_jazzy}"
HEXBOT_SUDO_PASSWORD="${HEXBOT_SUDO_PASSWORD:-}"

docker_cmd() {
  if [[ -n "${HEXBOT_SUDO_PASSWORD}" ]]; then
    printf '%s\n' "${HEXBOT_SUDO_PASSWORD}" | sudo -S docker "$@"
    return
  fi
  sudo docker "$@"
}

HOST_PATTERNS=(
  "hexbot_locomotion_mpc_bridge.py"
)

CONTAINER_PATTERNS=(
  "ros2 launch ocs2_hexbot_legged_robot_ros hexbot_sqp.launch.py"
  "legged_robot_sqp_mpc"
  "legged_robot_ipm_mpc"
  "hexbot_runtime_mrt"
  "hexbot_cmd_vel_target"
  "hexbot_auto_gait_command"
  "robot_state_publisher"
)

echo "[info] stopping host Hexbot runtime processes"
for pattern in "${HOST_PATTERNS[@]}"; do
  pkill -9 -f "${pattern}" >/dev/null 2>&1 || true
done

if ! docker_cmd ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "[info] Jazzy container is not running: ${CONTAINER_NAME}"
  exit 0
fi

echo "[info] stopping container Hexbot runtime processes in ${CONTAINER_NAME}"
container_process_list="$(docker_cmd exec -i "${CONTAINER_NAME}" ps -eo pid=,cmd= || true)"
container_pids=()
while IFS= read -r line; do
  [[ -z "${line}" ]] && continue
  pid="$(awk '{print $1}' <<<"${line}")"
  cmd="${line#${pid}}"
  for pattern in "${CONTAINER_PATTERNS[@]}"; do
    if [[ "${cmd}" == *"${pattern}"* ]]; then
      container_pids+=("${pid}")
      echo "[killed] ${pid} ${cmd}"
      break
    fi
  done
done <<<"${container_process_list}"

if [[ ${#container_pids[@]} -gt 0 ]]; then
  docker_cmd exec -i "${CONTAINER_NAME}" kill -9 "${container_pids[@]}" >/dev/null 2>&1 || true
fi

echo "[done] Hexbot runtime cleanup complete"
