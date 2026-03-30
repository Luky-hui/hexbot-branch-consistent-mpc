#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_ROOT="$(cd "${PACKAGE_DIR}/.." && pwd)"
TARGET_DIR="${SRC_ROOT}/third_party/mpc_wbc_sources"

if ! command -v git >/dev/null 2>&1; then
  echo "[error] git not found in PATH"
  exit 1
fi

mkdir -p "${TARGET_DIR}"
echo "[info] cloning MPC/WBC sources into ${TARGET_DIR}"

clone_if_missing() {
  local url="$1"
  local name="$2"
  local target="${TARGET_DIR}/${name}"
  if [[ -d "${target}/.git" ]]; then
    echo "[skip] ${name} already exists"
    return
  fi
  git clone "${url}" "${target}"
}

clone_if_missing "https://ghproxy.com/https://github.com/osqp/osqp.git" "osqp"
clone_if_missing "https://ghproxy.com/https://github.com/coin-or/qpOASES.git" "qpOASES"
clone_if_missing "https://ghproxy.com/https://github.com/leggedrobotics/ocs2.git" "ocs2"
clone_if_missing "https://ghproxy.com/https://github.com/qiayuanl/legged_control.git" "legged_control"
clone_if_missing "https://github.com/Renforce-Dynamics/MultiModalWBC.git" "MultiModalWBC"

cat <<'EOF'

[done] repositories prepared

Role summary:
  - ocs2: primary MPC line for Hexbot migration, use ros2 branch.
  - osqp: default QP solver for the first integration pass.
  - qpOASES: optional alternate QP solver.
  - legged_control: architecture reference only, not a direct ROS 2 dependency.
  - MultiModalWBC: research/reference source only, not the first production backend.
EOF
