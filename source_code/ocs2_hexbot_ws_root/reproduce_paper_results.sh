#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
RESULTS_ROOT="${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/results"
BATCH_ID="paper_forward_v1_repro"
RUNS="10"
FORCE="false"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --batch-id)
      BATCH_ID="$2"
      shift 2
      ;;
    --runs)
      RUNS="$2"
      shift 2
      ;;
    --force)
      FORCE="true"
      shift
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

BATCH_ROOT="${RESULTS_ROOT}/batch_${BATCH_ID}"

if [[ -e "${BATCH_ROOT}" && "${FORCE}" != "true" ]]; then
  echo "[error] batch output already exists: ${BATCH_ROOT}" >&2
  echo "[hint] use --batch-id <new_id> or rerun with --force" >&2
  exit 2
fi

if [[ -e "${BATCH_ROOT}" && "${FORCE}" == "true" ]]; then
  rm -rf "${BATCH_ROOT}"
fi

python3 "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/run_experiments_batch.py" \
  --runs "${RUNS}" \
  --batch-id "${BATCH_ID}" \
  "${EXTRA_ARGS[@]}"

python3 "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/analyze_hexbot_metrics.py" \
  --input "${BATCH_ROOT}"

python3 "${WORKSPACE_ROOT}/src/hexbot_locomotion_mpc/src/plot_hexbot_results.py" \
  --input "${BATCH_ROOT}"

echo "[done] paper reproduction artifacts written to ${BATCH_ROOT}/analysis"
