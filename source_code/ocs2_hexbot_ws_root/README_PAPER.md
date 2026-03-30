# Hexbot OCS2 Paper Reproduction

This repository accompanies our Hexbot locomotion study on **branch-consistent hexapod MPC**, where an **Interior-Point Method (IPM)** backend plus a **TarsusBranchConstraint** removes physically inconsistent tarsus solutions that remained feasible under the earlier SQP pipeline.

## Abstract in One Sentence

We show that encoding left/right tarsus branch consistency as a solver-visible hard inequality and solving the resulting problem with IPM eliminates branch-guard violations, restores stable forward walking, and preserves real-time MPC execution at approximately `4.8 ms` average solve time.

## Repository Scope

The paper release includes:

- the Hexbot OCS2 controller stack used for the final experiments
- the solver-side branch-consistency constraint
- the batch experiment runner
- the metric extraction pipeline
- the vector figures and CSV summaries used in the paper

The paper-ready analysis artifacts are committed under:

- `src/hexbot_locomotion_mpc/results/batch_paper_forward_v1/analysis`

Raw runtime logs and per-run probe dumps are intentionally excluded from version control to keep the repository lightweight and review-friendly.

## Prerequisites

### Host

- Ubuntu `22.04`
- Python `3.10`
- Docker
- NVIDIA Isaac Sim `4.5.0`

### Containerized Build Environment

The controller stack is built and executed inside:

- Docker image: `osrf/ros:jazzy-desktop`
- ROS 2 distribution: `Jazzy`

Install container dependencies with:

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_run_hexbot_jazzy.sh
printf 'root\n' | sudo -S -k docker exec -i ocs2_hexbot_jazzy bash -lc \
  "cd /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws && bash container_setup_env.sh"
```

Then build with:

```bash
printf 'root\n' | sudo -S -k docker exec -i ocs2_hexbot_jazzy bash -lc \
  "cd /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws && bash container_build_hexbot_stack.sh"
```

### Python Packages

Install the paper-analysis Python dependencies on the host with:

```bash
python3 -m pip install -r /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/requirements_paper.txt
```

`seaborn` is optional. The plotting script falls back to native `matplotlib` styling if `seaborn` is not installed.

## Reproduce the Results

Ensure that:

1. Isaac Sim `4.5.0` is running with the Hexbot stage loaded.
2. The ROS bridge is active and `/clock`, `/joint_states`, and `/odom` are live.
3. The Jazzy container has already been built as described above.

Then run the full paper pipeline with a **single command**:

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/reproduce_paper_results.sh \
  --runs 10 \
  --batch-id paper_forward_v1_repro
```

This command will:

1. batch-run the `Proposed` and `Baseline_SQP` conditions
2. extract the paper metrics into CSV tables
3. render the final `.pdf` and `.svg` figures

The main entrypoints are:

- `src/hexbot_locomotion_mpc/src/run_experiments_batch.py`
- `src/hexbot_locomotion_mpc/src/analyze_hexbot_metrics.py`
- `src/hexbot_locomotion_mpc/src/plot_hexbot_results.py`

## Released Experimental Conditions

### Proposed

- solver: `ipm`
- `enableTarsusBranchConstraint = true`

### Baseline_SQP

- solver: `sqp`
- `enableTarsusBranchConstraint = false`

## Reported Paper Metrics

The analysis pipeline exports:

- forward displacement
- tracking RMSE
- branch-guard event counts
- backend reject counts
- maximum still duration
- trunk-sink and weird-pose rates
- MPC solve-time statistics

The committed paper figures are:

- `analysis/figures/figure1_forward_displacement_and_tracking_rmse.pdf`
- `analysis/figures/figure2_guard_and_max_still_boxplots.pdf`
- `analysis/figures/figure3_mpc_solve_time_with_realtime_budget.pdf`

## Notes on Reproducibility

- The accepted walking metric is computed along the robot's `base_link` forward axis in Isaac Sim, rather than raw world-frame `x`, because the released Hexbot asset spawns approximately facing world `-X`.
- The released controller defaults to `IPM` for paper reproduction because the previous SQP path did not enforce the state inequality in a behaviorally equivalent way during locomotion.
- Additional environment details are frozen in `paper_environment.yml`.
