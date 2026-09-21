# Branch-Consistent MPC for Real-Time Hexapod Locomotion

This repository accompanies the study **"Interior-Point Model Predictive Control with Solver-Visible Branch-Consistency Constraints for Real-Time Hexapod Locomotion."** It preserves the ROS 2 and OCS2 controller sources, Isaac Sim experiment records, analysis outputs, figures, and manuscript sources used to study physically invalid tarsus branches in hexapod locomotion.

![Methodology overview](paper_figures_and_analysis/figures/methodology_overview.svg)

## Overview

The controller encodes left/right tarsus branch admissibility directly in the optimal-control problem through `TarsusBranchConstraint`. The constrained problem is solved with an Interior-Point Method (IPM), keeping the physical branch condition visible to the solver instead of relying only on a post-optimization runtime guard.

The manuscript reports:

- branch-guard events reduced from `275.9` to `0.0` per short-horizon run;
- tracking RMSE reduced from `0.242 m` to `0.175 m`;
- `4.82 ms` average MPC solve time in the controlled short-horizon comparison;
- `0.306 m` mean forward displacement over five repeated 20-second endurance runs, with zero guard and reject events; and
- `0.638 m` forward displacement in the reported 60-second endurance evaluation, with `8.27 deg` yaw drift, `0.011 m` body-height range, and `4.63 ms` average solve time.

See the [English technical report](docs/manuscript/technical_report_en.pdf) or [Chinese technical report](docs/manuscript/technical_report_zh.pdf) for the full method, evaluation protocol, and discussion.

## Repository Layout

```text
.
|-- source_code/
|   |-- ocs2_hexbot_ws_root/       # Reproduction and container scripts
|   |-- ocs2_hexbot_ws_src/        # ROS 2 packages and OCS2 controller sources
|   `-- hexbot_paper_icra_v1/      # English and Chinese LaTeX sources
|-- experiment_data/               # Raw JSON experiment records
|-- paper_figures_and_analysis/     # Aggregated CSV files and publication figures
`-- docs/
    |-- ARCHIVE.md                  # Archive provenance and scope
    `-- manuscript/                 # English and Chinese technical reports
```

The internal layout of `source_code/`, `experiment_data/`, and `paper_figures_and_analysis/` is intentionally preserved so that archived relative references remain intact.

## Reproduction Entry Points

The primary reproduction guide is [source_code/ocs2_hexbot_ws_root/README_PAPER.md](source_code/ocs2_hexbot_ws_root/README_PAPER.md). The main entry points are:

- `source_code/ocs2_hexbot_ws_root/reproduce_paper_results.sh`
- `source_code/ocs2_hexbot_ws_src/hexbot_locomotion_mpc/src/run_experiments_batch.py`
- `source_code/ocs2_hexbot_ws_src/hexbot_locomotion_mpc/src/analyze_hexbot_metrics.py`
- `source_code/ocs2_hexbot_ws_src/hexbot_locomotion_mpc/src/plot_hexbot_results.py`

The archived environment targets Ubuntu 22.04, Python 3.10, Docker, ROS 2 Jazzy, and NVIDIA Isaac Sim 4.5.0. Some scripts retain the original workspace path `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws`; map that path in the runtime environment or update it consistently before execution.

## Data and Analysis

Raw records are documented in [experiment_data/README.md](experiment_data/README.md). Aggregated metrics and ready-to-view SVG/PDF figures are under `paper_figures_and_analysis/`.

![Forward displacement and tracking RMSE](paper_figures_and_analysis/figures/figure1_forward_displacement_and_tracking_rmse.svg)

![Branch guard events and maximum still duration](paper_figures_and_analysis/figures/figure2_guard_and_max_still_boxplots.svg)

![MPC solve time and real-time budget](paper_figures_and_analysis/figures/figure3_mpc_solve_time_with_realtime_budget.svg)

## Manuscript Sources

The LaTeX source is preserved in `source_code/hexbot_paper_icra_v1/`:

- `paper.tex` and `sections/` contain the English manuscript source;
- `paper_zh.tex` and `sections_zh/` contain the Chinese manuscript source; and
- `figures/`, `tools/`, and `refs.bib` contain shared assets and references.
