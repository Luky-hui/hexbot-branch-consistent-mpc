# Hexbot Paper Archive v1

This archive preserves the final paper-related source code, experiment records, and analysis assets for the Hexbot hexapod locomotion study.

## Scope

This archive captures the code and data associated with the final solver-visible branch-consistency solution:

- OCS2 controller path upgraded from SQP to IPM
- TarsusBranchConstraint integrated into the solver-visible feasible set
- Endurance forward-walking evaluations and batch analysis outputs
- Paper-ready LaTeX sources and vector figures

## Directory Layout

- `source_code/`
  - Clean project sources copied from the active workspace
  - Excludes build artifacts, install trees, `.tmp_upstream`, and cache folders
- `experiment_data/`
  - Raw JSON experiment records copied from the Isaac/ROS closed-loop log directory
  - Batch metadata copied from the result folders
- `paper_figures_and_analysis/`
  - CSV summary tables
  - Analysis manifests
  - Paper figures in PDF/SVG format

## Notes

- This archive is intended as a cold backup for preservation and later reproduction.
- The copied source tree intentionally excludes transient build outputs.
- Experiment JSON files were copied from the current workspace state at archive time.
- Paper figures include the final methodology overview and endurance time-series figures used for the manuscript.
