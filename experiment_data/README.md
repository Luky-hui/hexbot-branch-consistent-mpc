# Experiment Data

This directory contains the raw experiment records from the Hexbot MPC simulation study.

- `batch_metadata/` contains batch manifests, run indexes, per-run records, and analysis manifests for the endurance evaluations.
- `idle_reference_probe_json/` contains development, baseline, reference, and closed-loop probe records.
- Aggregated metrics and publication figures are available in `../paper_figures_and_analysis/`.

The JSON records include experiment summaries and captured posture, solver trace, precomputation, ROS 2 topic, runtime topic, probe, and Isaac preflight data. The original filenames and directory layout are preserved to retain the link between each batch manifest and its associated run records.
