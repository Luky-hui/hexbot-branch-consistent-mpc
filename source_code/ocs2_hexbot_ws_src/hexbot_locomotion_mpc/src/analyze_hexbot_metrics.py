#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


THIS_FILE = Path(__file__).resolve()
WORKSPACE_ROOT = THIS_FILE.parents[3]
DEFAULT_RESULTS_ROOT = WORKSPACE_ROOT / "src" / "hexbot_locomotion_mpc" / "results"


GUARD_PATTERN = re.compile(
    r"Applied joint branch-consistency guard[^\n]*joint=([A-Za-z0-9_]+)"
)


def safe_float(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return float(value)
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def safe_int(value: Any) -> int | None:
    numeric = safe_float(value)
    if numeric is None:
        return None
    return int(round(numeric))


def bool_to_int(value: Any) -> int:
    return 1 if bool(value) else 0


def mean(values: Iterable[float]) -> float | None:
    sequence = [float(v) for v in values if v is not None]
    if not sequence:
        return None
    return sum(sequence) / float(len(sequence))


def variance(values: Iterable[float]) -> float | None:
    sequence = [float(v) for v in values if v is not None]
    if not sequence:
        return None
    mu = mean(sequence)
    if mu is None:
        return None
    return sum((value - mu) ** 2 for value in sequence) / float(len(sequence))


def stddev(values: Iterable[float]) -> float | None:
    value = variance(values)
    return None if value is None else math.sqrt(value)


def rmse(values: Iterable[float]) -> float | None:
    sequence = [float(v) for v in values if v is not None]
    if not sequence:
        return None
    return math.sqrt(sum(value * value for value in sequence) / float(len(sequence)))


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except json.JSONDecodeError:
        return None


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def discover_batch_roots(input_path: Path) -> list[Path]:
    input_path = input_path.resolve()
    if (input_path / "batch_manifest.json").is_file():
        return [input_path]
    return sorted(
        path.parent for path in input_path.rglob("batch_manifest.json")
    )


def discover_summary_files(batch_root: Path) -> list[Path]:
    return sorted(
        path
        for path in batch_root.rglob("*_summary.json")
        if "_generated_tasks" not in path.parts
    )


def infer_group_slug(summary_path: Path, batch_root: Path) -> str:
    try:
        rel = summary_path.relative_to(batch_root)
    except ValueError:
        return summary_path.parent.name
    return rel.parts[0] if rel.parts else summary_path.parent.name


def load_manifest_group_map(batch_root: Path) -> dict[str, dict[str, Any]]:
    manifest = load_json(batch_root / "batch_manifest.json") or {}
    mapping: dict[str, dict[str, Any]] = {}
    for group in manifest.get("groups", []) or []:
        slug = str(group.get("group_slug") or "").strip()
        if slug:
            mapping[slug] = dict(group)
    return mapping


def parse_guard_counts(summary: dict[str, Any], container_log: Path) -> tuple[int | None, dict[str, int]]:
    metrics = (
        ((summary.get("solver_trace") or {}).get("branch_guard") or {}).get("metrics")
        or {}
    )
    joint_counts = (
        ((summary.get("solver_trace") or {}).get("branch_guard") or {}).get("dominant_guard_joints")
        or {}
    )
    total = safe_int(metrics.get("total_events"))
    parsed_joint_counts: dict[str, int] = {}
    if isinstance(joint_counts, dict):
        for key, value in joint_counts.items():
            count = safe_int(value)
            if count is not None:
                parsed_joint_counts[str(key)] = count

    if total is not None:
        return total, parsed_joint_counts

    try:
        text = container_log.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return None, parsed_joint_counts

    counter = Counter(GUARD_PATTERN.findall(text))
    return sum(counter.values()), dict(counter)


def extract_solver_values(summary: dict[str, Any]) -> dict[str, str]:
    for container in (
        ((summary.get("ocs2_topics") or {}).get("solver_capture_snapshot") or {}).get("status"),
        ((summary.get("probe") or {}).get("solver_diagnostics") or {}).get("last_status"),
    ):
        if not container:
            continue
        for status in container:
            values = status.get("values") or {}
            if values:
                return {str(key): str(value) for key, value in values.items()}
    return {}


@dataclass
class RunRecord:
    batch_id: str
    group: str
    group_slug: str
    run_tag: str
    solver: str | None
    task_branch_constraint_enabled: str | None
    summary_file: str
    posture_file: str | None
    container_log: str | None
    target_distance_source: str
    target_distance_m: float | None
    target_distance_cmd_m: float | None
    target_distance_target_snapshot_last_m: float | None
    isaac_forward_m: float | None
    odom_forward_m: float | None
    isaac_yaw_drift_rad: float | None
    odom_yaw_drift_rad: float | None
    yaw_drift_deg: float | None
    z_height_mean_m: float | None
    z_height_variance_m2: float | None
    z_height_std_m: float | None
    z_height_range_m: float | None
    tracking_error_m: float | None
    tracking_rmse_m: float | None
    guard_total_events: int | None
    reject_backend_joint_command: int | None
    max_still_sec: float | None
    trunk_sink_detected: int
    weird_pose_detected: int
    mpc_solve_time_avg_ms: float | None
    mpc_solve_time_last_ms: float | None
    mpc_solve_time_max_ms: float | None
    mpc_solve_time_samples: int | None
    solver_timeout_threshold_ms: float | None
    solver_timeout_exceed_count: int | None
    solver_timeout_exceed_rate: float | None
    solver_timeout_flag: int
    solver_perf_cost: float | None
    solver_perf_ineq_constraints_sse: float | None
    solver_perf_eq_constraints_sse: float | None
    mode_sequence: str
    probe_ready: int


def build_run_record(
    *,
    batch_id: str,
    batch_root: Path,
    summary_path: Path,
    group_map: dict[str, dict[str, Any]],
    target_distance_source: str,
) -> RunRecord:
    summary = load_json(summary_path) or {}
    run_tag = summary_path.name[: -len("_summary.json")]
    group_slug = infer_group_slug(summary_path, batch_root)
    group_info = group_map.get(group_slug, {})
    group_name = str(group_info.get("name") or group_slug)

    posture_path = summary_path.with_name(f"{run_tag}_posture.json")
    container_log = summary_path.with_name(f"{run_tag}_container.log")

    posture = summary.get("posture") or load_json(posture_path) or {}
    motion = (summary.get("probe") or {}).get("motion") or {}
    isaac_motion = posture.get("isaac_motion") or {}
    posture_metrics = posture.get("metrics") or {}
    solver_timing_summary = (
        ((summary.get("ocs2_topics") or {}).get("solver_timing_summary")) or {}
    )

    cmd_vel = summary.get("cmd_vel") or {}
    target_distance_cmd_m = None
    linear_x = safe_float(cmd_vel.get("linear_x"))
    probe_duration_sec = safe_float(cmd_vel.get("probe_duration_sec"))
    if linear_x is not None and probe_duration_sec is not None:
        target_distance_cmd_m = linear_x * probe_duration_sec

    target_snapshot = ((summary.get("ocs2_topics") or {}).get("target_snapshot")) or {}
    target_distance_target_snapshot_last_m = safe_float(
        ((target_snapshot.get("delta_body_last") or {}).get("forward_m"))
    )
    if target_distance_source == "target_snapshot_last":
        target_distance_m = target_distance_target_snapshot_last_m
    else:
        target_distance_m = target_distance_cmd_m

    isaac_forward_m = safe_float(isaac_motion.get("delta_forward_m"))
    odom_forward_m = safe_float(motion.get("delta_forward_m"))
    isaac_yaw_drift_rad = safe_float(isaac_motion.get("delta_yaw_rad"))
    odom_yaw_drift_rad = safe_float(posture_metrics.get("odom_delta_yaw_rad"))
    yaw_drift_deg = safe_float(posture_metrics.get("yaw_drift_deg"))
    z_height_mean_m = safe_float(posture_metrics.get("z_height_mean_m"))
    z_height_variance_m2 = safe_float(posture_metrics.get("z_height_variance_m2"))
    z_height_std_m = safe_float(posture_metrics.get("z_height_std_m"))
    z_height_range_m = safe_float(posture_metrics.get("z_height_range_m"))
    tracking_error_m = None
    tracking_rmse_m = None
    if isaac_forward_m is not None and target_distance_m is not None:
        tracking_error_m = isaac_forward_m - target_distance_m
        tracking_rmse_m = abs(tracking_error_m)

    guard_total_events, _ = parse_guard_counts(summary, container_log)
    solver_values = extract_solver_values(summary)
    mode_sequence = (
        (((summary.get("ocs2_topics") or {}).get("mode_schedule_snapshot")) or {}).get("mode_sequence")
        or []
    )

    return RunRecord(
        batch_id=batch_id,
        group=group_name,
        group_slug=group_slug,
        run_tag=run_tag,
        solver=str(summary.get("solver")) if summary.get("solver") is not None else None,
        task_branch_constraint_enabled=(
            str(group_info.get("enable_tarsus_branch_constraint"))
            if "enable_tarsus_branch_constraint" in group_info
            else None
        ),
        summary_file=str(summary_path),
        posture_file=str(posture_path) if posture_path.exists() else None,
        container_log=str(container_log) if container_log.exists() else None,
        target_distance_source=target_distance_source,
        target_distance_m=target_distance_m,
        target_distance_cmd_m=target_distance_cmd_m,
        target_distance_target_snapshot_last_m=target_distance_target_snapshot_last_m,
        isaac_forward_m=isaac_forward_m,
        odom_forward_m=odom_forward_m,
        isaac_yaw_drift_rad=isaac_yaw_drift_rad,
        odom_yaw_drift_rad=odom_yaw_drift_rad,
        yaw_drift_deg=yaw_drift_deg,
        z_height_mean_m=z_height_mean_m,
        z_height_variance_m2=z_height_variance_m2,
        z_height_std_m=z_height_std_m,
        z_height_range_m=z_height_range_m,
        tracking_error_m=tracking_error_m,
        tracking_rmse_m=tracking_rmse_m,
        guard_total_events=guard_total_events,
        reject_backend_joint_command=safe_int(
            ((summary.get("counts") or {}).get("reject_backend_joint_command"))
        ),
        max_still_sec=safe_float(motion.get("max_still_sec")),
        trunk_sink_detected=bool_to_int(posture.get("trunk_sink_detected")),
        weird_pose_detected=bool_to_int(posture.get("weird_pose_detected")),
        mpc_solve_time_avg_ms=safe_float(solver_values.get("solve_time_avg_ms")),
        mpc_solve_time_last_ms=safe_float(solver_values.get("solve_time_last_ms")),
        mpc_solve_time_max_ms=safe_float(solver_values.get("solve_time_max_ms")),
        mpc_solve_time_samples=safe_int(solver_values.get("solve_time_samples")),
        solver_timeout_threshold_ms=safe_float(
            solver_timing_summary.get("timeout_threshold_ms")
        ),
        solver_timeout_exceed_count=safe_int(
            solver_timing_summary.get("timeout_exceed_count")
        ),
        solver_timeout_exceed_rate=safe_float(
            solver_timing_summary.get("timeout_exceed_rate")
        ),
        solver_timeout_flag=bool_to_int(
            (safe_int(solver_timing_summary.get("timeout_exceed_count")) or 0) > 0
        ),
        solver_perf_cost=safe_float(solver_values.get("perf_cost")),
        solver_perf_ineq_constraints_sse=safe_float(
            solver_values.get("perf_ineq_constraints_sse")
        ),
        solver_perf_eq_constraints_sse=safe_float(
            solver_values.get("perf_eq_constraints_sse")
        ),
        mode_sequence=json.dumps(mode_sequence, ensure_ascii=False),
        probe_ready=bool_to_int((summary.get("probe") or {}).get("ready")),
    )


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def summarize_group(
    *,
    batch_id: str,
    group_name: str,
    group_slug: str,
    records: list[RunRecord],
    runs_index: list[dict[str, Any]],
) -> dict[str, Any]:
    attempted_labels = {
        str(row.get("label_prefix"))
        for row in runs_index
        if str(row.get("group_slug")) == group_slug
    }
    valid_records = records

    errors = [record.tracking_error_m for record in valid_records if record.tracking_error_m is not None]
    isaac_values = [record.isaac_forward_m for record in valid_records if record.isaac_forward_m is not None]
    guard_values = [float(record.guard_total_events) for record in valid_records if record.guard_total_events is not None]
    reject_values = [float(record.reject_backend_joint_command) for record in valid_records if record.reject_backend_joint_command is not None]
    still_values = [record.max_still_sec for record in valid_records if record.max_still_sec is not None]
    yaw_abs_deg_values = [
        abs(record.yaw_drift_deg)
        for record in valid_records
        if record.yaw_drift_deg is not None
    ]
    z_height_variance_values = [
        record.z_height_variance_m2
        for record in valid_records
        if record.z_height_variance_m2 is not None
    ]
    z_height_range_values = [
        record.z_height_range_m
        for record in valid_records
        if record.z_height_range_m is not None
    ]
    solve_avg_values = [record.mpc_solve_time_avg_ms for record in valid_records if record.mpc_solve_time_avg_ms is not None]
    solve_last_values = [record.mpc_solve_time_last_ms for record in valid_records if record.mpc_solve_time_last_ms is not None]
    solve_max_values = [record.mpc_solve_time_max_ms for record in valid_records if record.mpc_solve_time_max_ms is not None]
    timeout_count_values = [
        float(record.solver_timeout_exceed_count)
        for record in valid_records
        if record.solver_timeout_exceed_count is not None
    ]
    timeout_rate_values = [
        record.solver_timeout_exceed_rate
        for record in valid_records
        if record.solver_timeout_exceed_rate is not None
    ]
    timeout_flag_values = [float(record.solver_timeout_flag) for record in valid_records]

    return {
        "batch_id": batch_id,
        "group": group_name,
        "group_slug": group_slug,
        "attempted_logical_runs": len(attempted_labels) if attempted_labels else len(valid_records),
        "completed_runs_with_summary": len(valid_records),
        "tracking_rmse_m": rmse(errors),
        "tracking_error_mean_m": mean(errors),
        "tracking_error_std_m": stddev(errors),
        "isaac_forward_mean_m": mean(isaac_values),
        "isaac_forward_std_m": stddev(isaac_values),
        "guard_total_mean": mean(guard_values),
        "guard_total_std": stddev(guard_values),
        "guard_total_sum": sum(guard_values) if guard_values else None,
        "reject_count_mean": mean(reject_values),
        "reject_count_std": stddev(reject_values),
        "reject_count_sum": sum(reject_values) if reject_values else None,
        "max_still_sec_mean": mean(still_values),
        "max_still_sec_std": stddev(still_values),
        "yaw_drift_abs_mean_deg": mean(yaw_abs_deg_values),
        "yaw_drift_abs_std_deg": stddev(yaw_abs_deg_values),
        "z_height_variance_mean_m2": mean(z_height_variance_values),
        "z_height_variance_std_m2": stddev(z_height_variance_values),
        "z_height_range_mean_m": mean(z_height_range_values),
        "z_height_range_std_m": stddev(z_height_range_values),
        "trunk_sink_rate": mean(record.trunk_sink_detected for record in valid_records),
        "weird_pose_rate": mean(record.weird_pose_detected for record in valid_records),
        "mpc_solve_time_avg_ms_mean": mean(solve_avg_values),
        "mpc_solve_time_avg_ms_variance": variance(solve_avg_values),
        "mpc_solve_time_last_ms_mean": mean(solve_last_values),
        "mpc_solve_time_last_ms_variance": variance(solve_last_values),
        "mpc_solve_time_max_ms_mean": mean(solve_max_values),
        "mpc_solve_time_max_ms_variance": variance(solve_max_values),
        "solver_timeout_count_mean": mean(timeout_count_values),
        "solver_timeout_count_std": stddev(timeout_count_values),
        "solver_timeout_rate_mean": mean(timeout_rate_values),
        "solver_timeout_rate_std": stddev(timeout_rate_values),
        "solver_timeout_run_rate": mean(timeout_flag_values),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Extract Hexbot walking metrics from archived batch results and export "
            "paper-ready CSV tables."
        )
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_RESULTS_ROOT,
        help="Either one batch directory or the top-level results root.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for CSV outputs. Defaults to <input>/analysis or <batch>/analysis.",
    )
    parser.add_argument(
        "--target-distance-source",
        choices=("cmd_vel", "target_snapshot_last"),
        default="cmd_vel",
        help=(
            "Reference distance used for tracking RMSE. "
            "`cmd_vel` uses linear_x * probe_duration_sec; "
            "`target_snapshot_last` uses the last OCS2 target delta_body.forward_m."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    batch_roots = discover_batch_roots(args.input)
    if not batch_roots:
        raise SystemExit(f"No batch_manifest.json found under {args.input}")

    if args.output_dir is not None:
        output_dir = args.output_dir.resolve()
    elif len(batch_roots) == 1:
        output_dir = (batch_roots[0] / "analysis").resolve()
    else:
        output_dir = (args.input.resolve() / "analysis").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    per_run_rows: list[dict[str, Any]] = []
    group_records: dict[tuple[str, str, str], list[RunRecord]] = defaultdict(list)
    runs_index_by_batch: dict[str, list[dict[str, Any]]] = {}

    for batch_root in batch_roots:
        batch_id = batch_root.name.removeprefix("batch_")
        runs_index = load_jsonl(batch_root / "runs.jsonl")
        runs_index_by_batch[batch_id] = runs_index
        group_map = load_manifest_group_map(batch_root)
        for summary_path in discover_summary_files(batch_root):
            record = build_run_record(
                batch_id=batch_id,
                batch_root=batch_root,
                summary_path=summary_path,
                group_map=group_map,
                target_distance_source=args.target_distance_source,
            )
            row = record.__dict__.copy()
            per_run_rows.append(row)
            group_records[(record.batch_id, record.group, record.group_slug)].append(record)

    if not per_run_rows:
        raise SystemExit("No *_summary.json artifacts were found to analyze.")

    per_run_fieldnames = [
        "batch_id",
        "group",
        "group_slug",
        "run_tag",
        "solver",
        "task_branch_constraint_enabled",
        "summary_file",
        "posture_file",
        "container_log",
        "target_distance_source",
        "target_distance_m",
        "target_distance_cmd_m",
        "target_distance_target_snapshot_last_m",
        "isaac_forward_m",
        "odom_forward_m",
        "isaac_yaw_drift_rad",
        "odom_yaw_drift_rad",
        "yaw_drift_deg",
        "z_height_mean_m",
        "z_height_variance_m2",
        "z_height_std_m",
        "z_height_range_m",
        "tracking_error_m",
        "tracking_rmse_m",
        "guard_total_events",
        "reject_backend_joint_command",
        "max_still_sec",
        "trunk_sink_detected",
        "weird_pose_detected",
        "mpc_solve_time_avg_ms",
        "mpc_solve_time_last_ms",
        "mpc_solve_time_max_ms",
        "mpc_solve_time_samples",
        "solver_timeout_threshold_ms",
        "solver_timeout_exceed_count",
        "solver_timeout_exceed_rate",
        "solver_timeout_flag",
        "solver_perf_cost",
        "solver_perf_ineq_constraints_sse",
        "solver_perf_eq_constraints_sse",
        "mode_sequence",
        "probe_ready",
    ]
    write_csv(output_dir / "hexbot_metrics_per_run.csv", per_run_rows, per_run_fieldnames)

    group_summary_rows: list[dict[str, Any]] = []
    for (batch_id, group_name, group_slug), records in sorted(group_records.items()):
        group_summary_rows.append(
            summarize_group(
                batch_id=batch_id,
                group_name=group_name,
                group_slug=group_slug,
                records=records,
                runs_index=runs_index_by_batch.get(batch_id, []),
            )
        )

    group_fieldnames = [
        "batch_id",
        "group",
        "group_slug",
        "attempted_logical_runs",
        "completed_runs_with_summary",
        "tracking_rmse_m",
        "tracking_error_mean_m",
        "tracking_error_std_m",
        "isaac_forward_mean_m",
        "isaac_forward_std_m",
        "guard_total_mean",
        "guard_total_std",
        "guard_total_sum",
        "reject_count_mean",
        "reject_count_std",
        "reject_count_sum",
        "max_still_sec_mean",
        "max_still_sec_std",
        "yaw_drift_abs_mean_deg",
        "yaw_drift_abs_std_deg",
        "z_height_variance_mean_m2",
        "z_height_variance_std_m2",
        "z_height_range_mean_m",
        "z_height_range_std_m",
        "trunk_sink_rate",
        "weird_pose_rate",
        "mpc_solve_time_avg_ms_mean",
        "mpc_solve_time_avg_ms_variance",
        "mpc_solve_time_last_ms_mean",
        "mpc_solve_time_last_ms_variance",
        "mpc_solve_time_max_ms_mean",
        "mpc_solve_time_max_ms_variance",
        "solver_timeout_count_mean",
        "solver_timeout_count_std",
        "solver_timeout_rate_mean",
        "solver_timeout_rate_std",
        "solver_timeout_run_rate",
    ]
    write_csv(output_dir / "hexbot_metrics_group_summary.csv", group_summary_rows, group_fieldnames)

    analysis_manifest = {
        "input": str(args.input.resolve()),
        "batch_roots": [str(path) for path in batch_roots],
        "output_dir": str(output_dir),
        "target_distance_source": args.target_distance_source,
        "per_run_csv": str(output_dir / "hexbot_metrics_per_run.csv"),
        "group_summary_csv": str(output_dir / "hexbot_metrics_group_summary.csv"),
        "total_runs_analyzed": len(per_run_rows),
        "total_groups_analyzed": len(group_summary_rows),
    }
    (output_dir / "analysis_manifest.json").write_text(
        json.dumps(analysis_manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(json.dumps(analysis_manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
