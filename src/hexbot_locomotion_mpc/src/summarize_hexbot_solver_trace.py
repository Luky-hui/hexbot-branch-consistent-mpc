#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from statistics import mean
from typing import Any


TRACE_PREFIX = "HEXBOT_SOLVER_TRACE "


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize structured HEXBOT_SOLVER_TRACE lines from an OCS2 container log."
    )
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--base-sink-threshold-m",
        type=float,
        default=0.03,
        help="Treat optimized base height below target by more than this margin as a crouch sample.",
    )
    parser.add_argument("--top-samples", type=int, default=12)
    return parser.parse_args()


def load_traces(text: str) -> list[dict[str, Any]]:
    traces: list[dict[str, Any]] = []
    for line in text.splitlines():
        marker = line.find(TRACE_PREFIX)
        if marker < 0:
            continue
        payload = line[marker + len(TRACE_PREFIX) :].strip()
        if not payload:
            continue
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            traces.append(parsed)
    return traces


def is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and float(value) == float(value)


def safe_mean(values: list[float]) -> float | None:
    if not values:
        return None
    return float(mean(values))


def summarize_traces(
    traces: list[dict[str, Any]], base_sink_threshold_m: float, top_samples: int
) -> dict[str, Any]:
    mode_counts = Counter()
    target_height_gaps: list[float] = []
    current_support_height_deltas: list[float] = []
    support_height_deltas: list[float] = []
    measured_joint_deltas: list[float] = []
    measured_target_joint_deltas: list[float] = []
    target_joint_deltas: list[float] = []
    stance_force_ratios: list[float] = []
    foot_max_position_errors: list[float] = []
    foot_max_abs_z_deltas: list[float] = []
    foot_most_negative_normal_deltas: list[float] = []
    current_target_foot_max_position_errors: list[float] = []
    current_target_foot_max_abs_z_deltas: list[float] = []
    current_target_foot_most_negative_normal_deltas: list[float] = []
    optimized_current_foot_max_position_errors: list[float] = []
    optimized_current_foot_max_abs_z_deltas: list[float] = []
    optimized_current_foot_most_negative_normal_deltas: list[float] = []
    target_touchdown_anchor_max_position_errors: list[float] = []
    current_touchdown_anchor_max_position_errors: list[float] = []
    optimized_touchdown_anchor_max_position_errors: list[float] = []
    target_joint_counts = Counter()
    measured_joint_counts = Counter()
    measured_target_joint_counts = Counter()
    foot_max_position_counts = Counter()
    foot_max_abs_z_counts = Counter()
    foot_most_negative_normal_counts = Counter()
    current_target_foot_max_position_counts = Counter()
    current_target_foot_max_abs_z_counts = Counter()
    current_target_foot_most_negative_normal_counts = Counter()
    optimized_current_foot_max_position_counts = Counter()
    optimized_current_foot_max_abs_z_counts = Counter()
    optimized_current_foot_most_negative_normal_counts = Counter()
    target_touchdown_anchor_max_position_counts = Counter()
    current_touchdown_anchor_max_position_counts = Counter()
    optimized_touchdown_anchor_max_position_counts = Counter()
    crouch_samples: list[dict[str, Any]] = []
    foot_drop_samples: list[dict[str, Any]] = []
    current_foot_drop_samples: list[dict[str, Any]] = []
    execution_gap_samples: list[dict[str, Any]] = []
    target_touchdown_anchor_samples: list[dict[str, Any]] = []
    current_touchdown_anchor_samples: list[dict[str, Any]] = []
    optimized_touchdown_anchor_samples: list[dict[str, Any]] = []

    for trace in traces:
        planned_mode = trace.get("planned_mode")
        if planned_mode is not None:
            mode_counts[str(planned_mode)] += 1

        measured_joint = trace.get("optimized_vs_measured_joint")
        if measured_joint:
            measured_joint_counts[str(measured_joint)] += 1

        measured_target_joint = trace.get("measured_vs_target_joint")
        if measured_target_joint:
            measured_target_joint_counts[str(measured_target_joint)] += 1

        target_joint = trace.get("optimized_vs_target_joint")
        if target_joint:
            target_joint_counts[str(target_joint)] += 1

        measured_delta = trace.get("optimized_vs_measured_joint_max_abs_delta")
        if is_finite_number(measured_delta):
            measured_joint_deltas.append(float(measured_delta))

        measured_target_delta = trace.get("measured_vs_target_joint_max_abs_delta")
        if is_finite_number(measured_target_delta):
            measured_target_joint_deltas.append(float(measured_target_delta))

        target_delta = trace.get("optimized_vs_target_joint_max_abs_delta")
        if is_finite_number(target_delta):
            target_joint_deltas.append(float(target_delta))

        stance_force_ratio = trace.get("stance_force_ratio")
        if is_finite_number(stance_force_ratio):
            stance_force_ratios.append(float(stance_force_ratio))

        support_height_delta = trace.get("optimized_minus_target_support_height")
        if is_finite_number(support_height_delta):
            support_height_deltas.append(float(support_height_delta))

        current_support_height_delta = trace.get("current_minus_target_support_height")
        if is_finite_number(current_support_height_delta):
            current_support_height_deltas.append(float(current_support_height_delta))

        foot_max_position_error = trace.get("optimized_vs_target_foot_max_position_error")
        foot_max_position_leg = trace.get("optimized_vs_target_foot_max_position_leg")
        if is_finite_number(foot_max_position_error):
            foot_max_position_errors.append(float(foot_max_position_error))
        if foot_max_position_leg:
            foot_max_position_counts[str(foot_max_position_leg)] += 1

        foot_max_abs_z_delta = trace.get("optimized_vs_target_foot_max_abs_z_delta")
        foot_max_abs_z_leg = trace.get("optimized_vs_target_foot_max_abs_z_leg")
        if is_finite_number(foot_max_abs_z_delta):
            foot_max_abs_z_deltas.append(float(foot_max_abs_z_delta))
        if foot_max_abs_z_leg:
            foot_max_abs_z_counts[str(foot_max_abs_z_leg)] += 1

        foot_most_negative_normal_delta = trace.get(
            "optimized_vs_target_foot_most_negative_normal_delta"
        )
        foot_most_negative_leg = trace.get("optimized_vs_target_foot_most_negative_leg")
        if is_finite_number(foot_most_negative_normal_delta):
            foot_most_negative_normal_deltas.append(
                float(foot_most_negative_normal_delta)
            )
            foot_drop_samples.append(
                {
                    "time": trace.get("time"),
                    "planned_mode": planned_mode,
                    "leg": foot_most_negative_leg,
                    "most_negative_normal_delta_m": float(
                        foot_most_negative_normal_delta
                    ),
                    "max_position_error_m": foot_max_position_error,
                    "max_position_error_leg": foot_max_position_leg,
                    "max_abs_z_delta_m": foot_max_abs_z_delta,
                    "max_abs_z_delta_leg": foot_max_abs_z_leg,
                    "signed_z_delta_m": trace.get("optimized_vs_target_foot_signed_z_delta"),
                    "optimized_base_z": trace.get("optimized_base_z"),
                    "target_interpolated_base_z": trace.get(
                        "target_interpolated_base_z"
                    ),
                    "optimized_minus_target_support_height_m": support_height_delta,
                }
            )
        if foot_most_negative_leg:
            foot_most_negative_normal_counts[str(foot_most_negative_leg)] += 1

        current_target_foot_max_position_error = trace.get(
            "current_vs_target_foot_max_position_error"
        )
        current_target_foot_max_position_leg = trace.get(
            "current_vs_target_foot_max_position_leg"
        )
        if is_finite_number(current_target_foot_max_position_error):
            current_target_foot_max_position_errors.append(
                float(current_target_foot_max_position_error)
            )
        if current_target_foot_max_position_leg:
            current_target_foot_max_position_counts[
                str(current_target_foot_max_position_leg)
            ] += 1

        current_target_foot_max_abs_z_delta = trace.get(
            "current_vs_target_foot_max_abs_z_delta"
        )
        current_target_foot_max_abs_z_leg = trace.get(
            "current_vs_target_foot_max_abs_z_leg"
        )
        if is_finite_number(current_target_foot_max_abs_z_delta):
            current_target_foot_max_abs_z_deltas.append(
                float(current_target_foot_max_abs_z_delta)
            )
        if current_target_foot_max_abs_z_leg:
            current_target_foot_max_abs_z_counts[
                str(current_target_foot_max_abs_z_leg)
            ] += 1

        current_target_foot_most_negative_normal_delta = trace.get(
            "current_vs_target_foot_most_negative_normal_delta"
        )
        current_target_foot_most_negative_leg = trace.get(
            "current_vs_target_foot_most_negative_leg"
        )
        if is_finite_number(current_target_foot_most_negative_normal_delta):
            current_target_foot_most_negative_normal_deltas.append(
                float(current_target_foot_most_negative_normal_delta)
            )
            current_foot_drop_samples.append(
                {
                    "time": trace.get("time"),
                    "planned_mode": planned_mode,
                    "leg": current_target_foot_most_negative_leg,
                    "most_negative_normal_delta_m": float(
                        current_target_foot_most_negative_normal_delta
                    ),
                    "max_position_error_m": current_target_foot_max_position_error,
                    "max_position_error_leg": current_target_foot_max_position_leg,
                    "max_abs_z_delta_m": current_target_foot_max_abs_z_delta,
                    "max_abs_z_delta_leg": current_target_foot_max_abs_z_leg,
                    "signed_z_delta_m": trace.get(
                        "current_vs_target_foot_signed_z_delta"
                    ),
                    "current_base_z": trace.get("current_base_z"),
                    "target_interpolated_base_z": trace.get(
                        "target_interpolated_base_z"
                    ),
                    "current_minus_target_support_height_m": current_support_height_delta,
                }
            )
        if current_target_foot_most_negative_leg:
            current_target_foot_most_negative_normal_counts[
                str(current_target_foot_most_negative_leg)
            ] += 1

        optimized_current_foot_max_position_error = trace.get(
            "optimized_vs_current_foot_max_position_error"
        )
        optimized_current_foot_max_position_leg = trace.get(
            "optimized_vs_current_foot_max_position_leg"
        )
        if is_finite_number(optimized_current_foot_max_position_error):
            optimized_current_foot_max_position_errors.append(
                float(optimized_current_foot_max_position_error)
            )
        if optimized_current_foot_max_position_leg:
            optimized_current_foot_max_position_counts[
                str(optimized_current_foot_max_position_leg)
            ] += 1

        optimized_current_foot_max_abs_z_delta = trace.get(
            "optimized_vs_current_foot_max_abs_z_delta"
        )
        optimized_current_foot_max_abs_z_leg = trace.get(
            "optimized_vs_current_foot_max_abs_z_leg"
        )
        if is_finite_number(optimized_current_foot_max_abs_z_delta):
            optimized_current_foot_max_abs_z_deltas.append(
                float(optimized_current_foot_max_abs_z_delta)
            )
        if optimized_current_foot_max_abs_z_leg:
            optimized_current_foot_max_abs_z_counts[
                str(optimized_current_foot_max_abs_z_leg)
            ] += 1

        optimized_current_foot_most_negative_normal_delta = trace.get(
            "optimized_vs_current_foot_most_negative_normal_delta"
        )
        optimized_current_foot_most_negative_leg = trace.get(
            "optimized_vs_current_foot_most_negative_leg"
        )
        if is_finite_number(optimized_current_foot_most_negative_normal_delta):
            optimized_current_foot_most_negative_normal_deltas.append(
                float(optimized_current_foot_most_negative_normal_delta)
            )
            execution_gap_samples.append(
                {
                    "time": trace.get("time"),
                    "planned_mode": planned_mode,
                    "leg": optimized_current_foot_most_negative_leg,
                    "most_negative_normal_delta_m": float(
                        optimized_current_foot_most_negative_normal_delta
                    ),
                    "max_position_error_m": optimized_current_foot_max_position_error,
                    "max_position_error_leg": optimized_current_foot_max_position_leg,
                    "max_abs_z_delta_m": optimized_current_foot_max_abs_z_delta,
                    "max_abs_z_delta_leg": optimized_current_foot_max_abs_z_leg,
                    "signed_z_delta_m": trace.get(
                        "optimized_vs_current_foot_signed_z_delta"
                    ),
                    "optimized_base_z": trace.get("optimized_base_z"),
                    "current_base_z": trace.get("current_base_z"),
                    "current_minus_target_support_height_m": current_support_height_delta,
                }
            )
        if optimized_current_foot_most_negative_leg:
            optimized_current_foot_most_negative_normal_counts[
                str(optimized_current_foot_most_negative_leg)
            ] += 1

        target_touchdown_anchor_max_position_error = trace.get(
            "target_vs_touchdown_anchor_stance_max_position_error"
        )
        target_touchdown_anchor_max_position_leg = trace.get(
            "target_vs_touchdown_anchor_stance_max_position_leg"
        )
        if is_finite_number(target_touchdown_anchor_max_position_error):
            target_touchdown_anchor_max_position_errors.append(
                float(target_touchdown_anchor_max_position_error)
            )
            target_touchdown_anchor_samples.append(
                {
                    "time": trace.get("time"),
                    "planned_mode": planned_mode,
                    "leg": target_touchdown_anchor_max_position_leg,
                    "max_position_error_m": float(
                        target_touchdown_anchor_max_position_error
                    ),
                    "stance_foot_count": trace.get(
                        "touchdown_anchor_stance_foot_count"
                    ),
                }
            )
        if target_touchdown_anchor_max_position_leg:
            target_touchdown_anchor_max_position_counts[
                str(target_touchdown_anchor_max_position_leg)
            ] += 1

        current_touchdown_anchor_max_position_error = trace.get(
            "current_vs_touchdown_anchor_stance_max_position_error"
        )
        current_touchdown_anchor_max_position_leg = trace.get(
            "current_vs_touchdown_anchor_stance_max_position_leg"
        )
        if is_finite_number(current_touchdown_anchor_max_position_error):
            current_touchdown_anchor_max_position_errors.append(
                float(current_touchdown_anchor_max_position_error)
            )
            current_touchdown_anchor_samples.append(
                {
                    "time": trace.get("time"),
                    "planned_mode": planned_mode,
                    "leg": current_touchdown_anchor_max_position_leg,
                    "max_position_error_m": float(
                        current_touchdown_anchor_max_position_error
                    ),
                    "stance_foot_count": trace.get(
                        "touchdown_anchor_stance_foot_count"
                    ),
                }
            )
        if current_touchdown_anchor_max_position_leg:
            current_touchdown_anchor_max_position_counts[
                str(current_touchdown_anchor_max_position_leg)
            ] += 1

        optimized_touchdown_anchor_max_position_error = trace.get(
            "optimized_vs_touchdown_anchor_stance_max_position_error"
        )
        optimized_touchdown_anchor_max_position_leg = trace.get(
            "optimized_vs_touchdown_anchor_stance_max_position_leg"
        )
        if is_finite_number(optimized_touchdown_anchor_max_position_error):
            optimized_touchdown_anchor_max_position_errors.append(
                float(optimized_touchdown_anchor_max_position_error)
            )
            optimized_touchdown_anchor_samples.append(
                {
                    "time": trace.get("time"),
                    "planned_mode": planned_mode,
                    "leg": optimized_touchdown_anchor_max_position_leg,
                    "max_position_error_m": float(
                        optimized_touchdown_anchor_max_position_error
                    ),
                    "stance_foot_count": trace.get(
                        "touchdown_anchor_stance_foot_count"
                    ),
                }
            )
        if optimized_touchdown_anchor_max_position_leg:
            optimized_touchdown_anchor_max_position_counts[
                str(optimized_touchdown_anchor_max_position_leg)
            ] += 1

        optimized_height = trace.get("optimized_base_height_above_support")
        target_height = trace.get("target_base_height_above_support")
        if is_finite_number(optimized_height) and is_finite_number(target_height):
            gap = float(optimized_height) - float(target_height)
            target_height_gaps.append(gap)
            if gap < -abs(base_sink_threshold_m):
                crouch_samples.append(
                    {
                        "time": trace.get("time"),
                        "planned_mode": planned_mode,
                        "height_gap_m": gap,
                        "optimized_base_height_above_support": float(optimized_height),
                        "target_base_height_above_support": float(target_height),
                        "optimized_base_z": trace.get("optimized_base_z"),
                        "target_interpolated_base_z": trace.get(
                            "target_interpolated_base_z"
                        ),
                        "optimized_vs_target_joint_max_abs_delta": trace.get(
                            "optimized_vs_target_joint_max_abs_delta"
                        ),
                        "optimized_vs_target_joint": target_joint,
                        "stance_force_ratio": stance_force_ratio,
                        "stance_normal_forces": trace.get("stance_normal_forces", {}),
                    }
                )

    crouch_samples.sort(
        key=lambda item: float(item.get("height_gap_m", 0.0))
    )
    foot_drop_samples.sort(
        key=lambda item: float(item.get("most_negative_normal_delta_m", 0.0))
    )
    current_foot_drop_samples.sort(
        key=lambda item: float(item.get("most_negative_normal_delta_m", 0.0))
    )
    execution_gap_samples.sort(
        key=lambda item: float(item.get("most_negative_normal_delta_m", 0.0))
    )
    target_touchdown_anchor_samples.sort(
        key=lambda item: float(item.get("max_position_error_m", 0.0)),
        reverse=True,
    )
    current_touchdown_anchor_samples.sort(
        key=lambda item: float(item.get("max_position_error_m", 0.0)),
        reverse=True,
    )
    optimized_touchdown_anchor_samples.sort(
        key=lambda item: float(item.get("max_position_error_m", 0.0)),
        reverse=True,
    )

    summary = {
        "metrics": {
            "total_samples": len(traces),
            "first_time": traces[0].get("time") if traces else None,
            "last_time": traces[-1].get("time") if traces else None,
            "mode_counts": dict(mode_counts),
            "optimized_minus_target_base_height_mean_m": safe_mean(
                target_height_gaps
            ),
            "optimized_minus_target_base_height_min_m": (
                min(target_height_gaps) if target_height_gaps else None
            ),
            "optimized_minus_target_base_height_max_m": (
                max(target_height_gaps) if target_height_gaps else None
            ),
            "optimized_minus_target_support_height_mean_m": safe_mean(
                support_height_deltas
            ),
            "optimized_minus_target_support_height_min_m": (
                min(support_height_deltas) if support_height_deltas else None
            ),
            "optimized_minus_target_support_height_max_m": (
                max(support_height_deltas) if support_height_deltas else None
            ),
            "current_minus_target_support_height_mean_m": safe_mean(
                current_support_height_deltas
            ),
            "current_minus_target_support_height_min_m": (
                min(current_support_height_deltas)
                if current_support_height_deltas
                else None
            ),
            "current_minus_target_support_height_max_m": (
                max(current_support_height_deltas)
                if current_support_height_deltas
                else None
            ),
            "crouch_sample_count": len(crouch_samples),
            "crouch_sample_ratio": (
                float(len(crouch_samples)) / float(len(traces)) if traces else None
            ),
            "measured_vs_target_joint_max_abs_delta_mean_rad": safe_mean(
                measured_target_joint_deltas
            ),
            "measured_vs_target_joint_max_abs_delta_max_rad": (
                max(measured_target_joint_deltas)
                if measured_target_joint_deltas
                else None
            ),
            "optimized_vs_target_joint_max_abs_delta_mean_rad": safe_mean(
                target_joint_deltas
            ),
            "optimized_vs_target_joint_max_abs_delta_max_rad": (
                max(target_joint_deltas) if target_joint_deltas else None
            ),
            "current_vs_target_foot_max_position_error_mean_m": safe_mean(
                current_target_foot_max_position_errors
            ),
            "current_vs_target_foot_max_position_error_max_m": (
                max(current_target_foot_max_position_errors)
                if current_target_foot_max_position_errors
                else None
            ),
            "current_vs_target_foot_max_abs_z_delta_mean_m": safe_mean(
                current_target_foot_max_abs_z_deltas
            ),
            "current_vs_target_foot_max_abs_z_delta_max_m": (
                max(current_target_foot_max_abs_z_deltas)
                if current_target_foot_max_abs_z_deltas
                else None
            ),
            "current_vs_target_foot_most_negative_normal_delta_mean_m": safe_mean(
                current_target_foot_most_negative_normal_deltas
            ),
            "current_vs_target_foot_most_negative_normal_delta_min_m": (
                min(current_target_foot_most_negative_normal_deltas)
                if current_target_foot_most_negative_normal_deltas
                else None
            ),
            "optimized_vs_target_foot_max_position_error_mean_m": safe_mean(
                foot_max_position_errors
            ),
            "optimized_vs_target_foot_max_position_error_max_m": (
                max(foot_max_position_errors) if foot_max_position_errors else None
            ),
            "optimized_vs_target_foot_max_abs_z_delta_mean_m": safe_mean(
                foot_max_abs_z_deltas
            ),
            "optimized_vs_target_foot_max_abs_z_delta_max_m": (
                max(foot_max_abs_z_deltas) if foot_max_abs_z_deltas else None
            ),
            "optimized_vs_target_foot_most_negative_normal_delta_mean_m": safe_mean(
                foot_most_negative_normal_deltas
            ),
            "optimized_vs_target_foot_most_negative_normal_delta_min_m": (
                min(foot_most_negative_normal_deltas)
                if foot_most_negative_normal_deltas
                else None
            ),
            "optimized_vs_current_foot_max_position_error_mean_m": safe_mean(
                optimized_current_foot_max_position_errors
            ),
            "optimized_vs_current_foot_max_position_error_max_m": (
                max(optimized_current_foot_max_position_errors)
                if optimized_current_foot_max_position_errors
                else None
            ),
            "optimized_vs_current_foot_max_abs_z_delta_mean_m": safe_mean(
                optimized_current_foot_max_abs_z_deltas
            ),
            "optimized_vs_current_foot_max_abs_z_delta_max_m": (
                max(optimized_current_foot_max_abs_z_deltas)
                if optimized_current_foot_max_abs_z_deltas
                else None
            ),
            "optimized_vs_current_foot_most_negative_normal_delta_mean_m": safe_mean(
                optimized_current_foot_most_negative_normal_deltas
            ),
            "optimized_vs_current_foot_most_negative_normal_delta_min_m": (
                min(optimized_current_foot_most_negative_normal_deltas)
                if optimized_current_foot_most_negative_normal_deltas
                else None
            ),
            "target_vs_touchdown_anchor_stance_max_position_error_mean_m": safe_mean(
                target_touchdown_anchor_max_position_errors
            ),
            "target_vs_touchdown_anchor_stance_max_position_error_max_m": (
                max(target_touchdown_anchor_max_position_errors)
                if target_touchdown_anchor_max_position_errors
                else None
            ),
            "current_vs_touchdown_anchor_stance_max_position_error_mean_m": safe_mean(
                current_touchdown_anchor_max_position_errors
            ),
            "current_vs_touchdown_anchor_stance_max_position_error_max_m": (
                max(current_touchdown_anchor_max_position_errors)
                if current_touchdown_anchor_max_position_errors
                else None
            ),
            "optimized_vs_touchdown_anchor_stance_max_position_error_mean_m": safe_mean(
                optimized_touchdown_anchor_max_position_errors
            ),
            "optimized_vs_touchdown_anchor_stance_max_position_error_max_m": (
                max(optimized_touchdown_anchor_max_position_errors)
                if optimized_touchdown_anchor_max_position_errors
                else None
            ),
            "optimized_vs_measured_joint_max_abs_delta_mean_rad": safe_mean(
                measured_joint_deltas
            ),
            "optimized_vs_measured_joint_max_abs_delta_max_rad": (
                max(measured_joint_deltas) if measured_joint_deltas else None
            ),
            "stance_force_ratio_mean": safe_mean(stance_force_ratios),
            "stance_force_ratio_max": (
                max(stance_force_ratios) if stance_force_ratios else None
            ),
        },
        "dominant_joints": {
            "measured_vs_target": dict(measured_target_joint_counts.most_common(8)),
            "optimized_vs_target": dict(target_joint_counts.most_common(8)),
            "optimized_vs_measured": dict(measured_joint_counts.most_common(8)),
        },
        "dominant_feet": {
            "current_vs_target_max_position_error": dict(
                current_target_foot_max_position_counts.most_common(8)
            ),
            "current_vs_target_max_abs_z_delta": dict(
                current_target_foot_max_abs_z_counts.most_common(8)
            ),
            "current_vs_target_most_negative_normal_delta": dict(
                current_target_foot_most_negative_normal_counts.most_common(8)
            ),
            "max_position_error": dict(foot_max_position_counts.most_common(8)),
            "max_abs_z_delta": dict(foot_max_abs_z_counts.most_common(8)),
            "most_negative_normal_delta": dict(
                foot_most_negative_normal_counts.most_common(8)
            ),
            "optimized_vs_current_max_position_error": dict(
                optimized_current_foot_max_position_counts.most_common(8)
            ),
            "optimized_vs_current_max_abs_z_delta": dict(
                optimized_current_foot_max_abs_z_counts.most_common(8)
            ),
            "optimized_vs_current_most_negative_normal_delta": dict(
                optimized_current_foot_most_negative_normal_counts.most_common(8)
            ),
            "target_vs_touchdown_anchor_stance_max_position_error": dict(
                target_touchdown_anchor_max_position_counts.most_common(8)
            ),
            "current_vs_touchdown_anchor_stance_max_position_error": dict(
                current_touchdown_anchor_max_position_counts.most_common(8)
            ),
            "optimized_vs_touchdown_anchor_stance_max_position_error": dict(
                optimized_touchdown_anchor_max_position_counts.most_common(8)
            ),
        },
        "worst_crouch_samples": crouch_samples[: max(1, int(top_samples))],
        "worst_current_foot_drop_samples": current_foot_drop_samples[
            : max(1, int(top_samples))
        ],
        "worst_execution_gap_samples": execution_gap_samples[
            : max(1, int(top_samples))
        ],
        "worst_foot_drop_samples": foot_drop_samples[: max(1, int(top_samples))],
        "worst_target_touchdown_anchor_samples": target_touchdown_anchor_samples[
            : max(1, int(top_samples))
        ],
        "worst_current_touchdown_anchor_samples": current_touchdown_anchor_samples[
            : max(1, int(top_samples))
        ],
        "worst_optimized_touchdown_anchor_samples": optimized_touchdown_anchor_samples[
            : max(1, int(top_samples))
        ],
    }
    return summary


def main() -> int:
    args = parse_args()
    text = args.log.read_text(encoding="utf-8", errors="replace")
    traces = load_traces(text)
    payload = {
        "log_file": str(args.log.resolve()),
        "thresholds": {
            "base_sink_threshold_m": float(args.base_sink_threshold_m),
        },
        **summarize_traces(
            traces,
            base_sink_threshold_m=float(args.base_sink_threshold_m),
            top_samples=int(args.top_samples),
        ),
    }
    text_out = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text_out + "\n", encoding="utf-8")
    print(text_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
