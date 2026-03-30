#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

try:
    import seaborn as sns
except ImportError:  # pragma: no cover - seaborn is preferred but optional
    sns = None


THIS_FILE = Path(__file__).resolve()
WORKSPACE_ROOT = THIS_FILE.parents[3]
DEFAULT_RESULTS_ROOT = WORKSPACE_ROOT / "src" / "hexbot_locomotion_mpc" / "results"

PROPOSED_COLOR = "#4C72B0"
BASELINE_COLOR = "#DD8452"
GRID_COLOR = "#B8C0CC"
REALTIME_BUDGET_COLOR = "#C44E52"
DEFAULT_DPI = 400


@dataclass
class GroupMetrics:
    group: str
    display_name: str
    color: str
    isaac_forward_mean_m: float | None
    isaac_forward_std_m: float | None
    tracking_rmse_mean_m: float | None
    tracking_rmse_std_m: float | None
    guard_values: list[float]
    max_still_values: list[float]
    solve_time_avg_values: list[float]


def safe_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def mean(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / float(len(values))


def stddev(values: list[float]) -> float | None:
    if not values:
        return None
    mu = mean(values)
    if mu is None:
        return None
    return math.sqrt(sum((value - mu) ** 2 for value in values) / float(len(values)))


def load_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(f"CSV file not found: {path}")
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def resolve_analysis_dir(input_path: Path) -> Path:
    candidate = input_path.resolve()
    per_run = candidate / "hexbot_metrics_per_run.csv"
    summary = candidate / "hexbot_metrics_group_summary.csv"
    if per_run.is_file() and summary.is_file():
        return candidate

    analysis_dir = candidate / "analysis"
    per_run = analysis_dir / "hexbot_metrics_per_run.csv"
    summary = analysis_dir / "hexbot_metrics_group_summary.csv"
    if per_run.is_file() and summary.is_file():
        return analysis_dir

    raise FileNotFoundError(
        "Could not locate analysis CSV files. "
        f"Expected either {candidate} or {analysis_dir} to contain them."
    )


def configure_plot_style(fontsize: float) -> None:
    plt.rcParams.update(
        {
            "figure.dpi": DEFAULT_DPI,
            "savefig.dpi": DEFAULT_DPI,
            "font.size": fontsize,
            "axes.titlesize": fontsize + 2,
            "axes.labelsize": fontsize + 1,
            "axes.edgecolor": "#333333",
            "axes.linewidth": 1.0,
            "xtick.labelsize": fontsize,
            "ytick.labelsize": fontsize,
            "legend.fontsize": fontsize,
            "legend.frameon": False,
            "grid.color": GRID_COLOR,
            "grid.alpha": 0.45,
            "grid.linestyle": "--",
            "grid.linewidth": 0.8,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
        }
    )
    if sns is not None:
        sns.set_theme(style="whitegrid", palette="colorblind")


def display_name_for_group(group: str) -> str:
    normalized = group.strip().lower()
    if normalized == "proposed":
        return "Proposed"
    if normalized in {"baseline_sqp", "baseline sqp"}:
        return "Baseline SQP"
    return group.replace("_", " ")


def color_for_group(group: str) -> str:
    normalized = group.strip().lower()
    if normalized == "proposed":
        return PROPOSED_COLOR
    return BASELINE_COLOR


def collect_group_metrics(per_run_rows: list[dict[str, str]]) -> list[GroupMetrics]:
    grouped: dict[str, list[dict[str, str]]] = {}
    for row in per_run_rows:
        group = row.get("group", "").strip() or "Unknown"
        grouped.setdefault(group, []).append(row)

    preferred_order = {"Proposed": 0, "Baseline_SQP": 1, "Baseline SQP": 1}

    metrics: list[GroupMetrics] = []
    for group, rows in sorted(
        grouped.items(), key=lambda item: (preferred_order.get(item[0], 99), item[0])
    ):
        isaac_values = [
            value
            for value in (safe_float(row.get("isaac_forward_m")) for row in rows)
            if value is not None
        ]
        rmse_values = [
            value
            for value in (safe_float(row.get("tracking_rmse_m")) for row in rows)
            if value is not None
        ]
        guard_values = [
            value
            for value in (safe_float(row.get("guard_total_events")) for row in rows)
            if value is not None
        ]
        max_still_values = [
            value
            for value in (safe_float(row.get("max_still_sec")) for row in rows)
            if value is not None
        ]
        solve_time_avg_values = [
            value
            for value in (safe_float(row.get("mpc_solve_time_avg_ms")) for row in rows)
            if value is not None
        ]

        metrics.append(
            GroupMetrics(
                group=group,
                display_name=display_name_for_group(group),
                color=color_for_group(group),
                isaac_forward_mean_m=mean(isaac_values),
                isaac_forward_std_m=stddev(isaac_values),
                tracking_rmse_mean_m=mean(rmse_values),
                tracking_rmse_std_m=stddev(rmse_values),
                guard_values=guard_values,
                max_still_values=max_still_values,
                solve_time_avg_values=solve_time_avg_values,
            )
        )

    return metrics


def add_bar_labels(ax: plt.Axes, bars: list[Any], values: list[float | None]) -> None:
    for bar, value in zip(bars, values):
        if value is None:
            continue
        height = bar.get_height()
        offset = 0.02 * max(1.0, abs(height))
        va = "bottom" if height >= 0.0 else "top"
        y = height + offset if height >= 0.0 else height - offset
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            y,
            f"{value:.3f}",
            ha="center",
            va=va,
            fontsize=11,
            color="#222222",
        )


def apply_common_axis_style(ax: plt.Axes) -> None:
    ax.grid(True, axis="y")
    ax.set_axisbelow(True)
    if sns is not None:
        sns.despine(ax=ax, top=True, right=True)
    else:
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)


def set_bar_axis_limits(ax: plt.Axes, values: list[float], errors: list[float]) -> None:
    if not values:
        return
    lower = min(value - error for value, error in zip(values, errors))
    upper = max(value + error for value, error in zip(values, errors))
    if lower >= 0.0:
        lower = 0.0
    margin = max(0.02, 0.15 * max(abs(lower), abs(upper), 1e-6))
    ax.set_ylim(lower - margin, upper + margin)


def render_figure1(metrics: list[GroupMetrics], output_dir: Path) -> list[str]:
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.8), constrained_layout=True)
    labels = [item.display_name for item in metrics]
    colors = [item.color for item in metrics]

    forward_means = [item.isaac_forward_mean_m or 0.0 for item in metrics]
    forward_stds = [item.isaac_forward_std_m or 0.0 for item in metrics]
    rmse_means = [item.tracking_rmse_mean_m or 0.0 for item in metrics]
    rmse_stds = [item.tracking_rmse_std_m or 0.0 for item in metrics]

    error_kw = {"elinewidth": 1.4, "capsize": 6, "capthick": 1.4, "ecolor": "#333333"}
    x = range(len(metrics))

    forward_bars = axes[0].bar(
        x,
        forward_means,
        yerr=forward_stds,
        color=colors,
        error_kw=error_kw,
        width=0.62,
    )
    axes[0].set_xticks(list(x), labels)
    axes[0].set_title("Forward Displacement")
    axes[0].set_ylabel("Forward Displacement (m)")
    apply_common_axis_style(axes[0])
    set_bar_axis_limits(axes[0], forward_means, forward_stds)
    add_bar_labels(axes[0], list(forward_bars), forward_means)

    rmse_bars = axes[1].bar(
        x,
        rmse_means,
        yerr=rmse_stds,
        color=colors,
        error_kw=error_kw,
        width=0.62,
    )
    axes[1].set_xticks(list(x), labels)
    axes[1].set_title("Tracking RMSE")
    axes[1].set_ylabel("Tracking RMSE (m)")
    apply_common_axis_style(axes[1])
    set_bar_axis_limits(axes[1], rmse_means, rmse_stds)
    add_bar_labels(axes[1], list(rmse_bars), rmse_means)

    legend_handles = [
        Patch(facecolor=item.color, edgecolor="none", label=item.display_name)
        for item in metrics
    ]
    fig.legend(handles=legend_handles, loc="upper center", ncol=len(metrics), bbox_to_anchor=(0.5, 1.05))

    stem = output_dir / "figure1_forward_displacement_and_tracking_rmse"
    return save_figure(fig, stem)


def render_colored_boxplot(
    ax: plt.Axes,
    data: list[list[float]],
    labels: list[str],
    colors: list[str],
    ylabel: str,
    title: str,
) -> None:
    box = ax.boxplot(
        data,
        patch_artist=True,
        labels=labels,
        widths=0.55,
        medianprops={"color": "#222222", "linewidth": 1.6},
        whiskerprops={"linewidth": 1.3},
        capprops={"linewidth": 1.3},
        boxprops={"linewidth": 1.3},
        flierprops={
            "marker": "o",
            "markersize": 4,
            "markeredgecolor": "#333333",
            "markerfacecolor": "#FFFFFF",
            "alpha": 0.85,
        },
    )
    for patch, color in zip(box["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.85)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    apply_common_axis_style(ax)
    ax.set_ylim(bottom=0.0)


def render_figure2(metrics: list[GroupMetrics], output_dir: Path) -> list[str]:
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.8), constrained_layout=True)
    labels = [item.display_name for item in metrics]
    colors = [item.color for item in metrics]

    render_colored_boxplot(
        axes[0],
        [item.guard_values for item in metrics],
        labels,
        colors,
        ylabel="Branch Guard Events",
        title="Guard Trigger Distribution",
    )
    render_colored_boxplot(
        axes[1],
        [item.max_still_values for item in metrics],
        labels,
        colors,
        ylabel="Max Still Duration (s)",
        title="Still-Time Distribution",
    )

    legend_handles = [
        Patch(facecolor=item.color, edgecolor="none", label=item.display_name)
        for item in metrics
    ]
    fig.legend(handles=legend_handles, loc="upper center", ncol=len(metrics), bbox_to_anchor=(0.5, 1.05))

    stem = output_dir / "figure2_guard_and_max_still_boxplots"
    return save_figure(fig, stem)


def render_figure3(metrics: list[GroupMetrics], output_dir: Path) -> list[str]:
    fig, ax = plt.subplots(figsize=(6.8, 4.9), constrained_layout=True)
    labels = [item.display_name for item in metrics]
    colors = [item.color for item in metrics]
    means = [mean(item.solve_time_avg_values) or 0.0 for item in metrics]
    stds = [stddev(item.solve_time_avg_values) or 0.0 for item in metrics]
    x = range(len(metrics))
    error_kw = {"elinewidth": 1.4, "capsize": 6, "capthick": 1.4, "ecolor": "#333333"}

    bars = ax.bar(
        x,
        means,
        yerr=stds,
        color=colors,
        error_kw=error_kw,
        width=0.62,
    )
    ax.set_xticks(list(x), labels)
    ax.set_ylabel("MPC Solve Time (ms)")
    ax.set_title("Realtime MPC Solve Time")
    apply_common_axis_style(ax)

    budget_ms = 10.0
    ax.axhline(
        budget_ms,
        color=REALTIME_BUDGET_COLOR,
        linestyle="--",
        linewidth=1.8,
        label="Real-time Budget (10 ms)",
    )
    set_bar_axis_limits(ax, means + [budget_ms], stds + [0.0])
    add_bar_labels(ax, list(bars), means)

    legend_handles = [
        Patch(facecolor=item.color, edgecolor="none", label=item.display_name)
        for item in metrics
    ]
    legend_handles.append(
        Line2D(
            [0],
            [0],
            color=REALTIME_BUDGET_COLOR,
            linestyle="--",
            linewidth=1.8,
            label="Real-time Budget (10 ms)",
        )
    )
    ax.legend(handles=legend_handles, loc="upper right")

    stem = output_dir / "figure3_mpc_solve_time_with_realtime_budget"
    return save_figure(fig, stem)


def save_figure(fig: plt.Figure, stem: Path) -> list[str]:
    stem.parent.mkdir(parents=True, exist_ok=True)
    outputs: list[str] = []
    for suffix in (".pdf", ".svg"):
        output_path = stem.with_suffix(suffix)
        fig.savefig(output_path, dpi=DEFAULT_DPI, bbox_inches="tight")
        outputs.append(str(output_path))
    plt.close(fig)
    return outputs


def build_plot_manifest(
    *,
    analysis_dir: Path,
    output_dir: Path,
    generated_files: list[str],
    metrics: list[GroupMetrics],
) -> dict[str, Any]:
    groups: list[dict[str, Any]] = []
    for item in metrics:
        groups.append(
            {
                "group": item.group,
                "display_name": item.display_name,
                "isaac_forward_mean_m": item.isaac_forward_mean_m,
                "isaac_forward_std_m": item.isaac_forward_std_m,
                "tracking_rmse_mean_m": item.tracking_rmse_mean_m,
                "tracking_rmse_std_m": item.tracking_rmse_std_m,
                "guard_sample_count": len(item.guard_values),
                "max_still_sample_count": len(item.max_still_values),
                "solve_time_sample_count": len(item.solve_time_avg_values),
            }
        )

    return {
        "analysis_dir": str(analysis_dir),
        "output_dir": str(output_dir),
        "generated_files": generated_files,
        "figure_count": len(generated_files) // 2,
        "groups": groups,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate paper-grade Hexbot result figures from analysis CSV outputs."
    )
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="Path to a batch root or directly to an analysis directory.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional directory for figure outputs. Defaults to <analysis>/figures.",
    )
    parser.add_argument(
        "--fontsize",
        type=float,
        default=12.0,
        help="Base font size for paper figures.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    configure_plot_style(fontsize=args.fontsize)

    analysis_dir = resolve_analysis_dir(args.input)
    output_dir = args.output_dir.resolve() if args.output_dir else (analysis_dir / "figures")

    per_run_rows = load_csv_rows(analysis_dir / "hexbot_metrics_per_run.csv")
    if not per_run_rows:
        raise RuntimeError(f"No rows found in {(analysis_dir / 'hexbot_metrics_per_run.csv')}")

    metrics = collect_group_metrics(per_run_rows)
    if len(metrics) < 2:
        raise RuntimeError("Expected at least two groups to compare in the plotting stage.")

    generated_files: list[str] = []
    generated_files.extend(render_figure1(metrics, output_dir))
    generated_files.extend(render_figure2(metrics, output_dir))
    generated_files.extend(render_figure3(metrics, output_dir))

    manifest = build_plot_manifest(
        analysis_dir=analysis_dir,
        output_dir=output_dir,
        generated_files=generated_files,
        metrics=metrics,
    )
    manifest_path = output_dir / "plot_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print(
        json.dumps(
            {
                "analysis_dir": str(analysis_dir),
                "output_dir": str(output_dir),
                "generated_files": generated_files,
                "plot_manifest": str(manifest_path),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
