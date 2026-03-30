#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import numpy as np


ROOT = Path("/home/u-zhuang/ros2_ws/src/hexbot_paper_icra_v1")
FIG_DIR = ROOT / "figures"
ENDURANCE_POSTURE = Path(
    "/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/idle_reference_probe/"
    "endurance_final_60s_v1_20260330_142414_posture.json"
)


COLORS = {
    "blue": "#2563eb",
    "blue_fill": "#dbeafe",
    "amber": "#f59e0b",
    "amber_fill": "#fef3c7",
    "red": "#dc2626",
    "red_fill": "#fee2e2",
    "green": "#16a34a",
    "green_fill": "#dcfce7",
    "slate": "#475569",
    "slate_fill": "#e2e8f0",
    "purple": "#7c3aed",
    "purple_fill": "#ede9fe",
}


def _setup_matplotlib() -> None:
    plt.rcParams.update(
        {
            "font.size": 12,
            "axes.titlesize": 14,
            "axes.labelsize": 12,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "legend.fontsize": 10,
            "figure.titlesize": 15,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def _draw_box(
    ax,
    x,
    y,
    w,
    h,
    title,
    subtitle=None,
    fc="#ffffff",
    ec="#000000",
    lw=1.8,
    title_size=11,
    subtitle_size=9.5,
):
    box = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.012,rounding_size=0.02",
        linewidth=lw,
        edgecolor=ec,
        facecolor=fc,
    )
    ax.add_patch(box)
    ax.text(
        x + w / 2,
        y + h * 0.64,
        title,
        ha="center",
        va="center",
        color=ec,
        fontweight="bold",
        fontsize=title_size,
    )
    if subtitle:
        ax.text(
            x + w / 2,
            y + h * 0.28,
            subtitle,
            ha="center",
            va="center",
            color=COLORS["slate"],
            fontsize=subtitle_size,
        )
    return box


def _arrow(ax, x1, y1, x2, y2, color, lw=2.2, style="-|>", ls="-", mutation_scale=14, connectionstyle="arc3"):
    arr = FancyArrowPatch(
        (x1, y1),
        (x2, y2),
        arrowstyle=style,
        mutation_scale=mutation_scale,
        linewidth=lw,
        linestyle=ls,
        color=color,
        connectionstyle=connectionstyle,
    )
    ax.add_patch(arr)
    return arr


def generate_methodology_overview() -> None:
    dot = f"""
digraph MethodologyOverview {{
    graph [
        rankdir=LR,
        compound=true,
        newrank=true,
        splines=spline,
        overlap=false,
        nodesep="1.0",
        ranksep="1.5",
        pad="0.25",
        margin="0.05",
        bgcolor="white",
        fontname="Helvetica"
    ];
    node [
        shape=box,
        style="rounded,filled",
        fixedsize=false,
        fontname="Helvetica",
        fontsize=11,
        penwidth=1.8,
        margin="0.12,0.08",
        color="{COLORS['slate']}",
        fillcolor="white"
    ];
    edge [
        fontname="Helvetica",
        fontsize=10,
        color="{COLORS['slate']}",
        penwidth=2.0,
        arrowsize=0.8
    ];

    subgraph cluster_pipeline {{
        label="Pipeline";
        color="{COLORS['blue']}";
        fillcolor="{COLORS['blue_fill']}";
        fontcolor="{COLORS['blue']}";
        style="rounded,filled";
        nodesep="0.8";
        ranksep="0.95";
        margin="18";

        cmd_vel [label="cmd_vel\\nvelocity command", color="{COLORS['blue']}", fillcolor="white", width="1.6", height="0.8"];
        target_gen [label="Target Generator\\ngait + moving goal reference", color="{COLORS['amber']}", fillcolor="white", width="2.0", height="0.9"];
        ocs2_solver [label="OCS2 Solver\\noptimal control solve", color="{COLORS['amber']}", fillcolor="white", width="1.8", height="0.9"];
        mrt [label="MRT\\npolicy rollout and dispatch", color="{COLORS['slate']}", fillcolor="white", width="1.8", height="0.9"];
        isaac [label="Isaac Sim\\nstate feedback and execution", color="{COLORS['green']}", fillcolor="white", width="2.0", height="0.9"];
        pipeline_note [label="Nominal closed-loop flow\\nROS 2 target generation, OCS2 policy update,\\nMRT rollout, and Isaac Sim feedback remain intact.", color="{COLORS['blue']}", fillcolor="white", width="4.1", height="1.0"];

        {{ rank=same; cmd_vel; target_gen; ocs2_solver; }}
        {{ rank=same; mrt; isaac; }}

        cmd_vel -> target_gen;
        target_gen -> ocs2_solver;
        ocs2_solver -> mrt;
        mrt -> isaac;
        isaac -> pipeline_note [style=invis, weight=4];
        isaac -> cmd_vel [color="{COLORS['green']}", xlabel="state feedback", minlen="2"];
    }}

    subgraph cluster_sqp {{
        label="SQP Failure Path";
        color="{COLORS['red']}";
        fillcolor="{COLORS['red_fill']}";
        fontcolor="{COLORS['red']}";
        style="rounded,filled";
        nodesep="1.0";
        ranksep="1.1";
        margin="18";

        sqp_solver [label="SQP Solver\\nlocal linearization follows warm start", color="{COLORS['red']}", fillcolor="white", width="2.2", height="0.95"];
        branch_constraint [label="Branch Constraint\\npresent in problem definition", color="{COLORS['amber']}", fillcolor="white", width="1.9", height="0.95"];
        pathological [label="Pathological Commands\\nwrong branch remains numerically executable", color="{COLORS['red']}", fillcolor="white", width="2.6", height="0.95"];
        runtime_guard [label="Runtime Guard\\npost hoc interception", color="{COLORS['red']}", fillcolor="white", width="1.9", height="0.95"];
        sqp_note [label="Forbidden branch stays solver-feasible.\\nThe runtime layer must absorb branch errors because\\nSQP does not reliably remove the pathological solution path.", color="{COLORS['red']}", fillcolor="white", width="4.5", height="1.05"];

        {{ rank=same; sqp_solver; branch_constraint; }}
        {{ rank=same; pathological; runtime_guard; }}

        sqp_solver -> branch_constraint [
            style=dashed,
            color="{COLORS['red']}",
            penwidth=2.3,
            xlabel="constraint bypassed\\nin active SQP path",
            minlen="2"
        ];
        sqp_solver -> pathological [color="{COLORS['red']}", penwidth=2.2, minlen="2"];
        pathological -> runtime_guard [color="{COLORS['red']}", penwidth=2.2, minlen="2"];
        runtime_guard -> sqp_note [style=invis, weight=4];
    }}

    subgraph cluster_ipm {{
        label="IPM Fix Path";
        color="{COLORS['green']}";
        fillcolor="{COLORS['green_fill']}";
        fontcolor="{COLORS['green']}";
        style="rounded,filled";
        nodesep="1.0";
        ranksep="1.1";
        margin="18";

        ipm_solver [label="IPM Solver\\nprimal-dual inequality handling", color="{COLORS['green']}", fillcolor="white", width="2.1", height="0.95"];
        safe_commands [label="Safe Commands\\nwalking policy reaches Isaac Sim directly", color="{COLORS['green']}", fillcolor="white", width="2.3", height="0.95"];
        ipm_note [label="Forbidden branch is excluded inside the solver.\\nIPM keeps the constraint active before command publication,\\nso runtime guarding is no longer the primary repair mechanism.", color="{COLORS['green']}", fillcolor="white", width="4.6", height="1.05"];

        subgraph cluster_feasible {{
            label="Feasible Search Set\\nshaped by TarsusBranchConstraint";
            color="{COLORS['green']}";
            fontcolor="{COLORS['green']}";
            style="rounded";
            nodesep="0.8";
            ranksep="0.8";
            margin="16";

            allowed_node [label=<
                <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="8" COLOR="{COLORS['green']}">
                    <TR><TD BGCOLOR="{COLORS['green_fill']}"><B>allowed</B><BR/>branch</TD></TR>
                </TABLE>
            >, shape=plain];

            forbidden_node [label=<
                <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="8" COLOR="{COLORS['red']}">
                    <TR><TD BGCOLOR="white"><B>forbidden</B><BR/>branch<BR/><FONT COLOR="{COLORS['red']}"><B>X</B></FONT></TD></TR>
                </TABLE>
            >, shape=plain];

            {{ rank=same; allowed_node; forbidden_node; }}
            allowed_node -> forbidden_node [style=invis, minlen="1"];
        }}

        ipm_solver -> allowed_node [color="{COLORS['green']}", penwidth=2.2, minlen="2"];
        allowed_node -> safe_commands [color="{COLORS['green']}", penwidth=2.2, minlen="2"];
        safe_commands -> ipm_note [style=invis, weight=4];
    }}

    isaac -> sqp_solver [style=invis, weight=20];
    runtime_guard -> ipm_solver [style=invis, weight=20];
}}
"""

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    dot_path = FIG_DIR / "methodology_overview.dot"
    dot_path.write_text(dot, encoding="utf-8")

    for ext in ["svg", "pdf"]:
        subprocess.run(
            ["dot", f"-T{ext}", str(dot_path), "-o", str(FIG_DIR / f"methodology_overview.{ext}")],
            check=True,
        )


def _load_endurance_trace():
    with ENDURANCE_POSTURE.open() as f:
        posture = json.load(f)
    samples = posture["odom_trace"]["samples"]
    t = np.array([s["elapsed_sec"] for s in samples], dtype=float)
    x = np.array([s["x_m"] for s in samples], dtype=float)
    y = np.array([s["y_m"] for s in samples], dtype=float)
    z = np.array([s["z_m"] for s in samples], dtype=float)
    yaw = np.array([s.get("yaw_unwrapped_rad", s["yaw_rad"]) for s in samples], dtype=float)

    x0, y0, yaw0, z0 = x[0], y[0], yaw[0], z[0]
    forward = np.cos(yaw0) * (x - x0) + np.sin(yaw0) * (y - y0)
    yaw_deg = np.degrees(yaw - yaw0)
    z_rel = z - z0

    metrics = posture["metrics"]
    return t, forward, yaw_deg, z, z_rel, metrics


def generate_endurance_timeseries() -> None:
    t, forward, yaw_deg, z, z_rel, metrics = _load_endurance_trace()
    fig, axes = plt.subplots(3, 1, figsize=(11.2, 8.6), sharex=True, constrained_layout=True)

    # Forward
    axes[0].plot(t, forward, color=COLORS["blue"], lw=2.4, label="Forward displacement")
    axes[0].fill_between(t, 0.0, forward, color=COLORS["blue_fill"], alpha=0.35)
    axes[0].set_ylabel("Forward\nDisplacement (m)")
    axes[0].set_title("Captured closed-loop trajectory from the final 60-second endurance evaluation", loc="left")
    axes[0].grid(True, alpha=0.28)
    axes[0].legend(loc="upper left", frameon=False)
    axes[0].text(
        0.985,
        0.08,
        f"final = {forward[-1]:.3f} m",
        transform=axes[0].transAxes,
        ha="right",
        va="bottom",
        fontsize=10,
        color=COLORS["blue"],
        fontweight="bold",
    )

    # Yaw
    axes[1].axhspan(-10.0, 10.0, color=COLORS["green_fill"], alpha=0.55, zorder=0)
    axes[1].plot(t, yaw_deg, color=COLORS["amber"], lw=2.1, label="Yaw drift")
    axes[1].axhline(0.0, color=COLORS["slate"], lw=1.0, ls="--")
    axes[1].set_ylabel("Yaw Drift (deg)")
    axes[1].grid(True, alpha=0.28)
    axes[1].legend(loc="upper left", frameon=False)
    axes[1].text(
        0.985,
        0.08,
        f"final = {yaw_deg[-1]:.2f} deg",
        transform=axes[1].transAxes,
        ha="right",
        va="bottom",
        fontsize=10,
        color=COLORS["amber"],
        fontweight="bold",
    )

    # Z height
    z_mean = np.mean(z)
    axes[2].plot(t, z, color=COLORS["purple"], lw=2.1, label="Base height")
    axes[2].axhline(z_mean, color=COLORS["slate"], lw=1.0, ls="--", label="Mean height")
    axes[2].fill_between(t, z_mean - metrics["z_height_range_m"] / 2.0, z_mean + metrics["z_height_range_m"] / 2.0, color=COLORS["purple_fill"], alpha=0.45)
    axes[2].set_ylabel("Base Height (m)")
    axes[2].set_xlabel("Captured Elapsed Time (s)")
    axes[2].grid(True, alpha=0.28)
    axes[2].legend(loc="upper left", frameon=False)
    axes[2].text(
        0.985,
        0.08,
        f"range = {metrics['z_height_range_m']:.3f} m",
        transform=axes[2].transAxes,
        ha="right",
        va="bottom",
        fontsize=10,
        color=COLORS["purple"],
        fontweight="bold",
    )

    axes[2].set_xlim(t[0], t[-1])
    for ax in axes:
        ax.spines["left"].set_linewidth(1.0)
        ax.spines["bottom"].set_linewidth(1.0)

    for ext in ["pdf", "svg"]:
        fig.savefig(FIG_DIR / f"endurance_timeseries.{ext}", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    _setup_matplotlib()
    generate_methodology_overview()
    generate_endurance_timeseries()


if __name__ == "__main__":
    main()
