#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from statistics import mean
from typing import Any


LEG_NAMES = {
    0: "RR",
    1: "RM",
    2: "RF",
    3: "LR",
    4: "LM",
    5: "LF",
}

HEADER_RE = re.compile(
    r"LeggedRobotPreComputation\] t=(?P<time>-?\d+(?:\.\d+)?) "
    r"support_normal=\[(?P<nx>-?\d+(?:\.\d+)?), (?P<ny>-?\d+(?:\.\d+)?), (?P<nz>-?\d+(?:\.\d+)?)\] "
    r"support_height=(?P<height>-?\d+(?:\.\d+)?)"
)

FOOT_RE = re.compile(
    r"foot\[(?P<index>\d+)\] contact=(?P<contact>[01]) "
    r"pos=\[(?P<px>-?\d+(?:\.\d+)?), (?P<py>-?\d+(?:\.\d+)?), (?P<pz>-?\d+(?:\.\d+)?)\] "
    r"normal_dist=(?P<normal_dist>-?\d+(?:\.\d+)?) "
    r"ref_dist=(?P<ref_dist>-?\d+(?:\.\d+)?) "
    r"ref_vel=(?P<ref_vel>-?\d+(?:\.\d+)?) "
    r"normal_force=(?P<normal_force>-?\d+(?:\.\d+)?)"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize Hexbot LeggedRobotPreComputation blocks from an OCS2 container log."
    )
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--swing-clearance-tol", type=float, default=0.005)
    parser.add_argument("--swing-below-plane-tol", type=float, default=0.002)
    parser.add_argument("--force-imbalance-ratio", type=float, default=1.8)
    return parser.parse_args()


def parse_blocks(text: str) -> list[dict[str, Any]]:
    lines = text.splitlines()
    blocks: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None

    for line in lines:
        header_match = HEADER_RE.search(line)
        if header_match is not None:
            if current is not None:
                blocks.append(current)
            current = {
                "time": float(header_match.group("time")),
                "support_normal": [
                    float(header_match.group("nx")),
                    float(header_match.group("ny")),
                    float(header_match.group("nz")),
                ],
                "support_height": float(header_match.group("height")),
                "feet": {},
                "raw_header": line,
            }
            continue

        if current is None:
            continue

        foot_match = FOOT_RE.search(line)
        if foot_match is None:
            continue

        foot_index = int(foot_match.group("index"))
        current["feet"][foot_index] = {
            "leg": LEG_NAMES.get(foot_index, str(foot_index)),
            "contact": int(foot_match.group("contact")),
            "pos": [
                float(foot_match.group("px")),
                float(foot_match.group("py")),
                float(foot_match.group("pz")),
            ],
            "normal_dist": float(foot_match.group("normal_dist")),
            "ref_dist": float(foot_match.group("ref_dist")),
            "ref_vel": float(foot_match.group("ref_vel")),
            "normal_force": float(foot_match.group("normal_force")),
        }

    if current is not None:
        blocks.append(current)

    for block in blocks:
        block["clean"] = len(block["feet"]) == len(LEG_NAMES)

    return blocks


def safe_mean(values: list[float]) -> float | None:
    if not values:
        return None
    return float(mean(values))


def summarize_blocks(
    blocks: list[dict[str, Any]],
    swing_clearance_tol: float,
    swing_below_plane_tol: float,
    force_imbalance_ratio: float,
) -> dict[str, Any]:
    clean_blocks = [block for block in blocks if block.get("clean")]
    per_leg: dict[str, dict[str, Any]] = {
        LEG_NAMES[i]: {
            "swing_samples": 0,
            "stance_samples": 0,
            "swing_clearance_gap_values": [],
            "swing_normal_dist_values": [],
            "stance_normal_force_values": [],
            "stance_normal_dist_values": [],
            "swing_below_reference_count": 0,
            "swing_below_plane_count": 0,
        }
        for i in sorted(LEG_NAMES)
    }
    problematic_blocks: list[dict[str, Any]] = []

    for block in clean_blocks:
        stance_forces = []
        swing_issues = []
        for foot_index, foot in sorted(block["feet"].items()):
            leg_name = LEG_NAMES[foot_index]
            leg_stats = per_leg[leg_name]
            if foot["contact"] == 0:
                leg_stats["swing_samples"] += 1
                clearance_gap = float(foot["normal_dist"] - foot["ref_dist"])
                leg_stats["swing_clearance_gap_values"].append(clearance_gap)
                leg_stats["swing_normal_dist_values"].append(float(foot["normal_dist"]))
                if clearance_gap < -abs(swing_clearance_tol):
                    leg_stats["swing_below_reference_count"] += 1
                    swing_issues.append(
                        {
                            "leg": leg_name,
                            "normal_dist": foot["normal_dist"],
                            "ref_dist": foot["ref_dist"],
                            "clearance_gap": clearance_gap,
                        }
                    )
                if foot["normal_dist"] < -abs(swing_below_plane_tol):
                    leg_stats["swing_below_plane_count"] += 1
            else:
                leg_stats["stance_samples"] += 1
                leg_stats["stance_normal_force_values"].append(float(foot["normal_force"]))
                leg_stats["stance_normal_dist_values"].append(float(foot["normal_dist"]))
                stance_forces.append((leg_name, float(foot["normal_force"])))

        stance_force_ratio = None
        if len(stance_forces) >= 2:
            force_values = [value for _, value in stance_forces if value > 1.0e-6]
            if len(force_values) >= 2:
                stance_force_ratio = max(force_values) / max(min(force_values), 1.0e-6)

        if swing_issues or (
            stance_force_ratio is not None and stance_force_ratio > force_imbalance_ratio
        ):
            problematic_blocks.append(
                {
                    "time": block["time"],
                    "support_normal": block["support_normal"],
                    "support_height": block["support_height"],
                    "stance_force_ratio": stance_force_ratio,
                    "stance_forces": stance_forces,
                    "swing_issues": swing_issues,
                }
            )

    per_leg_summary: dict[str, Any] = {}
    for leg_name, leg_stats in per_leg.items():
        swing_gaps = leg_stats.pop("swing_clearance_gap_values")
        swing_dists = leg_stats.pop("swing_normal_dist_values")
        stance_forces = leg_stats.pop("stance_normal_force_values")
        stance_dists = leg_stats.pop("stance_normal_dist_values")
        per_leg_summary[leg_name] = {
            **leg_stats,
            "swing_clearance_gap_mean_m": safe_mean(swing_gaps),
            "swing_clearance_gap_min_m": min(swing_gaps) if swing_gaps else None,
            "swing_clearance_gap_max_m": max(swing_gaps) if swing_gaps else None,
            "swing_normal_dist_mean_m": safe_mean(swing_dists),
            "swing_normal_dist_min_m": min(swing_dists) if swing_dists else None,
            "swing_normal_dist_max_m": max(swing_dists) if swing_dists else None,
            "stance_normal_force_mean": safe_mean(stance_forces),
            "stance_normal_force_min": min(stance_forces) if stance_forces else None,
            "stance_normal_force_max": max(stance_forces) if stance_forces else None,
            "stance_normal_dist_mean_m": safe_mean(stance_dists),
            "stance_normal_dist_min_m": min(stance_dists) if stance_dists else None,
            "stance_normal_dist_max_m": max(stance_dists) if stance_dists else None,
        }

    global_metrics = {
        "total_blocks": len(blocks),
        "clean_blocks": len(clean_blocks),
        "first_clean_time": clean_blocks[0]["time"] if clean_blocks else None,
        "last_clean_time": clean_blocks[-1]["time"] if clean_blocks else None,
        "problematic_block_count": len(problematic_blocks),
        "max_stance_force_ratio": (
            max(
                (
                    block["stance_force_ratio"]
                    for block in problematic_blocks
                    if block["stance_force_ratio"] is not None
                ),
                default=None,
            )
        ),
    }

    return {
        "metrics": global_metrics,
        "per_leg": per_leg_summary,
        "problematic_blocks": problematic_blocks[:12],
    }


def main() -> int:
    args = parse_args()
    text = args.log.read_text(encoding="utf-8", errors="replace")
    blocks = parse_blocks(text)
    payload = {
        "log_file": str(args.log.resolve()),
        "thresholds": {
            "swing_clearance_tolerance_m": float(args.swing_clearance_tol),
            "swing_below_plane_tolerance_m": float(args.swing_below_plane_tol),
            "stance_force_imbalance_ratio": float(args.force_imbalance_ratio),
        },
        **summarize_blocks(
            blocks,
            swing_clearance_tol=float(args.swing_clearance_tol),
            swing_below_plane_tol=float(args.swing_below_plane_tol),
            force_imbalance_ratio=float(args.force_imbalance_ratio),
        ),
    }
    text_out = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text_out + "\n", encoding="utf-8")
    print(text_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
