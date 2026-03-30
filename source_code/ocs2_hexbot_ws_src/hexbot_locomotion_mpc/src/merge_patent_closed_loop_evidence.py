#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_yaml_doc(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return None
    lines = []
    started = False
    for line in text.splitlines():
        if not started:
            if line.startswith(("header:", "time_trajectory:", "event_times:")):
                started = True
                lines.append(line)
            continue
        if line.startswith("A message was lost!!!") or line.startswith("\ttotal count"):
            continue
        lines.append(line)
    cleaned_text = "\n".join(lines).strip()
    if not cleaned_text:
        return None
    docs = [doc for doc in yaml.safe_load_all(cleaned_text) if doc is not None]
    return docs[0] if docs else None


def to_float(value: Any) -> float | None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None


def unwrap_delta(delta: float) -> float:
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def extract_target_snapshot(
    document: dict[str, Any] | None, base_pose_offset: int
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "available": False,
        "base_pose_offset": int(base_pose_offset),
    }
    if not document:
        return payload

    time_trajectory = document.get("time_trajectory") or []
    state_trajectory = document.get("state_trajectory") or []
    if len(time_trajectory) < 2 or len(state_trajectory) < 2:
        return payload

    start_state = (state_trajectory[0] or {}).get("value") or []
    goal_state = (state_trajectory[1] or {}).get("value") or []
    if len(start_state) < base_pose_offset + 6 or len(goal_state) < base_pose_offset + 6:
        return payload

    def pose_from(values: list[Any]) -> dict[str, float]:
        return {
            "x": float(values[base_pose_offset + 0]),
            "y": float(values[base_pose_offset + 1]),
            "z": float(values[base_pose_offset + 2]),
            "yaw": float(values[base_pose_offset + 3]),
            "pitch": float(values[base_pose_offset + 4]),
            "roll": float(values[base_pose_offset + 5]),
        }

    start_pose = pose_from(start_state)
    goal_pose = pose_from(goal_state)
    delta_x = goal_pose["x"] - start_pose["x"]
    delta_y = goal_pose["y"] - start_pose["y"]
    delta_yaw = unwrap_delta(goal_pose["yaw"] - start_pose["yaw"])
    cos_yaw = math.cos(start_pose["yaw"])
    sin_yaw = math.sin(start_pose["yaw"])
    delta_forward = cos_yaw * delta_x + sin_yaw * delta_y
    delta_lateral = -sin_yaw * delta_x + cos_yaw * delta_y
    horizon_sec = float(time_trajectory[1]) - float(time_trajectory[0])

    payload.update(
        {
            "available": True,
            "time_trajectory": [float(time_trajectory[0]), float(time_trajectory[1])],
            "horizon_sec": horizon_sec,
            "start_pose": start_pose,
            "goal_pose": goal_pose,
            "delta_world": {
                "x": delta_x,
                "y": delta_y,
                "yaw": delta_yaw,
            },
            "delta_body": {
                "forward_m": delta_forward,
                "lateral_m": delta_lateral,
            },
        }
    )
    return payload


def extract_mode_schedule(document: dict[str, Any] | None) -> dict[str, Any]:
    payload: dict[str, Any] = {"available": False}
    if not document:
        return payload
    event_times = [float(value) for value in (document.get("event_times") or [])]
    mode_sequence = [int(value) for value in (document.get("mode_sequence") or [])]
    payload.update(
        {
            "available": True,
            "event_times": event_times,
            "mode_sequence": mode_sequence,
        }
    )
    return payload


def extract_solver_capture(document: dict[str, Any] | None) -> dict[str, Any]:
    payload: dict[str, Any] = {"available": False, "status": []}
    if not document:
        return payload

    status_payload = []
    max_level = 0
    for status in document.get("status") or []:
        level_raw = status.get("level", 0)
        if isinstance(level_raw, str) and level_raw:
            level = ord(level_raw[0])
        else:
            level = int(level_raw)
        max_level = max(max_level, level)
        values: dict[str, str] = {}
        for pair in status.get("values") or []:
            key = str(pair.get("key", ""))
            value = str(pair.get("value", ""))
            if key:
                values[key] = value
        status_payload.append(
            {
                "name": str(status.get("name", "")),
                "level": level,
                "message": str(status.get("message", "")),
                "hardware_id": str(status.get("hardware_id", "")),
                "values": values,
            }
        )

    payload.update(
        {
            "available": True,
            "header_stamp_sec": to_float((document.get("header") or {}).get("stamp", {}).get("sec")),
            "status": status_payload,
            "max_level": max_level,
        }
    )
    return payload


def build_summary(
    probe: dict[str, Any],
    target_snapshot: dict[str, Any],
    mode_schedule: dict[str, Any],
    solver_capture: dict[str, Any],
) -> dict[str, Any]:
    summary = {
        "probe": probe,
        "target_snapshot": target_snapshot,
        "mode_schedule_snapshot": mode_schedule,
        "solver_capture_snapshot": solver_capture,
    }

    actual_motion = probe.get("motion") or {}
    actual_forward = to_float(actual_motion.get("delta_forward_m"))
    actual_yaw = to_float(actual_motion.get("delta_yaw_rad"))
    target_body = target_snapshot.get("delta_body") or {}
    target_world = target_snapshot.get("delta_world") or {}
    target_forward = to_float(target_body.get("forward_m"))
    target_yaw = to_float(target_world.get("yaw"))

    comparison = {
        "available": (
            target_snapshot.get("available") is True
            and actual_forward is not None
            and actual_yaw is not None
            and target_forward is not None
            and target_yaw is not None
        )
    }
    if comparison["available"]:
        comparison.update(
            {
                "actual_forward_m": actual_forward,
                "actual_yaw_rad": actual_yaw,
                "target_snapshot_forward_m": target_forward,
                "target_snapshot_yaw_rad": target_yaw,
                "forward_gap_m": actual_forward - target_forward,
                "yaw_gap_rad": actual_yaw - target_yaw,
            }
        )
    summary["actual_vs_target_snapshot"] = comparison
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Merge host closed-loop probe output with container-side OCS2 topic captures."
    )
    parser.add_argument("--probe-json", type=Path, required=True)
    parser.add_argument("--target-yaml", type=Path, required=True)
    parser.add_argument("--mode-yaml", type=Path, required=True)
    parser.add_argument("--solver-yaml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--base-pose-offset", type=int, default=6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    probe = load_json(args.probe_json)
    target_snapshot = extract_target_snapshot(
        load_yaml_doc(args.target_yaml), args.base_pose_offset
    )
    mode_schedule = extract_mode_schedule(load_yaml_doc(args.mode_yaml))
    solver_capture = extract_solver_capture(load_yaml_doc(args.solver_yaml))
    merged = build_summary(probe, target_snapshot, mode_schedule, solver_capture)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(merged, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(merged, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
