#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_SRC_DIR = SCRIPT_DIR.parent / "src"
if str(PKG_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(PKG_SRC_DIR))

from hexapod_geometry import HexapodGeometry, load_yaml  # noqa: E402
from leg_ik import LegIKSolver  # noqa: E402


def copy_vector_map(vector_map: dict[str, list[float]]) -> dict[str, list[float]]:
    return {key: [float(value) for value in values] for key, values in vector_map.items()}


def parse_stance_position_entry(raw_value: object) -> tuple[float, float, float | None]:
    if isinstance(raw_value, dict):
        if "x" not in raw_value or "y" not in raw_value:
            raise ValueError(
                "Each stance.per_leg_stance_positions entry must provide x and y."
            )
        z_value = raw_value.get("z")
        return (
            float(raw_value["x"]),
            float(raw_value["y"]),
            None if z_value is None else float(z_value),
        )

    if not isinstance(raw_value, (list, tuple)) or len(raw_value) not in (2, 3):
        raise ValueError(
            "Each stance.per_leg_stance_positions entry must contain [x, y] or [x, y, z]."
        )

    z_override = None if len(raw_value) == 2 else float(raw_value[2])
    return float(raw_value[0]), float(raw_value[1]), z_override


def resolve_stand_foot_targets(
    geometry: HexapodGeometry, raw_config: dict[str, object]
) -> dict[str, list[float]]:
    implicit_targets = geometry.neutral_foot_positions()
    stance_cfg = raw_config.get("stance")
    if not isinstance(stance_cfg, dict) or not stance_cfg:
        return implicit_targets

    targets = copy_vector_map(implicit_targets)

    standing_body_height = stance_cfg.get("standing_body_height_m")
    if standing_body_height is None:
        standing_body_height = stance_cfg.get("standing_body_height")
    if standing_body_height is None:
        standing_body_height = stance_cfg.get("body_clearance_m")
    if standing_body_height is None:
        standing_body_height = stance_cfg.get("body_clearance")

    stance_foot_z_offset = stance_cfg.get("stance_foot_z_offset_m")
    if stance_foot_z_offset is None:
        stance_foot_z_offset = stance_cfg.get("stance_foot_z_offset")
    if stance_foot_z_offset is None:
        stance_foot_z_offset = stance_cfg.get("foot_z_offset_m")

    per_leg_positions = stance_cfg.get("per_leg_stance_positions")
    if per_leg_positions is None:
        per_leg_positions = stance_cfg.get("stance_positions")

    if standing_body_height is not None:
        stand_z = -float(standing_body_height)
        if stance_foot_z_offset is not None:
            stand_z -= float(stance_foot_z_offset)
        for leg_name in geometry.leg_order:
            targets[leg_name][2] = stand_z
    elif stance_foot_z_offset is not None:
        raise ValueError(
            "stance.stance_foot_z_offset_m requires stance.standing_body_height_m."
        )

    if per_leg_positions is None:
        return targets
    if not isinstance(per_leg_positions, dict):
        raise ValueError("stance.per_leg_stance_positions must be a leg->position map.")

    missing_legs = [
        leg_name for leg_name in geometry.leg_order if leg_name not in per_leg_positions
    ]
    extra_legs = sorted(
        leg_name
        for leg_name in per_leg_positions.keys()
        if leg_name not in geometry.leg_order
    )
    if missing_legs or extra_legs:
        raise ValueError(
            "stance.per_leg_stance_positions must define exactly the configured leg set. "
            f"missing={missing_legs}, extra={extra_legs}"
        )

    for leg_name in geometry.leg_order:
        x_pos, y_pos, z_override = parse_stance_position_entry(
            per_leg_positions[leg_name]
        )
        targets[leg_name][0] = x_pos
        targets[leg_name][1] = y_pos
        if z_override is not None:
            targets[leg_name][2] = z_override

    return targets


def resolve_target_leg_angles(
    joint_config_path: Path, geometry_config_path: Path
) -> tuple[HexapodGeometry, dict[str, list[float]]]:
    raw_config = load_yaml(geometry_config_path)
    geometry = HexapodGeometry.from_config_files(joint_config_path, geometry_config_path)
    stance_targets = resolve_stand_foot_targets(geometry, raw_config)
    if raw_config.get("stance"):
        ik_solver = LegIKSolver.from_config(geometry.ik_config)
        solved_leg_angles: dict[str, list[float]] = {}
        max_error_norm = max(ik_solver.tolerance * 10.0, 1.0e-3)
        for leg_name in geometry.leg_order:
            solution = ik_solver.solve(
                geometry,
                leg_name,
                stance_targets[leg_name],
                initial_guess=geometry.neutral_joint_positions[leg_name],
            )
            if not solution.success and solution.error_norm > max_error_norm:
                raise ValueError(
                    "Configured stand target is unreachable for "
                    f"{leg_name}: residual={solution.error_norm:.6f}, "
                    f"target={stance_targets[leg_name]}"
                )
            solved_leg_angles[leg_name] = [
                float(value) for value in solution.joint_angles
            ]
        return geometry, solved_leg_angles
    return geometry, copy_vector_map(geometry.neutral_joint_positions)


class JointStateConvergenceWaiter(Node):
    def __init__(
        self,
        target_joint_positions: dict[str, float],
        joint_state_topic: str,
        use_sim_time: bool,
        max_joint_diff_rad: float,
        stable_samples_required: int,
    ) -> None:
        super().__init__("hexbot_joint_state_convergence_waiter")
        if use_sim_time:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.target_joint_positions = {
            str(joint_name): float(value)
            for joint_name, value in target_joint_positions.items()
        }
        self.joint_state_topic = str(joint_state_topic)
        self.max_joint_diff_rad = float(max_joint_diff_rad)
        self.stable_samples_required = max(1, int(stable_samples_required))

        self.sample_count = 0
        self.current_max_joint_diff_rad: float | None = None
        self.best_max_joint_diff_rad = float("inf")
        self.stable_sample_count = 0
        self.ready = False

        self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_callback, 20
        )

    def _joint_state_callback(self, message: JointState) -> None:
        if not message.position:
            return

        positions_by_name: dict[str, float] = {}
        for index, joint_name in enumerate(message.name):
            if index >= len(message.position):
                break
            if joint_name not in self.target_joint_positions:
                continue
            positions_by_name[joint_name] = float(message.position[index])

        if len(positions_by_name) != len(self.target_joint_positions):
            return

        self.sample_count += 1
        max_joint_diff_rad = max(
            abs(positions_by_name[joint_name] - target_position)
            for joint_name, target_position in self.target_joint_positions.items()
        )
        self.current_max_joint_diff_rad = max_joint_diff_rad
        self.best_max_joint_diff_rad = min(self.best_max_joint_diff_rad, max_joint_diff_rad)
        if max_joint_diff_rad <= self.max_joint_diff_rad:
            self.stable_sample_count += 1
            if self.stable_sample_count >= self.stable_samples_required:
                self.ready = True
        else:
            self.stable_sample_count = 0

    def status_payload(self, elapsed_sec: float) -> dict[str, float | int | bool | None]:
        return {
            "joint_state_topic": self.joint_state_topic,
            "sample_count": self.sample_count,
            "ready": self.ready,
            "max_joint_diff_threshold_rad": self.max_joint_diff_rad,
            "current_max_joint_diff_rad": self.current_max_joint_diff_rad,
            "best_max_joint_diff_rad": None
            if self.best_max_joint_diff_rad == float("inf")
            else self.best_max_joint_diff_rad,
            "stable_samples_required": self.stable_samples_required,
            "stable_sample_count": self.stable_sample_count,
            "elapsed_sec": elapsed_sec,
        }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Wait until /joint_states converges close enough to the configured stand "
            "joint map for the given geometry config."
        )
    )
    parser.add_argument("--joint-config", type=Path, required=True)
    parser.add_argument("--geometry-config", type=Path, required=True)
    parser.add_argument("--joint-state-topic", type=str, default="/joint_states")
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--max-joint-diff-rad", type=float, default=0.10)
    parser.add_argument("--stable-samples", type=int, default=5)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    args = parser.parse_args()

    geometry, target_leg_angles = resolve_target_leg_angles(
        args.joint_config, args.geometry_config
    )
    target_joint_positions = dict(
        zip(
            geometry.joint_names,
            geometry.joint_positions_from_leg_map(target_leg_angles),
            strict=True,
        )
    )

    rclpy.init()
    node = JointStateConvergenceWaiter(
        target_joint_positions=target_joint_positions,
        joint_state_topic=args.joint_state_topic,
        use_sim_time=bool(args.use_sim_time),
        max_joint_diff_rad=float(args.max_joint_diff_rad),
        stable_samples_required=int(args.stable_samples),
    )
    start_wall_sec = time.monotonic()
    exit_code = 0
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (time.monotonic() - start_wall_sec) >= float(args.timeout_sec):
                exit_code = 1
                break
    finally:
        elapsed_sec = time.monotonic() - start_wall_sec
        payload = node.status_payload(elapsed_sec)
        print(json.dumps(payload, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
