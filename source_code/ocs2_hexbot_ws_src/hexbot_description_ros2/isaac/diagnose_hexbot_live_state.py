#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from isaac_vscode_exec import _execute_code


THIS_FILE = Path(__file__).resolve()
WORKSPACE_ROOT = THIS_FILE.parents[3]
DEFAULT_JOINT_CONFIG = (
    WORKSPACE_ROOT / "src" / "hexbot_description_ros2" / "config" / "joint_name_list.yaml"
)
DEFAULT_GEOMETRY_CONFIG = (
    WORKSPACE_ROOT
    / "src"
    / "hexbot_locomotion_v2"
    / "config"
    / "hexbot_v2_stage2_explicit_current_candidate.yaml"
)
LOCOMOTION_SRC = WORKSPACE_ROOT / "src" / "hexbot_locomotion_v2" / "src"
if str(LOCOMOTION_SRC) not in sys.path:
    sys.path.insert(0, str(LOCOMOTION_SRC))

from hexapod_geometry import HexapodGeometry, load_yaml  # noqa: E402
from leg_ik import LegIKSolver  # noqa: E402


class JointCapture(Node):
    def __init__(self, joint_state_topic: str, joint_command_topic: str) -> None:
        super().__init__("hexbot_live_state_diagnostic")
        self.joint_state_msg: JointState | None = None
        self.joint_command_msg: JointState | None = None
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)
        self.create_subscription(
            JointState, joint_command_topic, self._on_joint_command, 10
        )

    def _on_joint_state(self, message: JointState) -> None:
        self.joint_state_msg = message

    def _on_joint_command(self, message: JointState) -> None:
        self.joint_command_msg = message


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare expected stand joints, /joint_command, /joint_states, and "
            "Isaac articulation applied actions for the live hexbot scene."
        )
    )
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--joint-command-topic", default="/joint_command")
    parser.add_argument("--joint-config", type=Path, default=DEFAULT_JOINT_CONFIG)
    parser.add_argument(
        "--geometry-config", type=Path, default=DEFAULT_GEOMETRY_CONFIG
    )
    parser.add_argument("--articulation-path", default="/hexbot/hexbot")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8226)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--probe-timeout-sec",
        type=float,
        default=3.0,
        help="How long to wait for /joint_states and /joint_command samples.",
    )
    return parser.parse_args()


def _joint_state_to_map(message: JointState | None) -> dict[str, float] | None:
    if message is None:
        return None
    return {name: float(position) for name, position in zip(message.name, message.position)}


def _expected_stand_map(
    joint_config_path: Path, geometry_config_path: Path
) -> dict[str, float]:
    raw_joint = load_yaml(joint_config_path)
    raw_geom = load_yaml(geometry_config_path)
    geometry = HexapodGeometry.from_config_files(joint_config_path, geometry_config_path)
    ik = LegIKSolver(
        max_iterations=raw_geom["ik"]["max_iterations"],
        tolerance=raw_geom["ik"]["tolerance"],
        damping=raw_geom["ik"]["damping"],
        finite_difference_step=raw_geom["ik"]["finite_difference_step"],
        max_delta_per_iteration=raw_geom["ik"]["max_delta_per_iteration"],
        joint_lower_limit=raw_geom["ik"]["joint_lower_limit"],
        joint_upper_limit=raw_geom["ik"]["joint_upper_limit"],
        joint_lower_limits=raw_geom["ik"]["joint_lower_limits"],
        joint_upper_limits=raw_geom["ik"]["joint_upper_limits"],
    )

    expected: dict[str, float] = {}
    for leg_name in geometry.leg_order:
        target = raw_geom["stance"]["per_leg_stance_positions"][leg_name]
        solution = ik.solve(
            geometry,
            leg_name,
            [float(target["x"]), float(target["y"]), float(target["z"])],
            initial_guess=geometry.neutral_joint_positions[leg_name],
        )
        for joint_index, joint_name in enumerate(
            raw_joint["legs"][leg_name]["actuated_joints"]
        ):
            expected[joint_name] = float(solution.joint_angles[joint_index])
    return expected


def _foot_space_summary(
    geometry: HexapodGeometry,
    raw_joint: dict[str, Any],
    raw_geom: dict[str, Any],
    joint_map: dict[str, float] | None,
    label: str,
) -> dict[str, Any]:
    if joint_map is None:
        return {
            "label": label,
            "available": False,
            "max_abs_foot_delta_m": None,
            "legs": {},
        }

    per_leg: dict[str, Any] = {}
    max_abs_delta = 0.0
    for leg_name in geometry.leg_order:
        joint_names = raw_joint["legs"][leg_name]["actuated_joints"]
        joint_angles = [joint_map[joint_name] for joint_name in joint_names]
        current_foot = geometry.forward_leg(leg_name, joint_angles)
        expected_cfg = raw_geom["stance"]["per_leg_stance_positions"][leg_name]
        expected_foot = [
            float(expected_cfg["x"]),
            float(expected_cfg["y"]),
            float(expected_cfg["z"]),
        ]
        delta = [current_foot[idx] - expected_foot[idx] for idx in range(3)]
        leg_abs_delta = max(abs(value) for value in delta)
        max_abs_delta = max(max_abs_delta, leg_abs_delta)
        per_leg[leg_name] = {
            "current_foot_xyz_m": current_foot,
            "expected_foot_xyz_m": expected_foot,
            "delta_xyz_m": delta,
            "max_abs_delta_m": leg_abs_delta,
        }

    return {
        "label": label,
        "available": True,
        "max_abs_foot_delta_m": max_abs_delta,
        "legs": per_leg,
    }


def _compare_joint_maps(
    left_name: str, left_map: dict[str, float] | None, right_name: str, right_map: dict[str, float] | None
) -> dict[str, Any]:
    if left_map is None or right_map is None:
        return {
            "left": left_name,
            "right": right_name,
            "available": False,
            "max_abs_diff_rad": None,
            "top_diffs": [],
        }

    common = sorted(set(left_map) & set(right_map))
    diffs = [
        {
            "joint": joint_name,
            left_name: left_map[joint_name],
            right_name: right_map[joint_name],
            "abs_diff_rad": abs(left_map[joint_name] - right_map[joint_name]),
        }
        for joint_name in common
    ]
    diffs.sort(key=lambda item: item["abs_diff_rad"], reverse=True)
    return {
        "left": left_name,
        "right": right_name,
        "available": True,
        "max_abs_diff_rad": diffs[0]["abs_diff_rad"] if diffs else 0.0,
        "top_diffs": diffs[:6],
    }


def _fetch_isaac_state(
    host: str, port: int, timeout: float, articulation_path: str
) -> dict[str, Any]:
    code = f"""
import json
import traceback
import numpy as np
try:
    from isaacsim.core.prims import Articulation
    art = Articulation({articulation_path!r})
    art.initialize()
    action = art.get_applied_actions()
    payload = {{
        'articulation_path': {articulation_path!r},
        'dof_names': list(art.dof_names),
        'joint_positions': np.asarray(art.get_joint_positions()).reshape(-1).tolist(),
        'joint_velocities': np.asarray(art.get_joint_velocities()).reshape(-1).tolist(),
        'world_position': np.asarray(art.get_world_poses()[0]).reshape(-1).tolist(),
        'world_orientation_xyzw': np.asarray(art.get_world_poses()[1]).reshape(-1).tolist(),
        'solver_position_iterations': np.asarray(art.get_solver_position_iteration_counts()).reshape(-1).tolist(),
        'solver_velocity_iterations': np.asarray(art.get_solver_velocity_iteration_counts()).reshape(-1).tolist(),
        'applied_action_positions': np.asarray(action.joint_positions).reshape(-1).tolist() if getattr(action, 'joint_positions', None) is not None else None,
        'applied_action_velocities': np.asarray(action.joint_velocities).reshape(-1).tolist() if getattr(action, 'joint_velocities', None) is not None else None,
        'applied_action_efforts': np.asarray(action.joint_efforts).reshape(-1).tolist() if getattr(action, 'joint_efforts', None) is not None else None,
    }}
    print(json.dumps(payload))
except Exception:
    traceback.print_exc()
"""
    reply = _execute_code(host=host, port=port, timeout=timeout, code=code)
    if reply.get("status") != "ok":
        return {"available": False, "error": reply.get("output", "")}

    output = reply.get("output", "").strip()
    if not output:
        return {"available": False, "error": "Isaac reply had empty output."}

    try:
        payload = json.loads(output)
    except json.JSONDecodeError:
        return {"available": False, "error": output}

    return {"available": True, **payload}


def main() -> None:
    args = _parse_args()

    raw_joint = load_yaml(args.joint_config)
    raw_geom = load_yaml(args.geometry_config)
    geometry = HexapodGeometry.from_config_files(args.joint_config, args.geometry_config)
    expected_stand = _expected_stand_map(args.joint_config, args.geometry_config)

    rclpy.init()
    node = JointCapture(args.joint_state_topic, args.joint_command_topic)
    deadline_ns = (
        node.get_clock().now().nanoseconds + int(max(args.probe_timeout_sec, 0.1) * 1.0e9)
    )
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline_ns:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.joint_state_msg is not None and node.joint_command_msg is not None:
            break

    joint_state_map = _joint_state_to_map(node.joint_state_msg)
    joint_command_map = _joint_state_to_map(node.joint_command_msg)
    node.destroy_node()
    rclpy.shutdown()

    isaac_state = _fetch_isaac_state(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        articulation_path=args.articulation_path,
    )
    isaac_joint_map = None
    applied_action_map = None
    if isaac_state.get("available"):
        names = isaac_state.get("dof_names", [])
        positions = isaac_state.get("joint_positions", [])
        applied_positions = isaac_state.get("applied_action_positions", [])
        isaac_joint_map = {
            str(name): float(position) for name, position in zip(names, positions)
        }
        if applied_positions is not None:
            applied_action_map = {
                str(name): float(position)
                for name, position in zip(names, applied_positions)
            }

    payload = {
        "joint_state_topic": args.joint_state_topic,
        "joint_command_topic": args.joint_command_topic,
        "expected_joint_count": len(expected_stand),
        "joint_state_received": joint_state_map is not None,
        "joint_command_received": joint_command_map is not None,
        "expected_vs_command": _compare_joint_maps(
            "expected", expected_stand, "command", joint_command_map
        ),
        "command_vs_state": _compare_joint_maps(
            "command", joint_command_map, "state", joint_state_map
        ),
        "expected_vs_state": _compare_joint_maps(
            "expected", expected_stand, "state", joint_state_map
        ),
        "state_foot_vs_expected": _foot_space_summary(
            geometry, raw_joint, raw_geom, joint_state_map, "state"
        ),
        "command_foot_vs_expected": _foot_space_summary(
            geometry, raw_joint, raw_geom, joint_command_map, "command"
        ),
        "isaac": isaac_state,
        "expected_vs_isaac": _compare_joint_maps(
            "expected", expected_stand, "isaac_state", isaac_joint_map
        ),
        "command_vs_isaac_applied": _compare_joint_maps(
            "command", joint_command_map, "isaac_applied", applied_action_map
        ),
    }

    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
