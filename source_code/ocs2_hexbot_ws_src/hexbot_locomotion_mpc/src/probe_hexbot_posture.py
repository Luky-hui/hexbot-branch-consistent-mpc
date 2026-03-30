#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock


THIS_FILE = Path(__file__).resolve()
WORKSPACE_ROOT = THIS_FILE.parents[3]
ISAAC_SRC = WORKSPACE_ROOT / "src" / "hexbot_description_ros2" / "isaac"
if str(ISAAC_SRC) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(ISAAC_SRC))

from isaac_vscode_exec import _execute_code  # noqa: E402


def wrap_to_pi(angle_rad: float) -> float:
    wrapped = float(angle_rad)
    while wrapped > math.pi:
        wrapped -= 2.0 * math.pi
    while wrapped < -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def unwrap_angle(previous_angle: float | None, current_angle: float) -> float:
    if previous_angle is None:
        return current_angle
    delta = wrap_to_pi(current_angle - previous_angle)
    return previous_angle + delta


def mean(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / float(len(values))


def variance(values: list[float]) -> float | None:
    if not values:
        return None
    mu = mean(values)
    if mu is None:
        return None
    return sum((value - mu) ** 2 for value in values) / float(len(values))


def stddev(values: list[float]) -> float | None:
    value = variance(values)
    return None if value is None else math.sqrt(value)


def quaternion_to_rpy(quaternion) -> tuple[float, float, float]:
    x_value = float(quaternion.x)
    y_value = float(quaternion.y)
    z_value = float(quaternion.z)
    w_value = float(quaternion.w)
    sinr_cosp = 2.0 * (w_value * x_value + y_value * z_value)
    cosr_cosp = 1.0 - 2.0 * (x_value * x_value + y_value * y_value)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w_value * y_value - z_value * x_value)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_xyzw_to_yaw(quaternion: list[float] | tuple[float, ...] | None) -> float | None:
    if quaternion is None or len(quaternion) != 4:
        return None
    x_value = float(quaternion[0])
    y_value = float(quaternion[1])
    z_value = float(quaternion[2])
    w_value = float(quaternion[3])
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def extract_isaac_motion(
    isaac_start: dict[str, Any],
    isaac_end: dict[str, Any],
    base_link_path: str,
) -> dict[str, Any]:
    start_prim = (isaac_start.get("prims") or {}).get(base_link_path) or {}
    end_prim = (isaac_end.get("prims") or {}).get(base_link_path) or {}

    start_position = start_prim.get("world_translate")
    end_position = end_prim.get("world_translate")
    position_source = "base_link_prim"
    if start_position is None or end_position is None:
        start_position = (isaac_start.get("articulation") or {}).get("world_position")
        end_position = (isaac_end.get("articulation") or {}).get("world_position")
        position_source = "articulation"

    start_yaw = quaternion_xyzw_to_yaw(start_prim.get("world_orientation_xyzw"))
    end_yaw = quaternion_xyzw_to_yaw(end_prim.get("world_orientation_xyzw"))
    yaw_source = "base_link_prim"
    if start_yaw is None or end_yaw is None:
        start_yaw = quaternion_xyzw_to_yaw(
            (isaac_start.get("articulation") or {}).get("world_orientation_xyzw")
        )
        end_yaw = quaternion_xyzw_to_yaw(
            (isaac_end.get("articulation") or {}).get("world_orientation_xyzw")
        )
        yaw_source = "articulation"

    delta_x_m = None
    delta_y_m = None
    delta_z_m = None
    delta_forward_m = None
    delta_lateral_m = None
    if start_position is not None and end_position is not None:
        delta_x_m = float(end_position[0]) - float(start_position[0])
        delta_y_m = float(end_position[1]) - float(start_position[1])
        delta_z_m = float(end_position[2]) - float(start_position[2])
        if start_yaw is not None:
            cos_yaw = math.cos(start_yaw)
            sin_yaw = math.sin(start_yaw)
            delta_forward_m = cos_yaw * delta_x_m + sin_yaw * delta_y_m
            delta_lateral_m = -sin_yaw * delta_x_m + cos_yaw * delta_y_m

    return {
        "position_source": position_source,
        "yaw_source": yaw_source,
        "start_yaw_rad": start_yaw,
        "end_yaw_rad": end_yaw,
        "delta_yaw_rad": None
        if start_yaw is None or end_yaw is None
        else wrap_to_pi(end_yaw - start_yaw),
        "delta_x_m": delta_x_m,
        "delta_y_m": delta_y_m,
        "delta_z_m": delta_z_m,
        "delta_forward_m": delta_forward_m,
        "delta_lateral_m": delta_lateral_m,
    }


def odom_is_finite(message: Odometry) -> bool:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    linear = message.twist.twist.linear
    angular = message.twist.twist.angular
    values = [
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        linear.x,
        linear.y,
        linear.z,
        angular.x,
        angular.y,
        angular.z,
    ]
    return all(math.isfinite(float(value)) for value in values)


def pose_sample(message: Odometry, elapsed_sec: float) -> dict[str, float]:
    roll, pitch, yaw = quaternion_to_rpy(message.pose.pose.orientation)
    return {
        "elapsed_sec": elapsed_sec,
        "x_m": float(message.pose.pose.position.x),
        "y_m": float(message.pose.pose.position.y),
        "z_m": float(message.pose.pose.position.z),
        "roll_rad": roll,
        "pitch_rad": pitch,
        "yaw_rad": yaw,
        "linear_speed_mps": math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        ),
        "angular_speed_radps": abs(float(message.twist.twist.angular.z)),
    }


class PostureProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hexbot_posture_probe")
        self.args = args
        if args.use_sim_time:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.wall_start_sec = time.monotonic()
        self.clock_start_sec: float | None = None
        self.clock_now_sec: float | None = None

        self.odom_messages = 0
        self.odom_nonfinite_messages = 0
        self.start_sample: dict[str, float] | None = None
        self.end_sample: dict[str, float] | None = None
        self.min_z_sample: dict[str, float] | None = None
        self.max_z_sample: dict[str, float] | None = None
        self.max_abs_roll_sample: dict[str, float] | None = None
        self.max_abs_pitch_sample: dict[str, float] | None = None
        self.odom_samples: list[dict[str, float]] = []
        self.previous_yaw_unwrapped_rad: float | None = None

        self.create_subscription(Clock, args.clock_topic, self._on_clock, 20)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, 20)

    def _elapsed_sec(self) -> float | None:
        if not self.args.use_sim_time:
            return time.monotonic() - self.wall_start_sec
        if self.clock_start_sec is None or self.clock_now_sec is None:
            return None
        return self.clock_now_sec - self.clock_start_sec

    def _on_clock(self, message: Clock) -> None:
        clock_sec = float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        if self.clock_start_sec is None:
            self.clock_start_sec = clock_sec
        self.clock_now_sec = clock_sec

    def _on_odom(self, message: Odometry) -> None:
        self.odom_messages += 1
        if not odom_is_finite(message):
            self.odom_nonfinite_messages += 1
            return

        elapsed_sec = self._elapsed_sec()
        if elapsed_sec is None:
            return

        sample = pose_sample(message, elapsed_sec)
        sample["yaw_unwrapped_rad"] = unwrap_angle(
            self.previous_yaw_unwrapped_rad, float(sample["yaw_rad"])
        )
        self.previous_yaw_unwrapped_rad = float(sample["yaw_unwrapped_rad"])
        if self.start_sample is None:
            self.start_sample = sample
        self.end_sample = sample
        self.odom_samples.append(sample)

        if self.min_z_sample is None or sample["z_m"] < self.min_z_sample["z_m"]:
            self.min_z_sample = sample
        if self.max_z_sample is None or sample["z_m"] > self.max_z_sample["z_m"]:
            self.max_z_sample = sample
        if (
            self.max_abs_roll_sample is None
            or abs(sample["roll_rad"]) > abs(self.max_abs_roll_sample["roll_rad"])
        ):
            self.max_abs_roll_sample = sample
        if (
            self.max_abs_pitch_sample is None
            or abs(sample["pitch_rad"]) > abs(self.max_abs_pitch_sample["pitch_rad"])
        ):
            self.max_abs_pitch_sample = sample


def fetch_isaac_snapshot(
    host: str,
    port: int,
    timeout: float,
    robot_root_path: str,
    base_link_path: str,
    ground_path: str,
    articulation_path: str,
) -> dict[str, Any]:
    code = f"""
import json
import omni.timeline
import omni.usd
from pxr import UsdGeom

payload = {{}}
t = omni.timeline.get_timeline_interface()
stage = omni.usd.get_context().get_stage()
payload['stage_url'] = omni.usd.get_context().get_stage_url()
payload['playing'] = bool(t.is_playing())
payload['stopped'] = bool(t.is_stopped())
cache = UsdGeom.XformCache()
payload['prims'] = {{}}
for path in [{robot_root_path!r}, {base_link_path!r}, {ground_path!r}]:
    prim = stage.GetPrimAtPath(path)
    prim_payload = {{'valid': bool(prim)}}
    if prim:
        transform = cache.GetLocalToWorldTransform(prim)
        translation = transform.ExtractTranslation()
        rotation = transform.ExtractRotation().GetQuat()
        prim_payload['world_translate'] = [
            float(translation[0]),
            float(translation[1]),
            float(translation[2]),
        ]
        prim_payload['world_orientation_xyzw'] = [
            float(rotation.GetImaginary()[0]),
            float(rotation.GetImaginary()[1]),
            float(rotation.GetImaginary()[2]),
            float(rotation.GetReal()),
        ]
    payload['prims'][path] = prim_payload

try:
    import numpy as np
    from isaacsim.core.prims import Articulation
    art = Articulation({articulation_path!r})
    art.initialize()
    world_position, world_orientation = art.get_world_poses()
    payload['articulation'] = {{
        'available': True,
        'world_position': np.asarray(world_position).reshape(-1).tolist(),
        'world_orientation_xyzw': np.asarray(world_orientation).reshape(-1).tolist(),
    }}
except Exception as exc:
    payload['articulation'] = {{
        'available': False,
        'error': str(exc),
    }}

print(json.dumps(payload))
"""
    reply = _execute_code(host=host, port=port, timeout=timeout, code=code)
    if reply.get("status") != "ok":
        return {"available": False, "error": reply.get("output", ""), "reply": reply}

    output = str(reply.get("output", "")).strip()
    if not output:
        return {"available": False, "error": "empty output", "reply": reply}
    try:
        payload = json.loads(output)
    except json.JSONDecodeError:
        return {"available": False, "error": output, "reply": reply}
    payload["available"] = True
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Observe Hexbot trunk/base posture from /odom and the live Isaac stage, "
            "and decide whether the body is sinking or entering an abnormal pose."
        )
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--spin-timeout", type=float, default=0.1)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8226)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--robot-root-path", default="/hexbot/hexbot")
    parser.add_argument("--base-link-path", default="/hexbot/hexbot/base_link")
    parser.add_argument("--ground-path", default="/SimpleRoom/GroundPlane")
    parser.add_argument("--articulation-path", default="/hexbot/hexbot")
    parser.add_argument("--min-odom-samples", type=int, default=3)
    parser.add_argument("--max-base-drop-m", type=float, default=0.04)
    parser.add_argument("--max-final-base-drop-m", type=float, default=0.03)
    parser.add_argument("--max-abs-roll-rad", type=float, default=0.35)
    parser.add_argument("--max-abs-pitch-rad", type=float, default=0.35)
    parser.add_argument("--max-final-roll-rad", type=float, default=0.25)
    parser.add_argument("--max-final-pitch-rad", type=float, default=0.25)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    isaac_start = fetch_isaac_snapshot(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        robot_root_path=args.robot_root_path,
        base_link_path=args.base_link_path,
        ground_path=args.ground_path,
        articulation_path=args.articulation_path,
    )

    rclpy.init()
    node = PostureProbe(args)
    try:
        deadline = time.monotonic() + max(1.0, float(args.duration))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=float(args.spin_timeout))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    isaac_end = fetch_isaac_snapshot(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        robot_root_path=args.robot_root_path,
        base_link_path=args.base_link_path,
        ground_path=args.ground_path,
        articulation_path=args.articulation_path,
    )

    failures: list[str] = []
    start_sample = node.start_sample
    end_sample = node.end_sample
    min_z_sample = node.min_z_sample
    max_z_sample = node.max_z_sample
    max_abs_roll_sample = node.max_abs_roll_sample
    max_abs_pitch_sample = node.max_abs_pitch_sample

    trunk_sink_detected = False
    weird_pose_detected = False

    if node.odom_messages < int(args.min_odom_samples):
        failures.append(
            f"{args.odom_topic} received only {node.odom_messages} samples (expected at least {args.min_odom_samples})."
        )
    if node.odom_nonfinite_messages > 0:
        failures.append(
            f"{args.odom_topic} observed {node.odom_nonfinite_messages} non-finite messages."
        )

    base_drop_m = None
    final_base_drop_m = None
    max_abs_roll_rad = None
    max_abs_pitch_rad = None
    final_roll_rad = None
    final_pitch_rad = None
    odom_delta_yaw_rad = None
    yaw_drift_deg = None
    z_height_mean_m = None
    z_height_variance_m2 = None
    z_height_std_m = None
    z_height_range_m = None

    if start_sample is None or end_sample is None or min_z_sample is None:
        failures.append("No finite odom posture samples were collected.")
    else:
        base_drop_m = float(start_sample["z_m"] - min_z_sample["z_m"])
        final_base_drop_m = float(start_sample["z_m"] - end_sample["z_m"])
        max_abs_roll_rad = abs(float(max_abs_roll_sample["roll_rad"])) if max_abs_roll_sample else None
        max_abs_pitch_rad = abs(float(max_abs_pitch_sample["pitch_rad"])) if max_abs_pitch_sample else None
        final_roll_rad = float(end_sample["roll_rad"])
        final_pitch_rad = float(end_sample["pitch_rad"])
        if "yaw_unwrapped_rad" in start_sample and "yaw_unwrapped_rad" in end_sample:
            odom_delta_yaw_rad = float(end_sample["yaw_unwrapped_rad"]) - float(
                start_sample["yaw_unwrapped_rad"]
            )
            yaw_drift_deg = math.degrees(odom_delta_yaw_rad)
        z_values = [float(sample["z_m"]) for sample in node.odom_samples]
        z_height_mean_m = mean(z_values)
        z_height_variance_m2 = variance(z_values)
        z_height_std_m = stddev(z_values)
        if min_z_sample is not None and max_z_sample is not None:
            z_height_range_m = float(max_z_sample["z_m"]) - float(min_z_sample["z_m"])

        if base_drop_m > float(args.max_base_drop_m):
            trunk_sink_detected = True
            failures.append(
                f"Trunk sink detected: base_drop_m={base_drop_m:.4f} > {float(args.max_base_drop_m):.4f}."
            )
        if final_base_drop_m > float(args.max_final_base_drop_m):
            trunk_sink_detected = True
            failures.append(
                f"Final trunk height remained low: final_base_drop_m={final_base_drop_m:.4f} > {float(args.max_final_base_drop_m):.4f}."
            )
        if max_abs_roll_rad is not None and max_abs_roll_rad > float(args.max_abs_roll_rad):
            weird_pose_detected = True
            failures.append(
                f"Weird pose detected from roll: max_abs_roll_rad={max_abs_roll_rad:.4f} > {float(args.max_abs_roll_rad):.4f}."
            )
        if max_abs_pitch_rad is not None and max_abs_pitch_rad > float(args.max_abs_pitch_rad):
            weird_pose_detected = True
            failures.append(
                f"Weird pose detected from pitch: max_abs_pitch_rad={max_abs_pitch_rad:.4f} > {float(args.max_abs_pitch_rad):.4f}."
            )
        if abs(final_roll_rad) > float(args.max_final_roll_rad):
            weird_pose_detected = True
            failures.append(
                f"Final roll stayed abnormal: |roll|={abs(final_roll_rad):.4f} > {float(args.max_final_roll_rad):.4f}."
            )
        if abs(final_pitch_rad) > float(args.max_final_pitch_rad):
            weird_pose_detected = True
            failures.append(
                f"Final pitch stayed abnormal: |pitch|={abs(final_pitch_rad):.4f} > {float(args.max_final_pitch_rad):.4f}."
            )

    payload = {
        "ok": len(failures) == 0,
        "trunk_sink_detected": trunk_sink_detected,
        "weird_pose_detected": weird_pose_detected,
        "failures": failures,
        "message_counts": {
            "odom_total": node.odom_messages,
            "odom_nonfinite": node.odom_nonfinite_messages,
        },
        "metrics": {
            "base_drop_m": base_drop_m,
            "final_base_drop_m": final_base_drop_m,
            "max_abs_roll_rad": max_abs_roll_rad,
            "max_abs_pitch_rad": max_abs_pitch_rad,
            "final_roll_rad": final_roll_rad,
            "final_pitch_rad": final_pitch_rad,
            "odom_delta_yaw_rad": odom_delta_yaw_rad,
            "yaw_drift_deg": yaw_drift_deg,
            "z_height_mean_m": z_height_mean_m,
            "z_height_variance_m2": z_height_variance_m2,
            "z_height_std_m": z_height_std_m,
            "z_height_range_m": z_height_range_m,
        },
        "samples": {
            "start": start_sample,
            "end": end_sample,
            "min_z": min_z_sample,
            "max_z": max_z_sample,
            "max_abs_roll": max_abs_roll_sample,
            "max_abs_pitch": max_abs_pitch_sample,
        },
        "odom_trace": {
            "sample_count": len(node.odom_samples),
            "samples": node.odom_samples,
        },
        "isaac": {
            "start": isaac_start,
            "end": isaac_end,
        },
        "isaac_motion": extract_isaac_motion(
            isaac_start=isaac_start,
            isaac_end=isaac_end,
            base_link_path=args.base_link_path,
        ),
        "thresholds": {
            "min_odom_samples": int(args.min_odom_samples),
            "max_base_drop_m": float(args.max_base_drop_m),
            "max_final_base_drop_m": float(args.max_final_base_drop_m),
            "max_abs_roll_rad": float(args.max_abs_roll_rad),
            "max_abs_pitch_rad": float(args.max_abs_pitch_rad),
            "max_final_roll_rad": float(args.max_final_roll_rad),
            "max_final_pitch_rad": float(args.max_final_pitch_rad),
        },
    }

    text = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.output is not None:
        args.output.write_text(text + "\n", encoding="utf-8")
    raise SystemExit(0 if payload["ok"] else 1)


if __name__ == "__main__":
    main()
