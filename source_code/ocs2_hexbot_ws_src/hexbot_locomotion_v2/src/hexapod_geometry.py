from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
from typing import Any


def load_yaml(path: str | Path) -> dict[str, Any]:
    import yaml

    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def rotation_from_rpy(
    rpy: list[float] | tuple[float, float, float]
) -> list[list[float]]:
    roll, pitch, yaw = [float(value) for value in rpy]
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)

    rot_x = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    rot_y = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    rot_z = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    return mat3_mul(rot_z, mat3_mul(rot_y, rot_x))


def mat3_mul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    product = [[0.0, 0.0, 0.0] for _ in range(3)]
    for row in range(3):
        for col in range(3):
            product[row][col] = sum(left[row][idx] * right[idx][col] for idx in range(3))
    return product


def mat4_mul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    product = [[0.0, 0.0, 0.0, 0.0] for _ in range(4)]
    for row in range(4):
        for col in range(4):
            product[row][col] = sum(left[row][idx] * right[idx][col] for idx in range(4))
    return product


def transform_from_xyz_rpy(
    xyz: list[float] | tuple[float, float, float],
    rpy: list[float] | tuple[float, float, float],
) -> list[list[float]]:
    transform = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    rotation = rotation_from_rpy(rpy)
    for row in range(3):
        for col in range(3):
            transform[row][col] = rotation[row][col]
        transform[row][3] = float(xyz[row])
    return transform


def axis_angle_transform(
    axis: list[float] | tuple[float, float, float], angle: float
) -> list[list[float]]:
    axis_vec = [float(value) for value in axis]
    axis_norm = math.sqrt(sum(value * value for value in axis_vec))
    if axis_norm == 0.0:
        raise ValueError("Joint axis cannot be zero.")
    axis_vec = [value / axis_norm for value in axis_vec]

    x_axis, y_axis, z_axis = axis_vec
    skew = [[0.0, -z_axis, y_axis], [z_axis, 0.0, -x_axis], [-y_axis, x_axis, 0.0]]
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    skew_squared = mat3_mul(skew, skew)
    sin_angle = math.sin(angle)
    cos_angle = math.cos(angle)
    rot = [[0.0, 0.0, 0.0] for _ in range(3)]
    for row in range(3):
        for col in range(3):
            rot[row][col] = (
                identity[row][col]
                + sin_angle * skew[row][col]
                + (1.0 - cos_angle) * skew_squared[row][col]
            )

    transform = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for row in range(3):
        for col in range(3):
            transform[row][col] = rot[row][col]
    return transform


@dataclass(frozen=True)
class LegKinematicChain:
    name: str
    joint_names: tuple[str, str, str]
    mount_xyz: tuple[float, float, float]
    mount_rpy: tuple[float, float, float]
    coxa_axis: tuple[float, float, float]
    femur_axis: tuple[float, float, float]
    tarsus_axis: tuple[float, float, float]
    coxa_to_femur: list[list[float]]
    femur_to_tarsus: list[list[float]]
    tarsus_to_foot: list[list[float]]

    def forward_kinematics(self, joint_angles: list[float] | tuple[float, float, float]) -> list[float]:
        q_coxa, q_femur, q_tarsus = [float(value) for value in joint_angles]

        transform = transform_from_xyz_rpy(self.mount_xyz, self.mount_rpy)
        transform = mat4_mul(transform, axis_angle_transform(self.coxa_axis, q_coxa))
        transform = mat4_mul(transform, self.coxa_to_femur)
        transform = mat4_mul(transform, axis_angle_transform(self.femur_axis, q_femur))
        transform = mat4_mul(transform, self.femur_to_tarsus)
        transform = mat4_mul(transform, axis_angle_transform(self.tarsus_axis, q_tarsus))
        transform = mat4_mul(transform, self.tarsus_to_foot)
        return [transform[0][3], transform[1][3], transform[2][3]]


class HexapodGeometry:
    def __init__(self, joint_config: dict[str, Any], geometry_config: dict[str, Any]) -> None:
        self.robot_name = str(geometry_config["robot_name"])
        self.leg_order = list(geometry_config["leg_order"])
        self.joint_names = list(joint_config["joint_names"])
        self.ik_config = dict(geometry_config["ik"])
        self.controller_config = dict(geometry_config["controller"])
        self.gait_config = dict(geometry_config["gait"])
        self.neutral_joint_positions = {
            leg_name: [float(value) for value in angles]
            for leg_name, angles in geometry_config["neutral_joint_positions"].items()
        }

        link_config = geometry_config["links"]
        coxa_to_femur = transform_from_xyz_rpy(
            link_config["coxa_to_femur"]["xyz"], link_config["coxa_to_femur"]["rpy"]
        )
        femur_to_tarsus = transform_from_xyz_rpy(
            link_config["femur_to_tarsus"]["xyz"], link_config["femur_to_tarsus"]["rpy"]
        )
        tarsus_to_foot = transform_from_xyz_rpy(
            link_config["tarsus_to_foot"]["xyz"], link_config["tarsus_to_foot"]["rpy"]
        )

        self.legs: dict[str, LegKinematicChain] = {}
        self.joint_to_leg_index: dict[str, tuple[str, int]] = {}
        for leg_name in self.leg_order:
            leg_joint_names = tuple(joint_config["legs"][leg_name]["actuated_joints"])
            leg_cfg = geometry_config["legs"][leg_name]
            leg_chain = LegKinematicChain(
                name=leg_name,
                joint_names=leg_joint_names,
                mount_xyz=tuple(float(value) for value in leg_cfg["mount_xyz"]),
                mount_rpy=tuple(float(value) for value in leg_cfg["mount_rpy"]),
                coxa_axis=tuple(float(value) for value in leg_cfg["coxa_axis"]),
                femur_axis=tuple(float(value) for value in leg_cfg["femur_axis"]),
                tarsus_axis=tuple(float(value) for value in leg_cfg["tarsus_axis"]),
                coxa_to_femur=coxa_to_femur,
                femur_to_tarsus=femur_to_tarsus,
                tarsus_to_foot=tarsus_to_foot,
            )
            self.legs[leg_name] = leg_chain

            for joint_index, joint_name in enumerate(leg_joint_names):
                self.joint_to_leg_index[joint_name] = (leg_name, joint_index)

        missing_joints = [name for name in self.joint_names if name not in self.joint_to_leg_index]
        if missing_joints:
            raise ValueError(f"Joint ordering contains unknown joints: {missing_joints}")

    @classmethod
    def from_config_files(
        cls, joint_config_path: str | Path, geometry_config_path: str | Path
    ) -> "HexapodGeometry":
        return cls(load_yaml(joint_config_path), load_yaml(geometry_config_path))

    def forward_leg(
        self, leg_name: str, joint_angles: list[float] | tuple[float, float, float]
    ) -> list[float]:
        return self.legs[leg_name].forward_kinematics(joint_angles)

    def neutral_foot_positions(self) -> dict[str, list[float]]:
        return {
            leg_name: self.forward_leg(leg_name, self.neutral_joint_positions[leg_name])
            for leg_name in self.leg_order
        }

    def joint_positions_from_leg_map(self, leg_joint_map: dict[str, list[float]]) -> list[float]:
        ordered_positions: list[float] = []
        for joint_name in self.joint_names:
            leg_name, joint_index = self.joint_to_leg_index[joint_name]
            ordered_positions.append(float(leg_joint_map[leg_name][joint_index]))
        return ordered_positions
