#!/usr/bin/env python3
"""Synthesize explicit foot collider geometry into the hexbot USD asset.

This is a fallback repair path for the current Isaac Sim URDF importer result,
where ``foot_*`` links are present only as Xform placeholders and do not
contain any traversable geometry prims under their collision subtree.

The script parses the URDF to recover each foot link's collision sphere
definition, then authors a real ``UsdGeom.Sphere`` child under each ``foot_*``
prim so downstream audit/material scripts can target an actual collider Gprim.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from urllib.parse import unquote, urlparse

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

try:
    import omni.usd  # type: ignore
except ImportError:  # pragma: no cover - only available inside Isaac Sim
    omni = None

from foot_collision_defs import (
    DEFAULT_COLLISION_CHILD_NAME,
    FOOT_LEGS,
    load_foot_collision_defs,
)


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_USD_PATH = (
    THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted_reimport" / "hexbot_isaac_rooted_reimport.usd"
)
DEFAULT_URDF_PATH = THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted.urdf"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Author explicit sphere colliders under each foot_* prim."
    )
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD_PATH)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF_PATH)
    parser.add_argument(
        "--child-name",
        default=None,
        help=(
            "Override name of the authored child sphere prim under each foot root. "
            f"Default inherits the URDF collision name or falls back to "
            f"{DEFAULT_COLLISION_CHILD_NAME!r}."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print intended changes without saving the stage.",
    )
    return parser.parse_args()


def resolve_path(path: Path, base_dir: Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


def normalize_layer_identifier(identifier: str) -> str:
    if identifier.startswith("file:"):
        parsed = urlparse(identifier)
        return str(Path(unquote(parsed.path)).resolve())
    if "://" not in identifier:
        return str(Path(identifier).resolve())
    return identifier


def open_stage(usd_path: Path) -> tuple[Usd.Stage | None, str]:
    if omni is not None:
        try:
            context = omni.usd.get_context()
            live_stage = context.get_stage() if context is not None else None
        except Exception:
            live_stage = None
        if live_stage is not None:
            live_identifier = normalize_layer_identifier(
                live_stage.GetRootLayer().identifier
            )
            target_identifier = normalize_layer_identifier(str(usd_path))
            if live_identifier == target_identifier:
                return live_stage, "live"
    return Usd.Stage.Open(str(usd_path)), "file"


def rpy_to_quat(rpy: tuple[float, float, float]) -> Gf.Quatf:
    roll, pitch, yaw = rpy
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

def find_foot_root(stage: Usd.Stage, link_name: str) -> Usd.Prim | None:
    matches: list[Usd.Prim] = []
    for prim in stage.Traverse():
        if prim.GetName() != link_name:
            continue
        matches.append(prim)
    if not matches:
        return None
    matches.sort(key=lambda prim: len(prim.GetPath().pathString))
    return matches[0]


def ensure_sphere_collider(
    foot_root: Usd.Prim,
    child_name: str,
    radius: float,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> str:
    sphere_path = foot_root.GetPath().AppendChild(child_name)
    sphere = UsdGeom.Sphere.Define(foot_root.GetStage(), sphere_path)
    sphere_prim = sphere.GetPrim()

    UsdPhysics.CollisionAPI.Apply(sphere_prim)
    PhysxSchema.PhysxCollisionAPI.Apply(sphere_prim)

    sphere.GetRadiusAttr().Set(float(radius))
    sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.55, 0.1)])

    xformable = UsdGeom.Xformable(sphere_prim)
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*xyz))
    if any(abs(value) > 1.0e-9 for value in rpy):
        orient_op = xformable.AddOrientOp()
        orient_op.Set(rpy_to_quat(rpy))

    collision_attr = sphere_prim.GetAttribute("physics:collisionEnabled")
    if not collision_attr or not collision_attr.IsValid():
        collision_attr = sphere_prim.CreateAttribute(
            "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, custom=False
        )
    collision_attr.Set(True)

    purpose_attr = sphere_prim.GetAttribute("purpose")
    if not purpose_attr or not purpose_attr.IsValid():
        purpose_attr = sphere_prim.CreateAttribute(
            "purpose", Sdf.ValueTypeNames.Token, custom=False
        )
    purpose_attr.Set("default")

    return sphere_path.pathString


def main() -> int:
    args = parse_args()
    usd_path = resolve_path(args.usd, THIS_DIR)
    urdf_path = resolve_path(args.urdf, THIS_DIR)
    foot_defs = load_foot_collision_defs(urdf_path)
    if len(foot_defs) != len(FOOT_LEGS):
        print(
            f"Failed to recover all foot collision definitions from {urdf_path}. "
            f"Recovered={len(foot_defs)}",
            file=sys.stderr,
        )
        return 1

    stage, stage_source = open_stage(usd_path)
    if stage is None:
        print(f"Failed to open stage: {usd_path}", file=sys.stderr)
        return 2

    created_paths: list[str] = []
    missing_roots: list[str] = []
    for link_name, collision_def in sorted(foot_defs.items()):
        foot_root = find_foot_root(stage, link_name)
        if foot_root is None or not foot_root.IsValid():
            missing_roots.append(link_name)
            continue
        child_name = (
            args.child_name
            or collision_def.get("collision_name")
            or DEFAULT_COLLISION_CHILD_NAME
        )
        created_paths.append(
            ensure_sphere_collider(
                foot_root=foot_root,
                child_name=child_name,
                radius=collision_def["radius"],
                xyz=collision_def["xyz"],
                rpy=collision_def["rpy"],
            )
        )

    print(f"USD: {usd_path}")
    print(f"URDF: {urdf_path}")
    print(f"stageSource={stage_source}")
    print(f"createdColliderCount={len(created_paths)}")
    for prim_path in created_paths:
        print(f"  - {prim_path}")
    if missing_roots:
        print("Missing foot roots:", file=sys.stderr)
        for name in missing_roots:
            print(f"  - {name}", file=sys.stderr)
        return 3

    if args.dry_run:
        print("Dry run enabled, USD was not saved.")
        return 0

    stage.GetRootLayer().Save()
    print(f"Saved explicit foot collider spheres into {stage.GetRootLayer().identifier}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
