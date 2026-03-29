#!/usr/bin/env python3
"""Apply Isaac Sim drive settings to the hexbot leg joints.

Run this with Isaac Sim's Python so the ``pxr`` bindings are available, e.g.:

    ./python.sh apply_hexbot_isaac_drives.py

The script opens the exported ``hexbot_isaac_rooted.usd`` asset, finds the
18 revolute leg joints, adds the required Drive/PhysX joint APIs, and writes
position-drive parameters back into the USD.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

try:
    import omni.usd  # type: ignore
except ImportError:  # pragma: no cover - only available inside Isaac Sim
    omni = None

from foot_collision_defs import FOOT_LEGS, load_foot_collision_defs


JOINT_NAME_PATTERN = re.compile(
    r"^(?P<group>coxa|femur|tarsus)(?:_joint)?_(?P<leg>L[FMR]|R[FMR])$",
    re.IGNORECASE,
)
FOOT_NAME_TOKENS = tuple(
    f"foot_{leg}".lower() for leg in ("LF", "LM", "LR", "RF", "RM", "RR")
)

THIS_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = THIS_DIR / "hexbot_drive_profile.json"
DEFAULT_URDF_PATH = THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted.urdf"
DEFAULT_USD_PATH = (
    THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted" / "hexbot_isaac_rooted.usd"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply Isaac Sim position-drive settings to hexbot joints."
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=DEFAULT_USD_PATH,
        help="Path to the hexbot USD asset to update.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to the JSON profile describing drive gains.",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF_PATH,
        help="URDF used to resolve expected foot collision child names.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Inspect and print the intended changes without saving the USD.",
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


def load_profile(config_path: Path) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8-sig") as stream:
        profile = json.load(stream)

    required_groups = {"coxa", "femur", "tarsus"}
    configured_groups = set(profile.get("joint_groups", {}))
    missing_groups = required_groups - configured_groups
    if missing_groups:
        raise ValueError(
            f"Profile {config_path} is missing joint_groups entries: "
            f"{', '.join(sorted(missing_groups))}"
        )

    return profile


def find_hexbot_joints(stage: Usd.Stage) -> tuple[dict[str, Usd.Prim], list[str]]:
    matched: dict[str, Usd.Prim] = {}
    all_revolute_names: list[str] = []

    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue

        name = prim.GetName()
        all_revolute_names.append(name)

        if JOINT_NAME_PATTERN.match(name):
            matched[name] = prim

    return matched, sorted(all_revolute_names)


def resolve_stage_path(stage: Usd.Stage, configured_path: str) -> Sdf.Path:
    if configured_path.startswith("/"):
        return Sdf.Path(configured_path)

    default_prim = stage.GetDefaultPrim()
    current_path = (
        default_prim.GetPath()
        if default_prim and default_prim.IsValid()
        else Sdf.Path.absoluteRootPath
    )
    for token in configured_path.split("/"):
        if token:
            current_path = current_path.AppendChild(token)
    return current_path


def ensure_parent_scopes(stage: Usd.Stage, prim_path: Sdf.Path) -> None:
    pending_paths: list[Sdf.Path] = []
    current_path = prim_path.GetParentPath()

    while current_path.pathString not in ("", "/"):
        prim = stage.GetPrimAtPath(current_path)
        if prim and prim.IsValid():
            break
        pending_paths.append(current_path)
        current_path = current_path.GetParentPath()

    for scope_path in reversed(pending_paths):
        stage.DefinePrim(scope_path, "Scope")


def is_foot_related_path(path_text: str) -> bool:
    lower_path = path_text.lower()
    return any(token in lower_path for token in FOOT_NAME_TOKENS)


def find_foot_related_prims(stage: Usd.Stage) -> list[Usd.Prim]:
    return [
        prim for prim in stage.Traverse() if is_foot_related_path(prim.GetPath().pathString)
    ]


def find_foot_geom_prims(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in find_foot_related_prims(stage) if prim.IsA(UsdGeom.Gprim)]


def find_foot_collision_prims(stage: Usd.Stage) -> list[Usd.Prim]:
    matched_prims: list[Usd.Prim] = []
    fallback_prims: list[Usd.Prim] = []

    for prim in find_foot_geom_prims(stage):
        path_text = prim.GetPath().pathString
        lower_path = path_text.lower()
        collision_attr = prim.GetAttribute("physics:collisionEnabled")
        has_collision_attr = bool(collision_attr) and collision_attr.IsValid()

        if prim.HasAPI(UsdPhysics.CollisionAPI) or has_collision_attr:
            matched_prims.append(prim)
            continue

        # URDF-imported feet may appear as direct Gprims at /.../foot_LF etc.
        # Keep the explicit API/attribute checks first, then fall back to any
        # foot-related Gprim so simple sphere feet can still receive material.
        if prim.IsA(UsdGeom.Gprim):
            fallback_prims.append(prim)

    if matched_prims:
        return matched_prims
    return fallback_prims


def find_expected_named_foot_collision_prims(
    stage: Usd.Stage, foot_collision_defs: dict[str, dict[str, Any]]
) -> tuple[list[Usd.Prim], list[str]]:
    matched_prims: list[Usd.Prim] = []
    missing_paths: list[str] = []

    for leg in FOOT_LEGS:
        link_name = f"foot_{leg}"
        collision_def = foot_collision_defs.get(link_name)
        if not collision_def:
            missing_paths.append(f"<missing urdf def for {link_name}>")
            continue

        foot_roots = [
            prim for prim in stage.Traverse() if prim.GetName() == link_name
        ]
        if not foot_roots:
            missing_paths.append(f"<missing foot root {link_name}>")
            continue

        foot_roots.sort(key=lambda prim: len(prim.GetPath().pathString))
        expected_path = foot_roots[0].GetPath().AppendChild(
            collision_def["collision_name"]
        )
        expected_prim = stage.GetPrimAtPath(expected_path)
        if expected_prim and expected_prim.IsValid():
            matched_prims.append(expected_prim)
        else:
            missing_paths.append(expected_path.pathString)

    return matched_prims, missing_paths


def set_attr(prim: Usd.Prim, attr_name: str, type_name: Any, value: Any) -> None:
    attr = prim.GetAttribute(attr_name)
    if not attr:
        attr = prim.CreateAttribute(attr_name, type_name, custom=False)
    attr.Set(value)


def apply_joint_drive(
    joint_prim: Usd.Prim,
    gains: dict[str, Any],
    joint_name: str,
    target_overrides: dict[str, float],
) -> dict[str, Any]:
    UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    PhysxSchema.PhysxJointAPI.Apply(joint_prim)

    target_position = target_overrides.get(
        joint_name, gains.get("target_position_deg", 0.0)
    )

    set_attr(
        joint_prim,
        "drive:angular:physics:stiffness",
        Sdf.ValueTypeNames.Float,
        float(gains["stiffness"]),
    )
    set_attr(
        joint_prim,
        "drive:angular:physics:damping",
        Sdf.ValueTypeNames.Float,
        float(gains["damping"]),
    )
    set_attr(
        joint_prim,
        "drive:angular:physics:maxForce",
        Sdf.ValueTypeNames.Float,
        float(gains["max_force"]),
    )
    set_attr(
        joint_prim,
        "drive:angular:physics:targetPosition",
        Sdf.ValueTypeNames.Float,
        float(target_position),
    )
    set_attr(
        joint_prim,
        "physxJoint:maxJointVelocity",
        Sdf.ValueTypeNames.Float,
        float(gains["max_joint_velocity"]),
    )

    drive_type = gains.get("drive_type")
    if drive_type:
        set_attr(
            joint_prim,
            "drive:angular:physics:type",
            Sdf.ValueTypeNames.Token,
            str(drive_type),
        )

    return {
        "joint": joint_name,
        "prim_path": joint_prim.GetPath().pathString,
        "stiffness": float(gains["stiffness"]),
        "damping": float(gains["damping"]),
        "max_force": float(gains["max_force"]),
        "max_joint_velocity": float(gains["max_joint_velocity"]),
        "target_position_deg": float(target_position),
    }


def apply_foot_material(
    stage: Usd.Stage,
    material_config: dict[str, Any],
    foot_collision_defs: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    if not material_config:
        return None

    material_path = resolve_stage_path(
        stage, material_config.get("path", "PhysicsMaterials/hexbotFootPhysicsMaterial")
    )
    ensure_parent_scopes(stage, material_path)

    material = UsdShade.Material.Define(stage, material_path)
    material_prim = material.GetPrim()
    UsdPhysics.MaterialAPI.Apply(material_prim)
    PhysxSchema.PhysxMaterialAPI.Apply(material_prim)

    set_attr(
        material_prim,
        "physics:staticFriction",
        Sdf.ValueTypeNames.Float,
        float(material_config["static_friction"]),
    )
    set_attr(
        material_prim,
        "physics:dynamicFriction",
        Sdf.ValueTypeNames.Float,
        float(material_config["dynamic_friction"]),
    )
    set_attr(
        material_prim,
        "physics:restitution",
        Sdf.ValueTypeNames.Float,
        float(material_config["restitution"]),
    )

    foot_related_prims = find_foot_related_prims(stage)
    foot_geom_prims = find_foot_geom_prims(stage)
    collision_prims = find_foot_collision_prims(stage)
    named_collision_prims, missing_named_paths = find_expected_named_foot_collision_prims(
        stage, foot_collision_defs
    )
    if len(named_collision_prims) == len(FOOT_LEGS):
        binding_prims = named_collision_prims
        binding_scope = "named_collision_prims"
    else:
        binding_prims = collision_prims if collision_prims else foot_geom_prims
        binding_scope = "collision_prims" if collision_prims else "foot_geom_prims"

    bound_paths: list[str] = []
    for binding_prim in binding_prims:
        if binding_prim.IsA(UsdGeom.Gprim) and not binding_prim.HasAPI(
            UsdPhysics.CollisionAPI
        ):
            UsdPhysics.CollisionAPI.Apply(binding_prim)
        collision_attr = binding_prim.GetAttribute("physics:collisionEnabled")
        if not collision_attr or not collision_attr.IsValid():
            collision_attr = binding_prim.CreateAttribute(
                "physics:collisionEnabled", Sdf.ValueTypeNames.Bool, custom=False
            )
        collision_attr.Set(True)

        UsdShade.MaterialBindingAPI.Apply(binding_prim)
        UsdShade.MaterialBindingAPI(binding_prim).Bind(
            material,
            UsdShade.Tokens.strongerThanDescendants,
            "physics",
        )
        bound_paths.append(binding_prim.GetPath().pathString)

    return {
        "material_path": material_path.pathString,
        "static_friction": float(material_config["static_friction"]),
        "dynamic_friction": float(material_config["dynamic_friction"]),
        "restitution": float(material_config["restitution"]),
        "foot_related_prim_count": len(foot_related_prims),
        "foot_related_prim_examples": [
            prim.GetPath().pathString for prim in foot_related_prims[:12]
        ],
        "foot_geom_prim_count": len(foot_geom_prims),
        "foot_geom_prim_examples": [
            prim.GetPath().pathString for prim in foot_geom_prims[:12]
        ],
        "named_collider_expected_count": len(FOOT_LEGS),
        "named_collider_found_count": len(named_collision_prims),
        "named_collider_paths": [
            prim.GetPath().pathString for prim in named_collision_prims
        ],
        "missing_named_collider_paths": missing_named_paths,
        "binding_scope": binding_scope,
        "collider_discovery_ok": bool(collision_prims),
        "bound_collider_count": len(bound_paths),
        "bound_colliders": bound_paths,
        "ok": bool(binding_prims),
    }


def main() -> int:
    args = parse_args()
    config_path = resolve_path(args.config, THIS_DIR)
    urdf_path = resolve_path(args.urdf, THIS_DIR)
    profile = load_profile(config_path)
    foot_collision_defs = load_foot_collision_defs(urdf_path)

    usd_input = args.usd
    if usd_input == DEFAULT_USD_PATH and profile.get("robot_usd"):
        usd_input = Path(profile["robot_usd"])

    usd_path = resolve_path(usd_input, config_path.parent)
    stage, stage_source = open_stage(usd_path)
    if stage is None:
        print(f"Failed to open USD stage: {usd_path}", file=sys.stderr)
        return 1

    matched_joints, all_revolute = find_hexbot_joints(stage)
    expected_count = int(profile.get("expected_joint_count", 18))
    if len(matched_joints) != expected_count:
        print(
            "Could not resolve the expected revolute joints for hexbot.",
            file=sys.stderr,
        )
        print(f"Expected {expected_count}, found {len(matched_joints)}.", file=sys.stderr)
        print("Detected revolute joints:", file=sys.stderr)
        for name in all_revolute:
            print(f"  - {name}", file=sys.stderr)
        return 2

    target_overrides = profile.get("joint_targets_deg", {})
    summaries = []
    for joint_name in sorted(matched_joints):
        match = JOINT_NAME_PATTERN.match(joint_name)
        if match is None:
            continue

        group_name = match.group("group").lower()
        gains = profile["joint_groups"][group_name]
        summaries.append(
            apply_joint_drive(
                matched_joints[joint_name], gains, joint_name, target_overrides
            )
        )

    material_summary = apply_foot_material(
        stage, profile.get("foot_material", {}), foot_collision_defs
    )

    print(f"Resolved {len(summaries)} leg joints in {usd_path} (stageSource={stage_source}):")
    for item in summaries:
        print(
            f"  - {item['joint']}: path={item['prim_path']}, "
            f"Kp={item['stiffness']}, Kd={item['damping']}, "
            f"maxForce={item['max_force']}, "
            f"maxVel={item['max_joint_velocity']}, "
            f"targetDeg={item['target_position_deg']}"
        )

    if material_summary is not None:
        if not material_summary["ok"]:
            print("Failed to discover foot prims for material binding.", file=sys.stderr)
            print(
                f"footRelatedPrimCount={material_summary['foot_related_prim_count']}",
                file=sys.stderr,
            )
            for prim_path in material_summary["foot_related_prim_examples"]:
                print(f"  - {prim_path}", file=sys.stderr)
            print(
                f"footGeomPrimCount={material_summary['foot_geom_prim_count']}",
                file=sys.stderr,
            )
            for prim_path in material_summary["foot_geom_prim_examples"]:
                print(f"  - geom {prim_path}", file=sys.stderr)
            return 3
        print("Applied foot physics material:")
        print(
            f"  - material={material_summary['material_path']}, "
            f"staticFriction={material_summary['static_friction']}, "
            f"dynamicFriction={material_summary['dynamic_friction']}, "
            f"restitution={material_summary['restitution']}, "
            f"boundColliders={material_summary['bound_collider_count']}, "
            f"bindingScope={material_summary['binding_scope']}"
        )
        if material_summary["missing_named_collider_paths"]:
            print("  - missingNamedColliders:")
            for collider_path in material_summary["missing_named_collider_paths"]:
                print(f"    * {collider_path}")
        for collider_path in material_summary["bound_colliders"]:
            print(f"    * {collider_path}")

    if args.dry_run:
        print("Dry run enabled, USD was not saved.")
        return 0

    stage.GetRootLayer().Save()
    print(f"Saved drive overrides into {stage.GetRootLayer().identifier}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
