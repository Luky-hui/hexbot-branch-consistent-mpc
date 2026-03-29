#!/usr/bin/env python3
"""Audit Isaac Sim joint-drive and foot-material settings for hexbot.

Run this with Isaac Sim's bundled Python so the ``pxr`` bindings are available:

    ./python.sh audit_hexbot_isaac_setup.py

The script opens the exported USD, verifies the 18 revolute leg joints, checks
that the expected drive attributes are present and match the configured profile,
and confirms that the six foot colliders are bound to the expected high-friction
physics material.
"""

from __future__ import annotations

import argparse
import json
import math
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
        description="Audit Isaac Sim drive and foot material settings for hexbot."
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=DEFAULT_USD_PATH,
        help="Path to the hexbot USD asset to inspect.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to the JSON profile describing expected drive gains.",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF_PATH,
        help="URDF used to resolve expected foot collision child names.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional JSON report output path.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return a non-zero exit code when any audit item fails.",
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
        # Keep the explicit API/attribute checks first, but still allow generic
        # foot-related Gprims as a fallback binding/audit candidate.
        if prim.IsA(UsdGeom.Gprim):
            fallback_prims.append(prim)

    if matched_prims:
        return matched_prims
    return fallback_prims


def audit_named_foot_colliders(
    stage: Usd.Stage, foot_collision_defs: dict[str, dict[str, Any]]
) -> dict[str, Any]:
    colliders: list[dict[str, Any]] = []
    missing_roots: list[str] = []
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
            missing_roots.append(link_name)
            continue

        foot_roots.sort(key=lambda prim: len(prim.GetPath().pathString))
        foot_root = foot_roots[0]
        expected_path = foot_root.GetPath().AppendChild(collision_def["collision_name"])
        expected_prim = stage.GetPrimAtPath(expected_path)
        if not expected_prim or not expected_prim.IsValid():
            missing_paths.append(expected_path.pathString)
            continue

        collision_attr = expected_prim.GetAttribute("physics:collisionEnabled")
        collision_enabled = (
            bool(collision_attr)
            and collision_attr.IsValid()
            and bool(collision_attr.Get())
        )
        colliders.append(
            {
                "foot_root": foot_root.GetPath().pathString,
                "prim_path": expected_path.pathString,
                "is_gprim": expected_prim.IsA(UsdGeom.Gprim),
                "has_collision_api": expected_prim.HasAPI(UsdPhysics.CollisionAPI),
                "collision_enabled": collision_enabled,
                "binding_targets": get_material_binding_targets(expected_prim),
            }
        )

    return {
        "expected_count": len(FOOT_LEGS),
        "resolved_count": len(colliders),
        "missing_roots": missing_roots,
        "missing_paths": missing_paths,
        "colliders": colliders,
        "ok": not missing_roots and not missing_paths and len(colliders) == len(FOOT_LEGS),
    }


def approx_equal(lhs: float | None, rhs: float | None, tolerance: float = 1.0e-4) -> bool:
    if lhs is None or rhs is None:
        return lhs is None and rhs is None
    return math.isclose(float(lhs), float(rhs), abs_tol=tolerance, rel_tol=0.0)


def read_attr(prim: Usd.Prim, attr_name: str) -> Any:
    attr = prim.GetAttribute(attr_name)
    if not attr or not attr.IsValid():
        return None
    return attr.Get()


def normalize_number(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def audit_joint(
    joint_prim: Usd.Prim,
    joint_name: str,
    gains: dict[str, Any],
    target_overrides: dict[str, float],
) -> dict[str, Any]:
    expected_target = float(
        target_overrides.get(joint_name, gains.get("target_position_deg", 0.0))
    )
    expected = {
        "stiffness": float(gains["stiffness"]),
        "damping": float(gains["damping"]),
        "max_force": float(gains["max_force"]),
        "max_joint_velocity": float(gains["max_joint_velocity"]),
        "target_position_deg": expected_target,
    }

    observed = {
        "stiffness": normalize_number(
            read_attr(joint_prim, "drive:angular:physics:stiffness")
        ),
        "damping": normalize_number(
            read_attr(joint_prim, "drive:angular:physics:damping")
        ),
        "max_force": normalize_number(
            read_attr(joint_prim, "drive:angular:physics:maxForce")
        ),
        "max_joint_velocity": normalize_number(
            read_attr(joint_prim, "physxJoint:maxJointVelocity")
        ),
        "target_position_deg": normalize_number(
            read_attr(joint_prim, "drive:angular:physics:targetPosition")
        ),
        "drive_type": read_attr(joint_prim, "drive:angular:physics:type"),
    }

    mismatches: list[str] = []
    for field_name, expected_value in expected.items():
        if not approx_equal(observed[field_name], expected_value):
            mismatches.append(field_name)

    expected_drive_type = gains.get("drive_type")
    if expected_drive_type is not None and observed["drive_type"] != expected_drive_type:
        mismatches.append("drive_type")

    return {
        "joint": joint_name,
        "group": JOINT_NAME_PATTERN.match(joint_name).group("group").lower(),
        "prim_path": joint_prim.GetPath().pathString,
        "expected": expected,
        "observed": observed,
        "mismatches": mismatches,
        "ok": not mismatches,
    }


def summarize_joint_results(results: list[dict[str, Any]]) -> dict[str, Any]:
    by_group: dict[str, dict[str, Any]] = {}
    for result in results:
        group_name = result["group"]
        if group_name not in by_group:
            by_group[group_name] = {"total": 0, "passed": 0, "failed_joints": []}
        by_group[group_name]["total"] += 1
        if result["ok"]:
            by_group[group_name]["passed"] += 1
        else:
            by_group[group_name]["failed_joints"].append(
                {"joint": result["joint"], "mismatches": result["mismatches"]}
            )

    return {
        "all_ok": all(result["ok"] for result in results),
        "joint_count": len(results),
        "groups": by_group,
    }


def get_material_binding_targets(prim: Usd.Prim) -> list[str]:
    targets: list[str] = []
    for rel_name in ("material:binding:physics", "material:binding"):
        relationship = prim.GetRelationship(rel_name)
        if relationship and relationship.IsValid():
            targets.extend(str(path) for path in relationship.GetTargets())
    return targets


def audit_foot_material(
    stage: Usd.Stage,
    material_config: dict[str, Any],
    named_collider_summary: dict[str, Any],
) -> dict[str, Any]:
    expected_material_path = resolve_stage_path(
        stage, material_config.get("path", "PhysicsMaterials/hexbotFootPhysicsMaterial")
    )
    material_prim = stage.GetPrimAtPath(expected_material_path)
    material_exists = bool(material_prim and material_prim.IsValid())

    observed_material = {
        "static_friction": normalize_number(
            read_attr(material_prim, "physics:staticFriction") if material_exists else None
        ),
        "dynamic_friction": normalize_number(
            read_attr(material_prim, "physics:dynamicFriction") if material_exists else None
        ),
        "restitution": normalize_number(
            read_attr(material_prim, "physics:restitution") if material_exists else None
        ),
    }
    expected_material = {
        "static_friction": float(material_config["static_friction"]),
        "dynamic_friction": float(material_config["dynamic_friction"]),
        "restitution": float(material_config["restitution"]),
    }

    material_mismatches: list[str] = []
    if not material_exists:
        material_mismatches.append("missing_material_prim")
    else:
        for field_name, expected_value in expected_material.items():
            if not approx_equal(observed_material[field_name], expected_value):
                material_mismatches.append(field_name)

    foot_related_prims = find_foot_related_prims(stage)
    foot_geom_prims = find_foot_geom_prims(stage)
    collision_prims = find_foot_collision_prims(stage)
    candidate_prims = collision_prims if collision_prims else foot_geom_prims
    binding_scope = "collision_prims" if collision_prims else "foot_geom_prims"
    collider_results: list[dict[str, Any]] = []
    expected_path_text = expected_material_path.pathString
    for candidate_prim in candidate_prims:
        targets = get_material_binding_targets(candidate_prim)
        bound_ok = expected_path_text in targets
        collision_attr = candidate_prim.GetAttribute("physics:collisionEnabled")
        collision_enabled = (
            bool(collision_attr)
            and collision_attr.IsValid()
            and bool(collision_attr.Get())
        )
        collider_results.append(
            {
                "prim_path": candidate_prim.GetPath().pathString,
                "binding_targets": targets,
                "is_gprim": candidate_prim.IsA(UsdGeom.Gprim),
                "has_collision_api": candidate_prim.HasAPI(UsdPhysics.CollisionAPI),
                "collision_enabled": collision_enabled,
                "ok": bound_ok,
            }
        )

    collider_discovery_ok = bool(collision_prims)
    foot_geom_ok = bool(foot_geom_prims)
    binding_candidates_ok = bool(candidate_prims)
    return {
        "expected_material_path": expected_path_text,
        "material_exists": material_exists,
        "expected_material": expected_material,
        "observed_material": observed_material,
        "material_mismatches": material_mismatches,
        "foot_related_prim_count": len(foot_related_prims),
        "foot_related_prim_examples": [
            prim.GetPath().pathString for prim in foot_related_prims[:12]
        ],
        "foot_geom_prim_count": len(foot_geom_prims),
        "foot_geom_prim_examples": [
            prim.GetPath().pathString for prim in foot_geom_prims[:12]
        ],
        "binding_scope": binding_scope,
        "bound_collider_count": len(collider_results),
        "foot_geom_ok": foot_geom_ok,
        "binding_candidates_ok": binding_candidates_ok,
        "collider_discovery_ok": collider_discovery_ok,
        "named_collider_summary": named_collider_summary,
        "all_colliders_bound": binding_candidates_ok
        and all(item["ok"] for item in collider_results),
        "colliders": collider_results,
        "ok": (not material_mismatches)
        and binding_candidates_ok
        and all(item["ok"] for item in collider_results),
    }


def print_report(report: dict[str, Any]) -> None:
    print(f"USD: {report['usd_path']}")
    print(f"Profile: {report['profile_path']}")
    print("")
    print(
        f"Joint audit: {report['joint_summary']['joint_count']} joints, "
        f"all_ok={report['joint_summary']['all_ok']}"
    )
    for group_name, group_summary in sorted(report["joint_summary"]["groups"].items()):
        print(
            f"  - {group_name}: {group_summary['passed']}/{group_summary['total']} pass"
        )
        for failed in group_summary["failed_joints"]:
            print(
                f"    * {failed['joint']}: mismatches={', '.join(failed['mismatches'])}"
            )

    foot_summary = report["foot_material"]
    print("")
    print(
        "Foot material audit: "
        f"material_exists={foot_summary['material_exists']}, "
        f"all_colliders_bound={foot_summary['all_colliders_bound']}, "
        f"ok={foot_summary['ok']}"
    )
    print(
        f"  - expected material={foot_summary['expected_material_path']}, "
        f"bound_collider_count={foot_summary['bound_collider_count']}, "
        f"binding_scope={foot_summary['binding_scope']}"
    )
    named_summary = foot_summary["named_collider_summary"]
    print(
        "  - named collider semantics: "
        f"ok={named_summary['ok']}, "
        f"resolved={named_summary['resolved_count']}/{named_summary['expected_count']}"
    )
    for prim_path in named_summary["missing_paths"]:
        print(f"    * missing named collider: {prim_path}")
    for root_name in named_summary["missing_roots"]:
        print(f"    * missing foot root: {root_name}")
    if not foot_summary["foot_geom_ok"]:
        print(
            f"  - no foot geometry prims were discovered; "
            f"foot_related_prim_count={foot_summary['foot_related_prim_count']}"
        )
        for prim_path in foot_summary["foot_related_prim_examples"]:
            print(f"    * foot-related prim: {prim_path}")
    elif not foot_summary["binding_candidates_ok"]:
        print(
            f"  - no foot binding candidates were discovered; "
            f"foot_geom_prim_count={foot_summary['foot_geom_prim_count']}"
        )
        for prim_path in foot_summary["foot_geom_prim_examples"]:
            print(f"    * foot-geom prim: {prim_path}")
    elif not foot_summary["collider_discovery_ok"]:
        print(
            "  - no explicit foot colliders were discovered; "
            "using foot-related prim fallback for binding audit"
        )
    if foot_summary["material_mismatches"]:
        print(
            f"  - material mismatches={', '.join(foot_summary['material_mismatches'])}"
        )
    for collider in foot_summary["colliders"]:
        if not collider["ok"]:
            print(
                f"    * unbound or wrong binding: {collider['prim_path']} -> "
                f"{collider['binding_targets']}"
            )
        elif not collider["has_collision_api"] or not collider["collision_enabled"]:
            print(
                "    * bound but missing explicit collision enable: "
                f"{collider['prim_path']} "
                f"(has_collision_api={collider['has_collision_api']}, "
                f"collision_enabled={collider['collision_enabled']}, "
                f"is_gprim={collider['is_gprim']})"
            )

    print("")
    print(f"Overall audit ok={report['all_ok']}")


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

    matched_joints, all_revolute_names = find_hexbot_joints(stage)
    expected_joint_count = int(profile.get("expected_joint_count", 18))

    joint_results: list[dict[str, Any]] = []
    target_overrides = profile.get("joint_targets_deg", {})
    for joint_name in sorted(matched_joints):
        match = JOINT_NAME_PATTERN.match(joint_name)
        if match is None:
            continue
        group_name = match.group("group").lower()
        gains = profile["joint_groups"][group_name]
        joint_results.append(
            audit_joint(matched_joints[joint_name], joint_name, gains, target_overrides)
        )

    joint_summary = summarize_joint_results(joint_results)
    count_ok = len(matched_joints) == expected_joint_count
    if not count_ok:
        joint_summary["all_ok"] = False

    named_collider_summary = audit_named_foot_colliders(stage, foot_collision_defs)
    foot_summary = audit_foot_material(
        stage, profile.get("foot_material", {}), named_collider_summary
    )
    report = {
        "usd_path": str(usd_path),
        "stage_source": stage_source,
        "profile_path": str(config_path),
        "urdf_path": str(urdf_path),
        "expected_joint_count": expected_joint_count,
        "resolved_joint_count": len(matched_joints),
        "resolved_joint_names": sorted(matched_joints.keys()),
        "all_revolute_names": all_revolute_names,
        "joint_count_ok": count_ok,
        "joint_summary": joint_summary,
        "joint_results": joint_results,
        "foot_material": foot_summary,
        "all_ok": count_ok and joint_summary["all_ok"] and foot_summary["ok"],
    }

    print_report(report)

    if args.json_out is not None:
        json_path = args.json_out
        if not json_path.is_absolute():
            json_path = (Path.cwd() / json_path).resolve()
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"JSON report written to {json_path}")

    if args.strict and not report["all_ok"]:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
