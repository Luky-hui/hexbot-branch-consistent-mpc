#!/usr/bin/env python3
"""Re-import hexbot URDF into USD with collision-preserving Isaac Sim settings.

Run this with Isaac Sim's bundled Python:

    ./python.sh reimport_hexbot_urdf_asset.py

The script uses Isaac Sim's official URDF importer commands and then verifies
that foot-related geometry and collision candidates actually exist in the
generated USD. This is intended for repairing the current live-stage asset
where ``foot_*`` links exist only as placeholder Xforms.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics

THIS_DIR = Path(__file__).resolve().parent
DEFAULT_URDF_PATH = THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted.urdf"
DEFAULT_OUTPUT_PATH = (
    THIS_DIR.parent
    / "urdf"
    / "hexbot_isaac_rooted_reimport"
    / "hexbot_isaac_rooted_reimport.usd"
)
FOOT_NAME_TOKENS = tuple(
    f"foot_{leg}".lower() for leg in ("LF", "LM", "LR", "RF", "RM", "RR")
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Re-import the hexbot URDF with Isaac Sim official importer settings "
            "that preserve fixed-joint foot collision geometry."
        )
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF_PATH,
        help="Absolute or relative path to the source URDF.",
    )
    parser.add_argument(
        "--usd-out",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help="Output USD path for the repaired asset.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional JSON summary path.",
    )
    parser.add_argument(
        "--merge-fixed-joints",
        action="store_true",
        help=(
            "Enable merge_fixed_joints. Default is disabled because the current "
            "repair path tries to preserve explicit foot links and colliders."
        ),
    )
    parser.add_argument(
        "--collision-from-visuals",
        action="store_true",
        help=(
            "Generate collisions from visuals. Default is disabled because the "
            "hexbot URDF already authors explicit collision geometry."
        ),
    )
    parser.add_argument(
        "--convex-decomp",
        action="store_true",
        help="Enable convex decomposition when generating collisions from visuals.",
    )
    parser.add_argument(
        "--fix-base",
        action="store_true",
        help="Create a fixed joint for the base link during import.",
    )
    parser.add_argument(
        "--self-collision",
        action="store_true",
        help="Enable articulation self-collision during import.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the import config and exit without importing.",
    )
    return parser.parse_args()


def resolve_path(path: Path, base_dir: Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


def set_import_option(import_config: Any, method_name: str, property_name: str, value: Any) -> None:
    setter = getattr(import_config, method_name, None)
    if callable(setter):
        setter(value)
        return

    if hasattr(import_config, property_name):
        setattr(import_config, property_name, value)
        return

    raise AttributeError(
        f"ImportConfig does not expose {method_name}() or {property_name}."
    )


def is_foot_related_path(path_text: str) -> bool:
    lower_path = path_text.lower()
    return any(token in lower_path for token in FOOT_NAME_TOKENS)


def collision_enabled(prim: Usd.Prim) -> bool | None:
    attr = prim.GetAttribute("physics:collisionEnabled")
    if not attr or not attr.IsValid():
        return None
    value = attr.Get()
    if value is None:
        return None
    return bool(value)


def summarize_stage(stage: Usd.Stage) -> dict[str, Any]:
    foot_related_prims = [
        prim for prim in stage.Traverse() if is_foot_related_path(prim.GetPath().pathString)
    ]
    foot_geom_prims = [prim for prim in foot_related_prims if prim.IsA(UsdGeom.Gprim)]
    foot_collision_prims = [
        prim
        for prim in foot_geom_prims
        if prim.HasAPI(UsdPhysics.CollisionAPI)
        or collision_enabled(prim) is True
    ]

    return {
        "foot_related_prim_count": len(foot_related_prims),
        "foot_related_prim_examples": [
            prim.GetPath().pathString for prim in foot_related_prims[:20]
        ],
        "foot_geom_prim_count": len(foot_geom_prims),
        "foot_geom_prim_examples": [
            prim.GetPath().pathString for prim in foot_geom_prims[:20]
        ],
        "foot_collision_prim_count": len(foot_collision_prims),
        "foot_collision_prim_examples": [
            prim.GetPath().pathString for prim in foot_collision_prims[:20]
        ],
    }


def main() -> int:
    args = parse_args()
    urdf_path = resolve_path(args.urdf, THIS_DIR)
    usd_out = resolve_path(args.usd_out, THIS_DIR)
    usd_out.parent.mkdir(parents=True, exist_ok=True)

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status or import_config is None:
        print("Failed to create Isaac Sim URDF ImportConfig.", file=sys.stderr)
        return 1

    # Recommended repair defaults for the current hexbot issue:
    # keep fixed foot links explicit, use authored collision spheres, and avoid
    # introducing additional approximations before the asset is structurally valid.
    set_import_option(import_config, "set_distance_scale", "distance_scale", 1.0)
    set_import_option(import_config, "set_make_default_prim", "make_default_prim", True)
    set_import_option(
        import_config, "set_create_physics_scene", "create_physics_scene", False
    )
    set_import_option(import_config, "set_import_inertia_tensor", "import_inertia_tensor", True)
    set_import_option(import_config, "set_fix_base", "fix_base", bool(args.fix_base))
    set_import_option(
        import_config,
        "set_merge_fixed_joints",
        "merge_fixed_joints",
        bool(args.merge_fixed_joints),
    )
    set_import_option(
        import_config,
        "set_collision_from_visuals",
        "collision_from_visuals",
        bool(args.collision_from_visuals),
    )
    set_import_option(
        import_config, "set_convex_decomp", "convex_decomp", bool(args.convex_decomp)
    )
    set_import_option(
        import_config, "set_self_collision", "self_collision", bool(args.self_collision)
    )
    set_import_option(import_config, "set_density", "density", 0.0)

    config_summary = {
        "urdf_path": str(urdf_path),
        "usd_out": str(usd_out),
        "merge_fixed_joints": bool(args.merge_fixed_joints),
        "collision_from_visuals": bool(args.collision_from_visuals),
        "convex_decomp": bool(args.convex_decomp),
        "fix_base": bool(args.fix_base),
        "self_collision": bool(args.self_collision),
    }

    print("Prepared Isaac Sim URDF import config:")
    for key, value in config_summary.items():
        print(f"  - {key}={value}")

    if args.dry_run:
        print("Dry run enabled, import not executed.")
        return 0

    status, imported_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        dest_path=str(usd_out),
    )
    if not status:
        print(f"URDFParseAndImportFile failed for {urdf_path}.", file=sys.stderr)
        return 2

    stage = Usd.Stage.Open(str(usd_out))
    if stage is None:
        print(f"Imported USD could not be opened: {usd_out}", file=sys.stderr)
        return 3

    stage_summary = summarize_stage(stage)
    report = {
        "imported_prim_path": imported_prim_path,
        "config": config_summary,
        "stage_summary": stage_summary,
    }

    print("")
    print(f"Import result prim path: {imported_prim_path}")
    print(f"Imported USD: {usd_out}")
    print(
        "Foot summary: "
        f"footRelatedPrimCount={stage_summary['foot_related_prim_count']}, "
        f"footGeomPrimCount={stage_summary['foot_geom_prim_count']}, "
        f"footCollisionPrimCount={stage_summary['foot_collision_prim_count']}"
    )
    if stage_summary["foot_geom_prim_examples"]:
        print("Foot geometry prim examples:")
        for prim_path in stage_summary["foot_geom_prim_examples"]:
            print(f"  - {prim_path}")
    else:
        print("No foot geometry prims discovered in the imported USD.")
        for prim_path in stage_summary["foot_related_prim_examples"]:
            print(f"  - foot-related prim: {prim_path}")

    if args.json_out is not None:
        json_out = resolve_path(args.json_out, Path.cwd())
        json_out.parent.mkdir(parents=True, exist_ok=True)
        json_out.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(f"JSON report written to {json_out}")

    if stage_summary["foot_geom_prim_count"] == 0:
        print(
            "Imported USD still has no foot geometry prims. Asset repair remains blocked.",
            file=sys.stderr,
        )
        return 4

    if stage_summary["foot_collision_prim_count"] == 0:
        print(
            "Imported USD has foot geometry but no explicit foot collision candidates.",
            file=sys.stderr,
        )
        return 5

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
