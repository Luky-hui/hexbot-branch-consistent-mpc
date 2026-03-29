#!/usr/bin/env python3
"""Inspect foot-related prim hierarchy and collision/material state in Isaac Sim.

Run this with Isaac Sim's bundled Python so the ``pxr`` and ``omni`` bindings
are available:

    ./python.sh inspect_hexbot_foot_prims.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
from urllib.parse import unquote, urlparse

from pxr import Usd, UsdGeom, UsdPhysics

try:
    import omni.usd  # type: ignore
except ImportError:  # pragma: no cover - only available inside Isaac Sim
    omni = None

from foot_collision_defs import load_foot_collision_defs


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_URDF_PATH = THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted.urdf"
DEFAULT_USD_PATH = (
    THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted" / "hexbot_isaac_rooted.usd"
)
FOOT_NAME_TOKENS = tuple(
    f"foot_{leg}".lower() for leg in ("LF", "LM", "LR", "RF", "RM", "RR")
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect foot-related prim hierarchy inside the loaded Isaac stage."
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=DEFAULT_USD_PATH,
        help="Target USD path used to match the live stage.",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF_PATH,
        help="URDF used to resolve expected foot collision child names.",
    )
    parser.add_argument(
        "--max-depth",
        type=int,
        default=2,
        help="Maximum descendant depth to print under each foot prim.",
    )
    return parser.parse_args()


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
            target_identifier = normalize_layer_identifier(str(usd_path.resolve()))
            if live_identifier == target_identifier:
                return live_stage, "live"

    return Usd.Stage.Open(str(usd_path.resolve())), "file"


def resolve_path(path: Path, base_dir: Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path.resolve()
    return (base_dir / path).resolve()


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


def applied_schemas(prim: Usd.Prim) -> list[str]:
    try:
        return list(prim.GetAppliedSchemas())
    except Exception:
        return []


def material_targets(prim: Usd.Prim) -> list[str]:
    targets: list[str] = []
    for rel_name in ("material:binding:physics", "material:binding"):
        relationship = prim.GetRelationship(rel_name)
        if relationship and relationship.IsValid():
            targets.extend(str(path) for path in relationship.GetTargets())
    return targets


def print_prim_info(prim: Usd.Prim, depth: int) -> None:
    indent = "  " * depth
    print(f"{indent}- path={prim.GetPath().pathString}")
    print(f"{indent}  type={prim.GetTypeName() or '<untyped>'}")
    print(f"{indent}  is_gprim={prim.IsA(UsdGeom.Gprim)}")
    print(f"{indent}  has_collision_api={prim.HasAPI(UsdPhysics.CollisionAPI)}")
    print(f"{indent}  collision_enabled={collision_enabled(prim)}")
    schemas = applied_schemas(prim)
    print(f"{indent}  applied_schemas={schemas if schemas else '[]'}")
    targets = material_targets(prim)
    if targets:
        print(f"{indent}  material_targets={targets}")


def print_descendants(prim: Usd.Prim, max_depth: int, current_depth: int = 0) -> None:
    if current_depth > max_depth:
        return
    print_prim_info(prim, current_depth)
    if current_depth == max_depth:
        return
    for child in prim.GetChildren():
        print_descendants(child, max_depth, current_depth + 1)


def has_descendant_gprim(prim: Usd.Prim) -> bool:
    for child in Usd.PrimRange(prim):
        if child.GetPath() == prim.GetPath():
            continue
        if child.IsA(UsdGeom.Gprim):
            return True
    return False


def main() -> int:
    args = parse_args()
    urdf_path = resolve_path(args.urdf, THIS_DIR)
    foot_collision_defs = load_foot_collision_defs(urdf_path)
    stage, stage_source = open_stage(args.usd)
    if stage is None:
        print(f"Failed to open stage for {args.usd}")
        return 1

    print(f"stageSource={stage_source}")
    print(f"rootLayer={stage.GetRootLayer().identifier}")
    print(f"urdf={urdf_path}")

    foot_prims = [
        prim
        for prim in stage.Traverse()
        if is_foot_related_path(prim.GetPath().pathString)
        and prim.GetName().lower().startswith("foot_")
    ]
    unique_paths: dict[str, Usd.Prim] = {}
    for prim in foot_prims:
        unique_paths[prim.GetPath().pathString] = prim

    print(f"footPrimCount={len(unique_paths)}")
    missing_named_paths: list[str] = []
    for prim_path in sorted(unique_paths):
        print("")
        print(f"Foot root: {prim_path}")
        foot_prim = unique_paths[prim_path]
        collision_def = foot_collision_defs.get(foot_prim.GetName())
        if collision_def:
            expected_path = foot_prim.GetPath().AppendChild(collision_def["collision_name"])
            expected_prim = stage.GetPrimAtPath(expected_path)
            expected_found = bool(expected_prim and expected_prim.IsValid())
            print(
                "  expected_named_collider="
                f"{expected_path.pathString} found={expected_found}"
            )
            if not expected_found:
                missing_named_paths.append(expected_path.pathString)
        print_descendants(foot_prim, args.max_depth)

        if not foot_prim.IsA(UsdGeom.Gprim) and not has_descendant_gprim(foot_prim):
            parent_prim = foot_prim.GetParent()
            if parent_prim and parent_prim.IsValid():
                print("")
                print(
                    "  Parent subtree context "
                    f"(foot root has no Gprim descendants): {parent_prim.GetPath().pathString}"
                )
                print_descendants(parent_prim, args.max_depth + 1)

    if missing_named_paths:
        print("")
        print("Missing expected named colliders:")
        for prim_path in missing_named_paths:
            print(f"  - {prim_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
