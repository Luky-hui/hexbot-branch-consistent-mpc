#!/usr/bin/env python3
"""Shared URDF foot-collider metadata helpers for hexbot Isaac tooling."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


FOOT_LEGS = ("LF", "LM", "LR", "RF", "RM", "RR")
DEFAULT_COLLISION_CHILD_NAME = "generated_collision_sphere"


def parse_xyz(value: str | None) -> tuple[float, float, float]:
    if not value:
        return (0.0, 0.0, 0.0)
    parts = [float(item) for item in value.split()]
    if len(parts) != 3:
        raise ValueError(f"Expected xyz triple, got: {value}")
    return (parts[0], parts[1], parts[2])


def parse_rpy(value: str | None) -> tuple[float, float, float]:
    if not value:
        return (0.0, 0.0, 0.0)
    parts = [float(item) for item in value.split()]
    if len(parts) != 3:
        raise ValueError(f"Expected rpy triple, got: {value}")
    return (parts[0], parts[1], parts[2])


def load_foot_collision_defs(urdf_path: Path) -> dict[str, dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    foot_defs: dict[str, dict[str, Any]] = {}
    for leg in FOOT_LEGS:
        link_name = f"foot_{leg}"
        link = root.find(f"./link[@name='{link_name}']")
        if link is None:
            continue
        collision = link.find("./collision")
        if collision is None:
            continue
        origin = collision.find("./origin")
        geometry = collision.find("./geometry")
        sphere = geometry.find("./sphere") if geometry is not None else None
        if sphere is None:
            continue
        foot_defs[link_name] = {
            "collision_name": collision.attrib.get(
                "name", DEFAULT_COLLISION_CHILD_NAME
            ),
            "radius": float(sphere.attrib["radius"]),
            "xyz": parse_xyz(origin.attrib.get("xyz") if origin is not None else None),
            "rpy": parse_rpy(origin.attrib.get("rpy") if origin is not None else None),
        }
    return foot_defs
