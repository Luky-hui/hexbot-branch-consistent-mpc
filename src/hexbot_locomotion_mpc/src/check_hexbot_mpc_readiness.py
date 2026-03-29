#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List


EXPECTED_JOINTS = [
    "coxa_joint_RR",
    "femur_joint_RR",
    "tarsus_joint_RR",
    "coxa_joint_RM",
    "femur_joint_RM",
    "tarsus_joint_RM",
    "coxa_joint_RF",
    "femur_joint_RF",
    "tarsus_joint_RF",
    "coxa_joint_LR",
    "femur_joint_LR",
    "tarsus_joint_LR",
    "coxa_joint_LM",
    "femur_joint_LM",
    "tarsus_joint_LM",
    "coxa_joint_LF",
    "femur_joint_LF",
    "tarsus_joint_LF",
]

EXPECTED_FEET = [
    "foot_LF",
    "foot_LM",
    "foot_LR",
    "foot_RF",
    "foot_RM",
    "foot_RR",
]

FRAME_ONLY_DYNAMIC_LINKS = [
    "leg_center_LF",
    "leg_center_LM",
    "leg_center_LR",
    "leg_center_RF",
    "leg_center_RM",
    "leg_center_RR",
]

ACTUATED_TYPES = {"continuous", "prismatic", "revolute"}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True, help="Path to hexbot URDF")
    return parser.parse_args()


def _safe_float(element: ET.Element | None, attribute: str) -> float | None:
    if element is None:
        return None
    raw_value = element.attrib.get(attribute)
    if raw_value is None:
        return None
    try:
        return float(raw_value)
    except ValueError:
        return None


def _link_has_valid_inertial(link: ET.Element) -> tuple[bool, bool]:
    inertial = link.find("inertial")
    if inertial is None:
        return False, False

    mass = _safe_float(inertial.find("mass"), "value")
    inertia = inertial.find("inertia")
    inertia_terms = []
    for attribute in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        value = _safe_float(inertia, attribute)
        inertia_terms.append(value)

    if mass is None or mass <= 0.0:
        return True, False
    if any(value is None for value in inertia_terms):
        return True, False

    diagonal = [inertia_terms[0], inertia_terms[3], inertia_terms[5]]
    if any(value is None or value <= 0.0 for value in diagonal):
        return True, False
    return True, True


def _collect_links(root: ET.Element) -> Dict[str, ET.Element]:
    return {link.attrib["name"]: link for link in root.findall("link")}


def _collect_dynamic_links(joints: List[ET.Element]) -> List[str]:
    dynamic_links: List[str] = []
    for joint in joints:
        joint_type = joint.attrib.get("type", "")
        if joint_type not in ACTUATED_TYPES:
            continue
        child = joint.find("child")
        parent = joint.find("parent")
        if child is not None:
            name = child.attrib.get("link", "")
            if name and name not in dynamic_links:
                dynamic_links.append(name)
        if parent is not None:
            name = parent.attrib.get("link", "")
            if name in FRAME_ONLY_DYNAMIC_LINKS and name not in dynamic_links:
                dynamic_links.append(name)
    return dynamic_links


def _collect_named_foot_collisions(links: Dict[str, ET.Element]) -> Dict[str, bool]:
    result: Dict[str, bool] = {}
    for foot_name in EXPECTED_FEET:
        link = links.get(foot_name)
        if link is None:
            result[foot_name] = False
            continue
        result[foot_name] = any(
            collision.attrib.get("name") == "generated_collision_sphere"
            for collision in link.findall("collision")
        )
    return result


def main() -> None:
    args = _parse_args()
    urdf_path = Path(args.urdf).expanduser().resolve()
    root = ET.parse(urdf_path).getroot()

    links = _collect_links(root)
    joints = list(root.findall("joint"))

    actuated_joint_names = [
        joint.attrib["name"]
        for joint in joints
        if joint.attrib.get("type", "") in ACTUATED_TYPES
    ]

    missing_expected_joints = [
        name for name in EXPECTED_JOINTS if name not in actuated_joint_names
    ]
    unexpected_actuated_joints = [
        name for name in actuated_joint_names if name not in EXPECTED_JOINTS
    ]

    dynamic_links = _collect_dynamic_links(joints)
    dynamic_links_missing_inertial: List[str] = []
    dynamic_links_invalid_inertial: List[str] = []

    for link_name in dynamic_links:
        if link_name in FRAME_ONLY_DYNAMIC_LINKS:
            continue
        link = links.get(link_name)
        if link is None:
            dynamic_links_missing_inertial.append(link_name)
            continue
        has_inertial, valid_inertial = _link_has_valid_inertial(link)
        if not has_inertial:
            dynamic_links_missing_inertial.append(link_name)
        elif not valid_inertial:
            dynamic_links_invalid_inertial.append(link_name)

    named_foot_collisions = _collect_named_foot_collisions(links)
    named_foot_collision_count = sum(1 for ok in named_foot_collisions.values() if ok)

    result = {
        "urdf_path": str(urdf_path),
        "link_count": len(links),
        "joint_count_total": len(joints),
        "actuated_joint_count": len(actuated_joint_names),
        "expected_actuated_joint_count": len(EXPECTED_JOINTS),
        "missing_expected_joints": missing_expected_joints,
        "unexpected_actuated_joints": unexpected_actuated_joints,
        "dynamic_link_count": len(dynamic_links),
        "frame_only_dynamic_links": [
            name for name in FRAME_ONLY_DYNAMIC_LINKS if name in dynamic_links
        ],
        "dynamic_links_missing_inertial": dynamic_links_missing_inertial,
        "dynamic_links_invalid_inertial": dynamic_links_invalid_inertial,
        "named_foot_collision_count": named_foot_collision_count,
        "named_foot_collisions": named_foot_collisions,
        "ready": (
            len(actuated_joint_names) == len(EXPECTED_JOINTS)
            and not missing_expected_joints
            and not unexpected_actuated_joints
            and not dynamic_links_missing_inertial
            and not dynamic_links_invalid_inertial
            and named_foot_collision_count == len(EXPECTED_FEET)
        ),
    }

    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
