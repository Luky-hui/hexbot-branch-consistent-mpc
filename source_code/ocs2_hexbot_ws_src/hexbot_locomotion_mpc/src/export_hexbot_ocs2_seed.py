#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path


HEXBOT_MAPPING = """robot_name: hexbot
urdf_path: /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/urdf/hexbot_isaac_rooted.urdf
root_link: base_link

leg_order:
  - RR
  - RM
  - RF
  - LR
  - LM
  - LF

contact_names_3dof:
  - foot_RR
  - foot_RM
  - foot_RF
  - foot_LR
  - foot_LM
  - foot_LF

joint_names:
  - coxa_joint_RR
  - femur_joint_RR
  - tarsus_joint_RR
  - coxa_joint_RM
  - femur_joint_RM
  - tarsus_joint_RM
  - coxa_joint_RF
  - femur_joint_RF
  - tarsus_joint_RF
  - coxa_joint_LR
  - femur_joint_LR
  - tarsus_joint_LR
  - coxa_joint_LM
  - femur_joint_LM
  - tarsus_joint_LM
  - coxa_joint_LF
  - femur_joint_LF
  - tarsus_joint_LF

default_joint_state:
  - 0.0010
  - -0.1308
  - 0.1414
  - -0.0017
  - -0.1672
  - 0.1817
  - -0.0032
  - -0.1990
  - 0.2159
  - 0.0035
  - 0.1043
  - -0.1188
  - 0.0020
  - 0.1362
  - -0.1551
  - -0.0003
  - 0.1726
  - -0.1948

estimated_com_height_m: 0.126288
target_displacement_velocity_mps: 0.15
target_rotation_velocity_rps: 0.60
"""

REFERENCE_INFO = """targetDisplacementVelocity          0.150;
targetRotationVelocity              0.600;

comHeight                           0.126288

defaultJointState
{
   (0,0)   0.001000   ; coxa_joint_RR
   (1,0)  -0.130800   ; femur_joint_RR
   (2,0)   0.141400   ; tarsus_joint_RR
   (3,0)  -0.001700   ; coxa_joint_RM
   (4,0)  -0.167200   ; femur_joint_RM
   (5,0)   0.181700   ; tarsus_joint_RM
   (6,0)  -0.003200   ; coxa_joint_RF
   (7,0)  -0.199000   ; femur_joint_RF
   (8,0)   0.215900   ; tarsus_joint_RF
   (9,0)   0.003500   ; coxa_joint_LR
   (10,0)  0.104300   ; femur_joint_LR
   (11,0) -0.118800   ; tarsus_joint_LR
   (12,0)  0.002000   ; coxa_joint_LM
   (13,0)  0.136200   ; femur_joint_LM
   (14,0) -0.155100   ; tarsus_joint_LM
   (15,0) -0.000300   ; coxa_joint_LF
   (16,0)  0.172600   ; femur_joint_LF
   (17,0) -0.194800   ; tarsus_joint_LF
}

initialModeSchedule
{
  modeSequence
  {
    [0]  STANCE
    [1]  STANCE
  }
  eventTimes
  {
    [0]  0.5
  }
}

defaultModeSequenceTemplate
{
  modeSequence
  {
    [0]  STANCE
  }
  switchingTimes
  {
    [0]  0.0
    [1]  1.0
  }
}
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parents[1] / "config" / "ocs2_hexbot"),
        help="Directory that will receive hexbot_mapping.yaml and reference.info",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    mapping_path = output_dir / "hexbot_mapping.yaml"
    reference_path = output_dir / "reference.info"

    mapping_path.write_text(HEXBOT_MAPPING, encoding="utf-8")
    reference_path.write_text(REFERENCE_INFO, encoding="utf-8")

    print(f"wrote {mapping_path}")
    print(f"wrote {reference_path}")


if __name__ == "__main__":
    main()
