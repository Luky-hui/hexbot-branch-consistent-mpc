/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot {

struct ModelSettings {
  scalar_t positionErrorGain = 0.0;
  scalar_t propulsionForceScale = 1.0;
  scalar_t propulsionForceResponseTime = 0.15;
  scalar_t propulsionForceMaxAcceleration = 0.6;
  scalar_t movingTangentialForcePenaltyScale = 0.08;
  scalar_t movingForceDeadzoneVelocity = 0.005;

  scalar_t phaseTransitionStanceTime = 0.4;

  bool verboseCppAd = true;
  bool recompileLibrariesCppAd = true;
  std::string modelFolderCppAd = "/tmp/ocs2_hexbot";

  std::vector<std::string> jointNames{
      "coxa_joint_RR", "femur_joint_RR", "tarsus_joint_RR",
      "coxa_joint_RM", "femur_joint_RM", "tarsus_joint_RM",
      "coxa_joint_RF", "femur_joint_RF", "tarsus_joint_RF",
      "coxa_joint_LR", "femur_joint_LR", "tarsus_joint_LR",
      "coxa_joint_LM", "femur_joint_LM", "tarsus_joint_LM",
      "coxa_joint_LF", "femur_joint_LF", "tarsus_joint_LF"};
  std::vector<std::string> contactNames6DoF{};
  std::vector<std::string> contactNames3DoF{
      "foot_RR", "foot_RM", "foot_RF", "foot_LR", "foot_LM", "foot_LF"};
};

ModelSettings loadModelSettings(const std::string& filename, const std::string& fieldName = "model_settings", bool verbose = "true");

}  // namespace legged_robot
}  // namespace ocs2
