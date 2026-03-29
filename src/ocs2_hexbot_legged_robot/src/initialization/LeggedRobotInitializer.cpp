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

#include "ocs2_hexbot_legged_robot/initialization/LeggedRobotInitializer.h"

#include "ocs2_hexbot_legged_robot/common/utils.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer::LeggedRobotInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager,
                                               ModelSettings modelSettings, bool extendNormalizedMomentum)
    : info_(std::move(info)),
      referenceManagerPtr_(&referenceManager),
      modelSettings_(std::move(modelSettings)),
      extendNormalizedMomentum_(extendNormalizedMomentum) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer* LeggedRobotInitializer::clone() const {
  return new LeggedRobotInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  const auto footPositionsInWorld = referenceManagerPtr_->getReferenceFootPositionsInWorld(time, &state);
  const auto supportPlane = referenceManagerPtr_->getSupportPlane(state, time);
  const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
  const auto xNominal = targetTrajectories.getDesiredState(time);
  const auto bodyPositionInWorld = approximateBasePositionInWorld(info_, xNominal);

  const vector3_t desiredHorizontalForce = computeDesiredHorizontalForce(
      info_, xNominal, state, modelSettings_.propulsionForceResponseTime, modelSettings_.propulsionForceScale,
      modelSettings_.propulsionForceMaxAcceleration, modelSettings_.movingForceDeadzoneVelocity);

  input = weightCompensatingInput(info_, contactFlags, footPositionsInWorld, bodyPositionInWorld, supportPlane, desiredHorizontalForce);
  nextState = state;
  if (!extendNormalizedMomentum_) {
    centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
  }
}

}  // namespace legged_robot
}  // namespace ocs2
