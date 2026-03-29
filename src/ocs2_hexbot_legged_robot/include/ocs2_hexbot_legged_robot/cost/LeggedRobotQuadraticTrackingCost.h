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

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include "ocs2_hexbot_legged_robot/common/ModelSettings.h"
#include "ocs2_hexbot_legged_robot/common/utils.h"
#include "ocs2_hexbot_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/**
 * State-input tracking cost used for intermediate times
 */
class LeggedRobotStateInputQuadraticCost final : public QuadraticStateInputCost {
 public:
  LeggedRobotStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info, ModelSettings modelSettings,
                                     const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateInputCost(std::move(Q), std::move(R)),
        info_(std::move(info)),
        modelSettings_(std::move(modelSettings)),
        referenceManagerPtr_(&referenceManager) {}

  ~LeggedRobotStateInputQuadraticCost() override = default;
  LeggedRobotStateInputQuadraticCost* clone() const override { return new LeggedRobotStateInputQuadraticCost(*this); }

 private:
  LeggedRobotStateInputQuadraticCost(const LeggedRobotStateInputQuadraticCost& rhs) = default;

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const vector_t xNominal = targetTrajectories.getDesiredState(time);
    const auto footPositionsInWorld = referenceManagerPtr_->getReferenceFootPositionsInWorld(time, &state);
    const auto supportPlane = referenceManagerPtr_->getSupportPlane(state, time);
    const auto bodyPositionInWorld = approximateBasePositionInWorld(info_, xNominal);
    const vector3_t desiredHorizontalForce = computeDesiredHorizontalForce(
        info_, xNominal, state, modelSettings_.propulsionForceResponseTime, modelSettings_.propulsionForceScale,
        modelSettings_.propulsionForceMaxAcceleration, modelSettings_.movingForceDeadzoneVelocity);

    const vector_t uNominal = weightCompensatingInput(info_, contactFlags, footPositionsInWorld, bodyPositionInWorld, supportPlane,
                                                      desiredHorizontalForce);
    vector_t inputDeviation = input - uNominal;

    const scalar_t desiredSpeed = xNominal.head<2>().norm();
    const scalar_t tangentialPenaltyScale =
        std::clamp(modelSettings_.movingTangentialForcePenaltyScale, scalar_t(1.0e-4), scalar_t(1.0));
    if (desiredSpeed > modelSettings_.movingForceDeadzoneVelocity && tangentialPenaltyScale < 1.0) {
      const scalar_t tangentialDeviationScale = std::sqrt(tangentialPenaltyScale);
      for (size_t contactIndex = 0; contactIndex < info_.numThreeDofContacts; ++contactIndex) {
        auto contactForceDeviation = centroidal_model::getContactForces(inputDeviation, contactIndex, info_);
        vector3_t contactForceDeviationInTerrain = supportPlane.worldToTerrainRotation * contactForceDeviation;
        contactForceDeviationInTerrain.x() *= tangentialDeviationScale;
        contactForceDeviationInTerrain.y() *= tangentialDeviationScale;
        contactForceDeviation = supportPlane.worldToTerrainRotation.transpose() * contactForceDeviationInTerrain;
      }
    }

    return {state - xNominal, inputDeviation};
  }

  const CentroidalModelInfo info_;
  const ModelSettings modelSettings_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};

/**
 * State tracking cost used for the final time
 */
class LeggedRobotStateQuadraticCost final : public QuadraticStateCost {
 public:
  LeggedRobotStateQuadraticCost(matrix_t Q, CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateCost(std::move(Q)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~LeggedRobotStateQuadraticCost() override = default;
  LeggedRobotStateQuadraticCost* clone() const override { return new LeggedRobotStateQuadraticCost(*this); }

 private:
  LeggedRobotStateQuadraticCost(const LeggedRobotStateQuadraticCost& rhs) = default;

  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const vector_t xNominal = targetTrajectories.getDesiredState(time);
    return state - xNominal;
  }

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
