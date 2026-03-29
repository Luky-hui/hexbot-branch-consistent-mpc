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

#include "ocs2_hexbot_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

#include <algorithm>
#include <cmath>

#include <ocs2_core/misc/Lookup.h>

namespace ocs2 {
namespace legged_robot {

namespace {

scalar_t wrapAngleToPi(scalar_t angle) {
  constexpr scalar_t kPi = 3.14159265358979323846;
  constexpr scalar_t kTwoPi = 2.0 * kPi;
  scalar_t wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                             std::vector<std::string> contactNames3DoF)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      contactNames3DoF_(std::move(contactNames3DoF)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<vector3_t> SwitchedModelReferenceManager::getFootPositionsInWorld(const vector_t& state) const {
  return computeFootPositionsInWorld(const_cast<PinocchioInterface&>(pinocchioInterface_), info_, contactNames3DoF_, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SwitchedModelReferenceManager::getReferenceState(scalar_t time, const vector_t* fallbackState) const {
  const auto isUsableState = [&](const vector_t& candidate) {
    return candidate.size() == static_cast<Eigen::Index>(info_.stateDim) && candidate.array().isFinite().all();
  };

  const auto& targetTrajectories = this->getTargetTrajectories();
  if (!targetTrajectories.empty()) {
    const auto referenceState = targetTrajectories.getDesiredState(time);
    if (isUsableState(referenceState)) {
      return referenceState;
    }
  }

  if (fallbackState != nullptr && isUsableState(*fallbackState)) {
    return *fallbackState;
  }

  return vector_t::Zero(info_.stateDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<vector3_t> SwitchedModelReferenceManager::getReferenceFootPositionsInWorld(scalar_t time,
                                                                                         const vector_t* fallbackState) const {
  const auto referenceState = getReferenceState(time, fallbackState);
  return getFootPositionsInWorld(referenceState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SupportPlane SwitchedModelReferenceManager::getSupportPlane(const vector_t& state, scalar_t time) const {
  const auto footPositionsInWorld = getReferenceFootPositionsInWorld(time, &state);
  const auto contactFlags = getContactFlags(time);
  return estimateSupportPlane(footPositionsInWorld, contactFlags);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  const scalar_t pastPadding = timeHorizon;
  const scalar_t futurePadding = 2.0 * timeHorizon;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - pastPadding, finalTime + futurePadding);

  feet_array_t<scalar_array_t> liftOffHeightSequence;
  feet_array_t<scalar_array_t> touchDownHeightSequence;
  feet_array_t<std::vector<vector3_t>> liftOffPositionSequence;
  feet_array_t<std::vector<vector3_t>> touchDownPositionSequence;
  for (auto& sequence : liftOffHeightSequence) {
    sequence.assign(modeSchedule.modeSequence.size(), 0.0);
  }
  for (auto& sequence : touchDownHeightSequence) {
    sequence.assign(modeSchedule.modeSequence.size(), 0.0);
  }
  for (auto& sequence : liftOffPositionSequence) {
    sequence.assign(modeSchedule.modeSequence.size(), vector3_t::Zero());
  }
  for (auto& sequence : touchDownPositionSequence) {
    sequence.assign(modeSchedule.modeSequence.size(), vector3_t::Zero());
  }

  if (modeSchedule.modeSequence.empty() || modeSchedule.eventTimes.empty()) {
    swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, liftOffPositionSequence,
                                touchDownPositionSequence);
    return;
  }

  for (size_t phase = 0; phase < modeSchedule.modeSequence.size(); ++phase) {
    const scalar_t phaseStartTime = phase == 0 ? initTime : modeSchedule.eventTimes[phase - 1];
    const scalar_t phaseEndTime = phase < modeSchedule.eventTimes.size() ? modeSchedule.eventTimes[phase] : finalTime;
    const auto startReferenceState = getReferenceState(phaseStartTime, &initState);
    const auto endReferenceState = getReferenceState(phaseEndTime, &initState);
    const auto startFootPositionsInWorld = getFootPositionsInWorld(startReferenceState);
    const auto endFootPositionsInWorld = getFootPositionsInWorld(endReferenceState);
    const auto contactFlags = modeNumber2StanceLeg(modeSchedule.modeSequence[phase]);
    const auto startSupportPlane =
        estimateSupportPlane(startFootPositionsInWorld, contactFlags);
    const auto endSupportPlane =
        estimateSupportPlane(endFootPositionsInWorld, contactFlags);
    const auto baseStartPositionInWorld = approximateBasePositionInWorld(info_, startReferenceState);
    const auto baseEndPositionInWorld = approximateBasePositionInWorld(info_, endReferenceState);
    const scalar_t phaseDuration = std::max<scalar_t>(phaseEndTime - phaseStartTime, 1e-3);
    const auto& terrainRotation = startSupportPlane.worldToTerrainRotation;
    const vector3_t baseTranslationInTerrain = terrainRotation * (baseEndPositionInWorld - baseStartPositionInWorld);
    const scalar_t startYaw = centroidal_model::getBasePose(startReferenceState, info_)(3);
    const scalar_t endYaw = centroidal_model::getBasePose(endReferenceState, info_)(3);
    const scalar_t yawRate = wrapAngleToPi(endYaw - startYaw) / phaseDuration;
    const vector3_t planarCommand(baseTranslationInTerrain.x() / phaseDuration, baseTranslationInTerrain.y() / phaseDuration, yawRate);

    for (size_t leg = 0; leg < contactNames3DoF_.size() && leg < liftOffHeightSequence.size(); ++leg) {
      const bool isInContact = contactFlags[leg];
      liftOffHeightSequence[leg][phase] =
          signedDistanceToSupportPlane(startFootPositionsInWorld[leg],
                                      startSupportPlane);
      touchDownHeightSequence[leg][phase] =
          signedDistanceToSupportPlane(endFootPositionsInWorld[leg], endSupportPlane);

      liftOffPositionSequence[leg][phase] =
          startFootPositionsInWorld[leg];

      if (!isInContact) {
        const vector3_t homePositionInTerrain =
            terrainRotation * (endFootPositionsInWorld[leg] - baseEndPositionInWorld);
        const vector3_t stepDeltaInTerrain =
            swingTrajectoryPtr_->computeTripodStepDelta(leg, homePositionInTerrain, planarCommand);
        const vector3_t touchDownRelativeInTerrain = homePositionInTerrain + stepDeltaInTerrain;

        vector3_t touchDownPositionInWorld =
            baseEndPositionInWorld + terrainRotation.transpose() * touchDownRelativeInTerrain;
        touchDownPositionInWorld.z() = solveSupportPlaneHeightAtXY(endSupportPlane, touchDownPositionInWorld.x(),
                                                                   touchDownPositionInWorld.y(),
                                                                   touchDownHeightSequence[leg][phase]);
        touchDownPositionSequence[leg][phase] = touchDownPositionInWorld;
      } else {
        touchDownPositionSequence[leg][phase] =
            endFootPositionsInWorld[leg];
      }
    }
  }

  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, liftOffPositionSequence,
                              touchDownPositionSequence);
}

}  // namespace legged_robot
}  // namespace ocs2
