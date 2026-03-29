/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <iomanip>

#include <ocs2_core/misc/Numerics.h>

#include <ocs2_hexbot_legged_robot/LeggedRobotPreComputation.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                     const SwitchedModelReferenceManager& referenceManager, ModelSettings settings)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      referenceManagerPtr_(&referenceManager),
      settings_(std::move(settings)) {
  eeZeroVelConConfigs_.resize(info_.numThreeDofContacts);
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  footPositionsInWorld_.fill(vector3_t::Zero());
  footReferencePositionsInWorld_.fill(vector3_t::Zero());
  footReferenceVelocitiesInWorld_.fill(vector3_t::Zero());
  footNormalDistances_.fill(0.0);
  footReferenceNormalPositions_.fill(0.0);
  footReferenceNormalVelocities_.fill(0.0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation* LeggedRobotPreComputation::clone() const {
  return new LeggedRobotPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  const auto contactFlags = referenceManagerPtr_->getContactFlags(t);
  footPositionsInWorld_ = computeFootPositionsInWorld(pinocchioInterface_, info_, settings_.contactNames3DoF, x);
  supportPlane_ = referenceManagerPtr_->getSupportPlane(x, t);

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    const vector2_t desiredPlanarPosition = swingTrajectoryPlannerPtr_->getPlanarPositionConstraint(footIndex, t);
    const vector2_t desiredPlanarVelocity = swingTrajectoryPlannerPtr_->getPlanarVelocityConstraint(footIndex, t);
    const scalar_t desiredNormalVelocity = swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t);
    const scalar_t desiredNormalPosition = swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
    footReferenceNormalPositions_[footIndex] = desiredNormalPosition;
    footReferenceNormalVelocities_[footIndex] = desiredNormalVelocity;
    footNormalDistances_[footIndex] = signedDistanceToSupportPlane(footPositionsInWorld_[footIndex], supportPlane_);

    vector3_t desiredFootPositionInWorld(desiredPlanarPosition.x(), desiredPlanarPosition.y(),
                                         solveSupportPlaneHeightAtXY(supportPlane_, desiredPlanarPosition.x(),
                                                                     desiredPlanarPosition.y(), desiredNormalPosition));
    vector3_t desiredFootVelocityInWorld(desiredPlanarVelocity.x(), desiredPlanarVelocity.y(), 0.0);
    const scalar_t normalZ = std::abs(supportPlane_.normalInWorld.z()) > 1e-6 ? supportPlane_.normalInWorld.z() : 1.0;
    desiredFootVelocityInWorld.z() =
        (desiredNormalVelocity - supportPlane_.normalInWorld.x() * desiredFootVelocityInWorld.x() -
         supportPlane_.normalInWorld.y() * desiredFootVelocityInWorld.y()) /
        normalZ;
    footReferencePositionsInWorld_[footIndex] = desiredFootPositionInWorld;
    footReferenceVelocitiesInWorld_[footIndex] = desiredFootVelocityInWorld;

    config.b = -supportPlane_.worldToTerrainRotation * desiredFootVelocityInWorld;
    config.Av = supportPlane_.worldToTerrainRotation;
    const scalar_t swingPositionErrorGain = 5.0 * settings_.positionErrorGain;
    config.Ax = swingPositionErrorGain * supportPlane_.worldToTerrainRotation;
    config.b -= swingPositionErrorGain * (supportPlane_.worldToTerrainRotation * desiredFootPositionInWorld);
    return config;
  };

  auto eeZeroVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = vector_t::Zero(3);
    config.Av = supportPlane_.worldToTerrainRotation;
    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
      config.Ax = matrix_t::Zero(3, 3);
      // Normal (z-in-terrain) position constraint — keeps foot on support plane
      config.Ax.row(2) =
          settings_.positionErrorGain *
          supportPlane_.normalInWorld.transpose();
      config.b(2) -=
          settings_.positionErrorGain * supportPlane_.heightAlongNormal;

      // Planar (x,y-in-terrain) position anchor — pins stance foot in world XY
      const vector2_t anchorXY = swingTrajectoryPlannerPtr_->getPlanarPositionConstraint(footIndex, t);
      const scalar_t anchorZ = solveSupportPlaneHeightAtXY(supportPlane_, anchorXY.x(), anchorXY.y(), 0.0);
      const vector3_t anchor(anchorXY.x(), anchorXY.y(), anchorZ);
      const scalar_t stancePlanarGain = settings_.positionErrorGain;
      config.Ax.row(0) = stancePlanarGain * supportPlane_.worldToTerrainRotation.row(0);
      config.Ax.row(1) = stancePlanarGain * supportPlane_.worldToTerrainRotation.row(1);
      config.b(0) -= stancePlanarGain * supportPlane_.worldToTerrainRotation.row(0).dot(anchor);
      config.b(1) -= stancePlanarGain * supportPlane_.worldToTerrainRotation.row(1).dot(anchor);
    }
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeZeroVelConConfigs_[i] = eeZeroVelConConfig(i);
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
    }
  }

  const bool shouldPrintDiagnostics =
      (t >= nextDiagnosticPrintTime_) || (nextDiagnosticPrintTime_ - t > 1.0) || !std::isfinite(nextDiagnosticPrintTime_);
  if (shouldPrintDiagnostics) {
    std::cerr << std::fixed << std::setprecision(4)
              << "[LeggedRobotPreComputation] t=" << t
              << " support_normal=[" << supportPlane_.normalInWorld.x() << ", " << supportPlane_.normalInWorld.y() << ", "
              << supportPlane_.normalInWorld.z() << "]"
              << " support_height=" << supportPlane_.heightAlongNormal << std::endl;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
      const auto contactForceInWorld = centroidal_model::getContactForces(u, i, info_);
      const scalar_t normalForce = supportPlane_.normalInWorld.dot(contactForceInWorld);
      std::cerr << "  foot[" << i << "] contact=" << static_cast<int>(contactFlags[i])
                << " pos=[" << footPositionsInWorld_[i].x() << ", " << footPositionsInWorld_[i].y() << ", "
                << footPositionsInWorld_[i].z() << "]"
                << " ref_pos=[" << footReferencePositionsInWorld_[i].x() << ", " << footReferencePositionsInWorld_[i].y() << ", "
                << footReferencePositionsInWorld_[i].z() << "]"
                << " ref_vel_world=[" << footReferenceVelocitiesInWorld_[i].x() << ", " << footReferenceVelocitiesInWorld_[i].y() << ", "
                << footReferenceVelocitiesInWorld_[i].z() << "]"
                << " normal_dist=" << footNormalDistances_[i]
                << " ref_dist=" << footReferenceNormalPositions_[i]
                << " ref_normal_vel=" << footReferenceNormalVelocities_[i]
                << " normal_force=" << normalForce << std::endl;
    }
    nextDiagnosticPrintTime_ = t + 0.5;
  }
}

}  // namespace legged_robot
}  // namespace ocs2
