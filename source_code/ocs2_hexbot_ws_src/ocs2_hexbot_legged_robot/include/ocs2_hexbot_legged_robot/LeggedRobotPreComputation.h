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

#pragma once

#include <limits>
#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "ocs2_hexbot_legged_robot/common/utils.h"
#include "ocs2_hexbot_legged_robot/common/ModelSettings.h"
#include "ocs2_hexbot_legged_robot/constraint/EndEffectorLinearConstraint.h"
#include "ocs2_hexbot_legged_robot/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_hexbot_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/** Callback for caching and reference update */
class LeggedRobotPreComputation : public PreComputation {
 public:
  LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                            const SwitchedModelReferenceManager& referenceManager, ModelSettings settings);
  ~LeggedRobotPreComputation() override = default;

  LeggedRobotPreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<EndEffectorLinearConstraint::Config>& getEeZeroVelocityConstraintConfigs() const { return eeZeroVelConConfigs_; }
  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }
  const SupportPlane& getSupportPlane() const { return supportPlane_; }
  const feet_array_t<vector3_t>& getFootPositionsInWorld() const { return footPositionsInWorld_; }
  const feet_array_t<scalar_t>& getFootNormalDistances() const { return footNormalDistances_; }
  const feet_array_t<scalar_t>& getFootReferenceNormalPositions() const { return footReferenceNormalPositions_; }
  const feet_array_t<scalar_t>& getFootReferenceNormalVelocities() const { return footReferenceNormalVelocities_; }
  const feet_array_t<vector3_t>& getFootReferencePositionsInWorld() const { return footReferencePositionsInWorld_; }
  const feet_array_t<vector3_t>& getFootReferenceVelocitiesInWorld() const { return footReferenceVelocitiesInWorld_; }

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  LeggedRobotPreComputation(const LeggedRobotPreComputation& other) = default;

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const ModelSettings settings_;

  std::vector<EndEffectorLinearConstraint::Config> eeZeroVelConConfigs_;
  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
  SupportPlane supportPlane_;
  feet_array_t<vector3_t> footPositionsInWorld_;
  feet_array_t<vector3_t> footReferencePositionsInWorld_;
  feet_array_t<vector3_t> footReferenceVelocitiesInWorld_;
  feet_array_t<scalar_t> footNormalDistances_{};
  feet_array_t<scalar_t> footReferenceNormalPositions_{};
  feet_array_t<scalar_t> footReferenceNormalVelocities_{};
  scalar_t nextDiagnosticPrintTime_ = -std::numeric_limits<scalar_t>::infinity();
};

}  // namespace legged_robot
}  // namespace ocs2
