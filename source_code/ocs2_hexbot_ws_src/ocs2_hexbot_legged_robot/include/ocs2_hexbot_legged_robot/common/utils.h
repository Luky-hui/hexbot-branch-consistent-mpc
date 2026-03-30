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

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cppad/cg.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_hexbot_legged_robot/common/Types.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Counts contact feet */
inline size_t numberOfClosedContacts(const contact_flag_t& contactFlags) {
  size_t numStanceLegs = 0;
  for (auto legInContact : contactFlags) {
    if (legInContact) {
      ++numStanceLegs;
    }
  }
  return numStanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
struct SupportPlane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vector3_t normalInWorld = vector3_t::UnitZ();
  matrix3_t worldToTerrainRotation = matrix3_t::Identity();
  scalar_t heightAlongNormal = 0.0;
  size_t numSupportFeet = 0;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline matrix3_t worldToTerrainRotationFromSurfaceNormal(const vector3_t& surfaceNormalInWorld) {
  vector3_t terrainZ = surfaceNormalInWorld;
  if (!terrainZ.allFinite() || terrainZ.norm() < 1e-6) {
    terrainZ = vector3_t::UnitZ();
  } else {
    terrainZ.normalize();
  }

  vector3_t terrainX = vector3_t::UnitX() - terrainZ.dot(vector3_t::UnitX()) * terrainZ;
  if (terrainX.norm() < 1e-6) {
    terrainX = vector3_t::UnitY() - terrainZ.dot(vector3_t::UnitY()) * terrainZ;
  }
  terrainX.normalize();

  vector3_t terrainY = terrainZ.cross(terrainX);
  terrainY.normalize();

  matrix3_t worldToTerrainRotation;
  worldToTerrainRotation.row(0) = terrainX.transpose();
  worldToTerrainRotation.row(1) = terrainY.transpose();
  worldToTerrainRotation.row(2) = terrainZ.transpose();
  return worldToTerrainRotation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline scalar_t signedDistanceToSupportPlane(const vector3_t& pointInWorld, const SupportPlane& supportPlane) {
  return supportPlane.normalInWorld.dot(pointInWorld) - supportPlane.heightAlongNormal;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline SupportPlane estimateSupportPlane(const feet_array_t<vector3_t>& footPositionsInWorld, const contact_flag_t& contactFlags) {
  std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> supportPoints;
  supportPoints.reserve(footPositionsInWorld.size());
  for (size_t i = 0; i < contactFlags.size(); ++i) {
    if (contactFlags[i] && footPositionsInWorld[i].allFinite()) {
      supportPoints.push_back(footPositionsInWorld[i]);
    }
  }

  if (supportPoints.empty()) {
    for (const auto& footPosition : footPositionsInWorld) {
      if (footPosition.allFinite()) {
        supportPoints.push_back(footPosition);
      }
    }
  }

  SupportPlane supportPlane;
  supportPlane.numSupportFeet = supportPoints.size();
  if (supportPoints.empty()) {
    supportPlane.worldToTerrainRotation = worldToTerrainRotationFromSurfaceNormal(supportPlane.normalInWorld);
    return supportPlane;
  }

  vector3_t centroid = vector3_t::Zero();
  for (const auto& point : supportPoints) {
    centroid += point;
  }
  centroid /= static_cast<scalar_t>(supportPoints.size());

  vector3_t supportNormal = vector3_t::UnitZ();
  if (supportPoints.size() >= 3) {
    matrix3_t covariance = matrix3_t::Zero();
    for (const auto& point : supportPoints) {
      const vector3_t centered = point - centroid;
      covariance.noalias() += centered * centered.transpose();
    }
    Eigen::SelfAdjointEigenSolver<matrix3_t> eigenSolver(covariance);
    if (eigenSolver.info() == Eigen::Success) {
      supportNormal = eigenSolver.eigenvectors().col(0);
    }
  }
  if (supportNormal.z() < 0.0) {
    supportNormal = -supportNormal;
  }
  if (!supportNormal.allFinite() || supportNormal.norm() < 1e-6) {
    supportNormal = vector3_t::UnitZ();
  } else {
    supportNormal.normalize();
  }

  scalar_t planeHeight = 0.0;
  for (const auto& point : supportPoints) {
    planeHeight += supportNormal.dot(point);
  }
  planeHeight /= static_cast<scalar_t>(supportPoints.size());

  supportPlane.normalInWorld = supportNormal;
  supportPlane.heightAlongNormal = planeHeight;
  supportPlane.worldToTerrainRotation = worldToTerrainRotationFromSurfaceNormal(supportNormal);
  return supportPlane;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline SupportPlane estimateSupportPlaneFromLowestFeet(const feet_array_t<vector3_t>& footPositionsInWorld,
                                                       scalar_t maxHeightAboveLowest = 0.02,
                                                       size_t minimumSupportFeet = 3) {
  std::vector<std::pair<scalar_t, size_t>> orderedFeet;
  orderedFeet.reserve(footPositionsInWorld.size());
  for (size_t i = 0; i < footPositionsInWorld.size(); ++i) {
    if (footPositionsInWorld[i].allFinite()) {
      orderedFeet.emplace_back(footPositionsInWorld[i].z(), i);
    }
  }

  if (orderedFeet.empty()) {
    return SupportPlane{};
  }

  std::sort(orderedFeet.begin(), orderedFeet.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

  contact_flag_t supportFeet{};
  supportFeet.fill(false);

  const scalar_t lowestHeight = orderedFeet.front().first;
  size_t selectedFeet = 0;
  for (const auto& [height, index] : orderedFeet) {
    if (selectedFeet < minimumSupportFeet || height <= lowestHeight + maxHeightAboveLowest) {
      supportFeet[index] = true;
      ++selectedFeet;
    }
  }

  return estimateSupportPlane(footPositionsInWorld, supportFeet);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline scalar_t solveSupportPlaneHeightAtXY(const SupportPlane& supportPlane, scalar_t x, scalar_t y,
                                            scalar_t desiredNormalDistance) {
  const scalar_t normalZ = std::abs(supportPlane.normalInWorld.z()) > 1e-6 ? supportPlane.normalInWorld.z() : 1.0;
  return (supportPlane.heightAlongNormal + desiredNormalDistance - supportPlane.normalInWorld.x() * x -
          supportPlane.normalInWorld.y() * y) /
         normalZ;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline feet_array_t<vector3_t> computeFootPositionsInWorld(PinocchioInterface& pinocchioInterface,
                                                           const CentroidalModelInfoTpl<scalar_t>& info,
                                                           const std::vector<std::string>& contactNames3DoF,
                                                           const vector_t& state) {
  feet_array_t<vector3_t> footPositionsInWorld;
  footPositionsInWorld.fill(vector3_t::Zero());

  const auto q = centroidal_model::getGeneralizedCoordinates(state, info);
  auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  const size_t numContacts = std::min(contactNames3DoF.size(), footPositionsInWorld.size());
  for (size_t i = 0; i < numContacts; ++i) {
    const auto frameId = model.getBodyId(contactNames3DoF[i]);
    footPositionsInWorld[i] = data.oMf[frameId].translation();
  }

  return footPositionsInWorld;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline vector3_t approximateBasePositionInWorld(const CentroidalModelInfoTpl<scalar_t>& info, const vector_t& state) {
  return centroidal_model::getGeneralizedCoordinates(state, info).head<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Shifts joint angles in state by multiples of 2pi to be on the same branch as referenceState. */
inline void branchNormalizeJointAngles(vector_t& state, const vector_t& referenceState,
                                       const CentroidalModelInfoTpl<scalar_t>& info) {
  const auto refJoints = centroidal_model::getJointAngles(referenceState, info);
  auto joints = centroidal_model::getJointAngles(state, info);
  for (Eigen::Index i = 0; i < joints.size(); ++i) {
    const scalar_t diff = refJoints(i) - joints(i);
    const scalar_t shift = std::round(diff / (2.0 * M_PI)) * (2.0 * M_PI);
    if (std::abs(shift) > 1e-6) {
      joints(i) += shift;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline vector3_t computeDesiredHorizontalForce(const CentroidalModelInfoTpl<scalar_t>& info, const vector_t& desiredState,
                                               const vector_t& currentState, scalar_t responseTime, scalar_t forceScale,
                                               scalar_t maxAcceleration, scalar_t deadzoneVelocity) {
  if (desiredState.size() < 2 || currentState.size() < 2) {
    return vector3_t::Zero();
  }

  vector3_t desiredVelocity(desiredState(0), desiredState(1), 0.0);
  vector3_t currentVelocity(currentState(0), currentState(1), 0.0);
  if (!desiredVelocity.allFinite() || !currentVelocity.allFinite()) {
    return vector3_t::Zero();
  }

  const scalar_t safeDeadzoneVelocity = std::max<scalar_t>(deadzoneVelocity, 0.0);
  if (desiredVelocity.head<2>().norm() < safeDeadzoneVelocity) {
    return vector3_t::Zero();
  }

  const scalar_t safeResponseTime = std::max<scalar_t>(responseTime, 1e-3);
  vector3_t desiredAcceleration = forceScale * (desiredVelocity - currentVelocity) / safeResponseTime;
  desiredAcceleration.z() = 0.0;

  const scalar_t safeMaxAcceleration = std::max<scalar_t>(maxAcceleration, 0.0);
  const scalar_t desiredAccelerationNorm = desiredAcceleration.head<2>().norm();
  if (safeMaxAcceleration > 1e-6 && desiredAccelerationNorm > safeMaxAcceleration) {
    desiredAcceleration.head<2>() *= safeMaxAcceleration / desiredAccelerationNorm;
  }

  return info.robotMass * desiredAcceleration;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Computes an input with zero joint velocity and forces which equally distribute the robot weight between contact feet. */
inline vector_t weightCompensatingInput(const CentroidalModelInfoTpl<scalar_t>& info, const contact_flag_t& contactFlags) {
  const auto numStanceLegs = numberOfClosedContacts(contactFlags);
  vector_t input = vector_t::Zero(info.inputDim);
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = info.robotMass * 9.81;
    const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
    for (size_t i = 0; i < contactFlags.size(); i++) {
      if (contactFlags[i]) {
        centroidal_model::getContactForces(input, i, info) = forceInInertialFrame;
      }
    }  // end of i loop
  }
  return input;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Computes a terrain-aware support-force distribution using the current support polygon and base projection. */
inline vector_t weightCompensatingInput(const CentroidalModelInfoTpl<scalar_t>& info, const contact_flag_t& contactFlags,
                                        const feet_array_t<vector3_t>& footPositionsInWorld, const vector3_t& bodyPositionInWorld,
                                        const SupportPlane& supportPlane,
                                        const vector3_t& desiredHorizontalForce = vector3_t::Zero()) {
  const auto numStanceLegs = numberOfClosedContacts(contactFlags);
  if (numStanceLegs == 0) {
    return vector_t::Zero(info.inputDim);
  }

  std::vector<size_t> stanceIndices;
  stanceIndices.reserve(contactFlags.size());
  for (size_t i = 0; i < contactFlags.size(); ++i) {
    if (contactFlags[i]) {
      stanceIndices.push_back(i);
    }
  }

  Eigen::VectorXd weights = Eigen::VectorXd::Constant(stanceIndices.size(), 1.0 / static_cast<scalar_t>(stanceIndices.size()));
  if (stanceIndices.size() > 1) {
    const auto& worldToTerrainRotation = supportPlane.worldToTerrainRotation;
    const vector3_t bodyInTerrain = worldToTerrainRotation * bodyPositionInWorld;

    Eigen::MatrixXd A(3, stanceIndices.size());
    A.row(0).setOnes();
    for (size_t column = 0; column < stanceIndices.size(); ++column) {
      const vector3_t footInTerrain = worldToTerrainRotation * footPositionsInWorld[stanceIndices[column]];
      A(1, column) = footInTerrain.x();
      A(2, column) = footInTerrain.y();
    }

    Eigen::Vector3d b;
    b << 1.0, bodyInTerrain.x(), bodyInTerrain.y();
    const Eigen::VectorXd solvedWeights = A.completeOrthogonalDecomposition().solve(b);
    if (solvedWeights.size() == weights.size() && solvedWeights.allFinite()) {
      weights = solvedWeights;
    }
  }

  for (int i = 0; i < weights.size(); ++i) {
    if (!std::isfinite(weights(i)) || weights(i) < 0.0) {
      weights(i) = 0.0;
    }
  }

  const scalar_t weightSum = weights.sum();
  if (!(weightSum > 1e-6)) {
    weights.setConstant(1.0 / static_cast<scalar_t>(weights.size()));
  } else {
    weights /= weightSum;
  }

  const scalar_t totalWeight = info.robotMass * 9.81;
  const scalar_t normalVerticalComponent = std::max<scalar_t>(supportPlane.normalInWorld.z(), 1e-3);
  const scalar_t totalNormalForce = totalWeight / normalVerticalComponent;

  // Horizontal propulsion force per stance foot (distributed equally)
  vector3_t horizontalForcePerFoot = vector3_t::Zero();
  if (numStanceLegs > 0) {
    horizontalForcePerFoot = desiredHorizontalForce / static_cast<scalar_t>(numStanceLegs);
  }

  vector_t input = vector_t::Zero(info.inputDim);
  for (size_t i = 0; i < stanceIndices.size(); ++i) {
    centroidal_model::getContactForces(input, stanceIndices[i], info) =
        supportPlane.normalInWorld * (totalNormalForce * weights(i)) + horizontalForcePerFoot;
  }
  return input;
}

}  // namespace legged_robot
}  // namespace ocs2
