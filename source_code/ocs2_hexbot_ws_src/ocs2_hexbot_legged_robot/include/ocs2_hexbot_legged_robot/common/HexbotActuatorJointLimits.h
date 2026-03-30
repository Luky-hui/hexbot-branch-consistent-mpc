#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot {

constexpr scalar_t kHexbotCoxaLowerLimit = -1.05;
constexpr scalar_t kHexbotCoxaUpperLimit = 1.05;
constexpr scalar_t kHexbotFemurLowerLimit = -1.00;
constexpr scalar_t kHexbotFemurUpperLimit = 1.00;
constexpr scalar_t kHexbotTarsusLowerLimit = -1.35;
constexpr scalar_t kHexbotTarsusUpperLimit = 1.35;
constexpr scalar_t kHexbotCoxaGuardBand = 0.03;
constexpr scalar_t kHexbotFemurGuardBand = 0.05;
constexpr scalar_t kHexbotTarsusGuardBand = 0.05;

inline std::pair<vector_t, vector_t> getHexbotActuatorJointLimits(const std::vector<std::string>& jointNames,
                                                                  scalar_t fallbackLower = -M_PI,
                                                                  scalar_t fallbackUpper = M_PI) {
  vector_t lower = vector_t::Constant(static_cast<Eigen::Index>(jointNames.size()), fallbackLower);
  vector_t upper = vector_t::Constant(static_cast<Eigen::Index>(jointNames.size()), fallbackUpper);

  for (size_t i = 0; i < jointNames.size(); ++i) {
    const auto& jointName = jointNames[i];
    if (jointName.rfind("coxa_joint_", 0) == 0) {
      lower(static_cast<Eigen::Index>(i)) = kHexbotCoxaLowerLimit;
      upper(static_cast<Eigen::Index>(i)) = kHexbotCoxaUpperLimit;
    } else if (jointName.rfind("femur_joint_", 0) == 0) {
      lower(static_cast<Eigen::Index>(i)) = kHexbotFemurLowerLimit;
      upper(static_cast<Eigen::Index>(i)) = kHexbotFemurUpperLimit;
    } else if (jointName.rfind("tarsus_joint_", 0) == 0) {
      lower(static_cast<Eigen::Index>(i)) = kHexbotTarsusLowerLimit;
      upper(static_cast<Eigen::Index>(i)) = kHexbotTarsusUpperLimit;
    }
  }

  return {std::move(lower), std::move(upper)};
}

inline std::pair<vector_t, vector_t> getHexbotActuatorGuardedJointLimits(const std::vector<std::string>& jointNames,
                                                                         scalar_t fallbackLower = -M_PI,
                                                                         scalar_t fallbackUpper = M_PI) {
  auto [lower, upper] = getHexbotActuatorJointLimits(jointNames, fallbackLower, fallbackUpper);

  for (size_t i = 0; i < jointNames.size(); ++i) {
    const auto& jointName = jointNames[i];
    scalar_t guardBand = 0.0;
    if (jointName.rfind("coxa_joint_", 0) == 0) {
      guardBand = kHexbotCoxaGuardBand;
    } else if (jointName.rfind("femur_joint_", 0) == 0) {
      guardBand = kHexbotFemurGuardBand;
    } else if (jointName.rfind("tarsus_joint_", 0) == 0) {
      guardBand = kHexbotTarsusGuardBand;
    }

    const Eigen::Index index = static_cast<Eigen::Index>(i);
    if (guardBand <= 0.0) {
      continue;
    }

    const scalar_t guardedLower = lower(index) + guardBand;
    const scalar_t guardedUpper = upper(index) - guardBand;
    if (guardedLower < guardedUpper) {
      lower(index) = guardedLower;
      upper(index) = guardedUpper;
    }
  }

  return {std::move(lower), std::move(upper)};
}

}  // namespace legged_robot
}  // namespace ocs2
