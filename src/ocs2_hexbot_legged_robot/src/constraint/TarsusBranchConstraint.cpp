#include "ocs2_hexbot_legged_robot/constraint/TarsusBranchConstraint.h"

#include <algorithm>
#include <iomanip>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

TarsusBranchConstraint::TarsusBranchConstraint(vector_t lowerBranchLimits, vector_t upperBranchLimits, CentroidalModelInfo info,
                                               scalar_t diagnosticThreshold)
    : StateInputConstraint(ConstraintOrder::Linear),
      lowerBranchLimits_(std::move(lowerBranchLimits)),
      upperBranchLimits_(std::move(upperBranchLimits)),
      info_(std::move(info)),
      diagnosticThreshold_(diagnosticThreshold) {
  for (Eigen::Index i = 0; i < lowerBranchLimits_.size(); ++i) {
    if (std::isfinite(lowerBranchLimits_(i))) {
      constrainedLowerJointIndices_.push_back(i);
    }
    if (std::isfinite(upperBranchLimits_(i))) {
      constrainedUpperJointIndices_.push_back(i);
    }
  }
  numConstraints_ = constrainedLowerJointIndices_.size() + constrainedUpperJointIndices_.size();
}

vector_t TarsusBranchConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                          const PreComputation& preComp) const {
  const vector_t jointAngles = centroidal_model::getJointAngles(state, info_);
  vector_t values = vector_t::Zero(static_cast<Eigen::Index>(numConstraints_));

  Eigen::Index row = 0;
  for (const Eigen::Index jointIndex : constrainedLowerJointIndices_) {
    values(row++) = jointAngles(jointIndex) - lowerBranchLimits_(jointIndex);
  }
  for (const Eigen::Index jointIndex : constrainedUpperJointIndices_) {
    values(row++) = upperBranchLimits_(jointIndex) - jointAngles(jointIndex);
  }

  if (values.size() > 0) {
    const Eigen::Index minIndex =
        std::distance(values.data(), std::min_element(values.data(), values.data() + values.size()));
    const scalar_t minMargin = values(minIndex);
    if (minMargin < diagnosticThreshold_ && ((time >= nextDiagnosticPrintTime_) || (nextDiagnosticPrintTime_ - time > 1.0))) {
      const bool lowerConstraint = minIndex < static_cast<Eigen::Index>(constrainedLowerJointIndices_.size());
      const Eigen::Index jointIndex =
          lowerConstraint ? constrainedLowerJointIndices_[static_cast<size_t>(minIndex)]
                          : constrainedUpperJointIndices_[static_cast<size_t>(minIndex - constrainedLowerJointIndices_.size())];
      std::cerr << std::fixed << std::setprecision(4)
                << "[TarsusBranchConstraint] t=" << time
                << " joint_index=" << jointIndex
                << " side=" << (lowerConstraint ? "lower" : "upper")
                << " margin=" << minMargin
                << " angle=" << jointAngles(jointIndex)
                << " lower=" << lowerBranchLimits_(jointIndex)
                << " upper=" << upperBranchLimits_(jointIndex) << std::endl;
      nextDiagnosticPrintTime_ = time + 0.5;
    }
  }

  return values;
}

VectorFunctionLinearApproximation TarsusBranchConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const PreComputation& preComp) const {
  const Eigen::Index jointDim = static_cast<Eigen::Index>(info_.actuatedDofNum);
  const Eigen::Index stateDim = static_cast<Eigen::Index>(state.size());
  const Eigen::Index jointOffset = stateDim - jointDim;

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(static_cast<Eigen::Index>(numConstraints_), stateDim);
  approx.dfdu = matrix_t::Zero(static_cast<Eigen::Index>(numConstraints_), input.size());

  Eigen::Index row = 0;
  for (const Eigen::Index jointIndex : constrainedLowerJointIndices_) {
    approx.dfdx(row++, jointOffset + jointIndex) = 1.0;
  }
  for (const Eigen::Index jointIndex : constrainedUpperJointIndices_) {
    approx.dfdx(row++, jointOffset + jointIndex) = -1.0;
  }

  return approx;
}

}  // namespace legged_robot
}  // namespace ocs2
