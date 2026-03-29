#include "ocs2_hexbot_legged_robot/constraint/JointPositionLimitConstraint.h"

#include <algorithm>
#include <iomanip>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

JointPositionLimitConstraint::JointPositionLimitConstraint(vector_t lowerLimits, vector_t upperLimits, CentroidalModelInfo info,
                                                           scalar_t diagnosticThreshold)
    : StateInputConstraint(ConstraintOrder::Linear),
      lowerLimits_(std::move(lowerLimits)),
      upperLimits_(std::move(upperLimits)),
      info_(std::move(info)),
      diagnosticThreshold_(diagnosticThreshold) {}

vector_t JointPositionLimitConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                const PreComputation& preComp) const {
  const vector_t jointAngles = centroidal_model::getJointAngles(state, info_);
  vector_t values(2 * jointAngles.size());
  values.head(jointAngles.size()) = jointAngles - lowerLimits_;
  values.tail(jointAngles.size()) = upperLimits_ - jointAngles;

  const Eigen::Index minIndex = std::distance(values.data(), std::min_element(values.data(), values.data() + values.size()));
  const scalar_t minMargin = values(minIndex);
  if (minMargin < diagnosticThreshold_ && ((time >= nextDiagnosticPrintTime_) || (nextDiagnosticPrintTime_ - time > 1.0))) {
    const bool lowerViolation = minIndex < jointAngles.size();
    const Eigen::Index jointIndex = lowerViolation ? minIndex : minIndex - jointAngles.size();
    std::cerr << std::fixed << std::setprecision(4)
              << "[JointPositionLimitConstraint] t=" << time
              << " joint_index=" << jointIndex
              << " side=" << (lowerViolation ? "lower" : "upper")
              << " margin=" << minMargin
              << " angle=" << jointAngles(jointIndex)
              << " lower=" << lowerLimits_(jointIndex)
              << " upper=" << upperLimits_(jointIndex) << std::endl;
    nextDiagnosticPrintTime_ = time + 0.5;
  }

  return values;
}

VectorFunctionLinearApproximation JointPositionLimitConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                       const vector_t& input,
                                                                                       const PreComputation& preComp) const {
  const Eigen::Index jointDim = static_cast<Eigen::Index>(info_.actuatedDofNum);
  const Eigen::Index stateDim = static_cast<Eigen::Index>(state.size());
  const Eigen::Index jointOffset = stateDim - jointDim;

  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(2 * jointDim, stateDim);
  approx.dfdu = matrix_t::Zero(2 * jointDim, input.size());
  approx.dfdx.block(0, jointOffset, jointDim, jointDim).diagonal().setOnes();
  approx.dfdx.block(jointDim, jointOffset, jointDim, jointDim).diagonal().setConstant(-1.0);
  return approx;
}

}  // namespace legged_robot
}  // namespace ocs2
