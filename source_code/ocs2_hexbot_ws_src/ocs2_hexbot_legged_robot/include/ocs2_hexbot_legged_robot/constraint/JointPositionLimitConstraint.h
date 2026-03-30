#pragma once

#include <limits>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

namespace ocs2 {
namespace legged_robot {

class JointPositionLimitConstraint final : public StateInputConstraint {
 public:
  JointPositionLimitConstraint(vector_t lowerLimits, vector_t upperLimits, CentroidalModelInfo info,
                               scalar_t diagnosticThreshold = 1e-3);

  ~JointPositionLimitConstraint() override = default;
  JointPositionLimitConstraint* clone() const override { return new JointPositionLimitConstraint(*this); }

  bool isActive(scalar_t time) const override { return true; }
  size_t getNumConstraints(scalar_t time) const override { return static_cast<size_t>(2 * info_.actuatedDofNum); }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  JointPositionLimitConstraint(const JointPositionLimitConstraint& other) = default;

  vector_t lowerLimits_;
  vector_t upperLimits_;
  CentroidalModelInfo info_;
  scalar_t diagnosticThreshold_;
  mutable scalar_t nextDiagnosticPrintTime_ = -std::numeric_limits<scalar_t>::infinity();
};

}  // namespace legged_robot
}  // namespace ocs2
