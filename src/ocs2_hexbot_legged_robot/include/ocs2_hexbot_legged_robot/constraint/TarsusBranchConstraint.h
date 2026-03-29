#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

namespace ocs2 {
namespace legged_robot {

class TarsusBranchConstraint final : public StateInputConstraint {
 public:
  TarsusBranchConstraint(vector_t lowerBranchLimits, vector_t upperBranchLimits, CentroidalModelInfo info,
                         scalar_t diagnosticThreshold = 1e-3);

  ~TarsusBranchConstraint() override = default;
  TarsusBranchConstraint* clone() const override { return new TarsusBranchConstraint(*this); }

  bool isActive(scalar_t time) const override { return true; }
  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  TarsusBranchConstraint(const TarsusBranchConstraint& other) = default;

  vector_t lowerBranchLimits_;
  vector_t upperBranchLimits_;
  CentroidalModelInfo info_;
  scalar_t diagnosticThreshold_;
  std::vector<Eigen::Index> constrainedLowerJointIndices_;
  std::vector<Eigen::Index> constrainedUpperJointIndices_;
  size_t numConstraints_ = 0;
  mutable scalar_t nextDiagnosticPrintTime_ = -std::numeric_limits<scalar_t>::infinity();
};

}  // namespace legged_robot
}  // namespace ocs2
