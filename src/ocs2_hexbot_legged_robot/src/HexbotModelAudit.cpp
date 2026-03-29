#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <array>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_hexbot_legged_robot/LeggedRobotInterface.h"
#include "ocs2_hexbot_legged_robot/common/HexbotActuatorJointLimits.h"
#include "ocs2_hexbot_legged_robot/common/utils.h"
#include "ocs2_hexbot_legged_robot/package_path.h"

namespace ocs2 {
namespace legged_robot {
namespace {

struct Options {
  std::string taskFile = getPath() + "/config/mpc/task.info";
  std::string urdfFile = getPath() + "/../hexbot_description_ros2/urdf/hexbot_isaac_rooted.urdf";
  std::string referenceFile = getPath() + "/config/command/reference.info";
  std::string label = "reference";
};

Options parseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto requireValue = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("[HexbotModelAudit] Missing value for " + name);
      }
      return std::string(argv[++i]);
    };
    if (arg == "--task") {
      options.taskFile = requireValue(arg);
    } else if (arg == "--urdf") {
      options.urdfFile = requireValue(arg);
    } else if (arg == "--reference") {
      options.referenceFile = requireValue(arg);
    } else if (arg == "--label") {
      options.label = requireValue(arg);
    } else {
      throw std::runtime_error("[HexbotModelAudit] Unknown argument: " + arg);
    }
  }
  return options;
}

vector_t loadReferenceJointState(const CentroidalModelInfo& info, const std::string& referenceFile) {
  return centroidal_model::loadDefaultJointState(info.actuatedDofNum, referenceFile);
}

scalar_t loadReferenceComHeight(const std::string& referenceFile, scalar_t fallback) {
  scalar_t comHeight = fallback;
  try {
    loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  } catch (const std::exception&) {
  }
  return comHeight;
}

std::string legNameForIndex(size_t index) {
  static const std::array<const char*, 6> kLegNames = {"RR", "RM", "RF", "LR", "LM", "LF"};
  return kLegNames.at(index);
}

void printVector3(const vector3_t& value) {
  std::cout << "[" << value.x() << ", " << value.y() << ", " << value.z() << "]";
}

}  // namespace

int runAudit(int argc, char** argv) {
  const auto options = parseArgs(argc, argv);
  LeggedRobotInterface interface(options.taskFile, options.urdfFile, options.referenceFile);

  const auto& info = interface.getCentroidalModelInfo();
  const auto& modelSettings = interface.modelSettings();
  vector_t state = interface.getInitialState();
  const vector_t referenceJointState = loadReferenceJointState(info, options.referenceFile);
  const scalar_t referenceComHeight = loadReferenceComHeight(options.referenceFile, state(8));
  state.segment(state.size() - info.actuatedDofNum, info.actuatedDofNum) = referenceJointState;
  state(8) = referenceComHeight;

  auto footPositions = computeFootPositionsInWorld(interface.getPinocchioInterface(), info, modelSettings.contactNames3DoF, state);
  contact_flag_t allContacts;
  allContacts.fill(true);
  const auto supportPlane = estimateSupportPlane(footPositions, allContacts);
  const auto [lowerLimits, upperLimits] = getHexbotActuatorJointLimits(modelSettings.jointNames);

  auto& pinocchioInterface = interface.getPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(state, info);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, q);

  const scalar_t nominalNormalForce = info.robotMass * 9.81 / static_cast<scalar_t>(std::max<size_t>(1, info.numThreeDofContacts));

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "# Hexbot Model Audit\n";
  std::cout << "label: " << options.label << "\n";
  std::cout << "task: " << options.taskFile << "\n";
  std::cout << "urdf: " << options.urdfFile << "\n";
  std::cout << "reference: " << options.referenceFile << "\n";
  std::cout << "reference_com_height_m: " << referenceComHeight << "\n";
  std::cout << "support_normal: ";
  printVector3(supportPlane.normalInWorld);
  std::cout << "\n";
  std::cout << "support_height_m: " << supportPlane.heightAlongNormal << "\n";

  std::cout << "\n## Per-Leg Metrics\n";
  for (size_t leg = 0; leg < info.numThreeDofContacts; ++leg) {
    matrix_t jacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, model.getFrameId(modelSettings.contactNames3DoF[leg]), pinocchio::LOCAL_WORLD_ALIGNED,
                                jacobian);
    matrix3_t legJacobian = matrix3_t::Zero();
    for (size_t jointOffset = 0; jointOffset < 3; ++jointOffset) {
      const auto jointId = model.getJointId(modelSettings.jointNames[3 * leg + jointOffset]);
      const auto velocityIndex = static_cast<Eigen::Index>(model.joints[jointId].idx_v());
      legJacobian.col(static_cast<Eigen::Index>(jointOffset)) = jacobian.block<3, 1>(0, velocityIndex);
    }
    const matrix3_t terrainLegJacobian = supportPlane.worldToTerrainRotation * legJacobian;
    const Eigen::JacobiSVD<matrix3_t> svd(terrainLegJacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const vector3_t supportForce = supportPlane.normalInWorld * nominalNormalForce;
    const vector3_t supportTorque = legJacobian.transpose() * supportForce;
    const vector3_t footPosition = footPositions[leg];
    const vector3_t terrainFootPosition = supportPlane.worldToTerrainRotation * footPosition;
    const vector_t legJointState = referenceJointState.segment<3>(3 * leg);
    const vector_t lowerMargin = legJointState - lowerLimits.segment<3>(3 * leg);
    const vector_t upperMargin = upperLimits.segment<3>(3 * leg) - legJointState;

    std::cout << "- leg: " << legNameForIndex(leg) << "\n";
    std::cout << "  foot_world: ";
    printVector3(footPosition);
    std::cout << "\n";
    std::cout << "  foot_terrain: ";
    printVector3(terrainFootPosition);
    std::cout << "\n";
    std::cout << "  normal_distance_m: " << signedDistanceToSupportPlane(footPosition, supportPlane) << "\n";
    std::cout << "  joint_state_rad: [" << legJointState(0) << ", " << legJointState(1) << ", " << legJointState(2) << "]\n";
    std::cout << "  lower_margin_rad: [" << lowerMargin(0) << ", " << lowerMargin(1) << ", " << lowerMargin(2) << "]\n";
    std::cout << "  upper_margin_rad: [" << upperMargin(0) << ", " << upperMargin(1) << ", " << upperMargin(2) << "]\n";
    std::cout << "  terrain_jacobian_det: " << terrainLegJacobian.determinant() << "\n";
    std::cout << "  terrain_jacobian_sigma: [" << svd.singularValues()(0) << ", " << svd.singularValues()(1) << ", "
              << svd.singularValues()(2) << "]\n";
    std::cout << "  terrain_vertical_response: [" << terrainLegJacobian.row(2)(0) << ", " << terrainLegJacobian.row(2)(1) << ", "
              << terrainLegJacobian.row(2)(2) << "]\n";
    std::cout << "  nominal_support_tau: [" << supportTorque(0) << ", " << supportTorque(1) << ", " << supportTorque(2) << "]\n";
  }

  std::cout << "\n## Mirror Checks\n";
  const std::array<std::pair<size_t, size_t>, 3> mirrorPairs = {{{0, 3}, {1, 4}, {2, 5}}};
  const matrix3_t rowMirror = (matrix3_t() << 1.0, 0.0, 0.0,
                                               0.0, -1.0, 0.0,
                                               0.0, 0.0, 1.0)
                                  .finished();
  const matrix3_t jointMirror = (matrix3_t() << 1.0, 0.0, 0.0,
                                                 0.0, -1.0, 0.0,
                                                 0.0, 0.0, -1.0)
                                    .finished();
  for (const auto& [rightIndex, leftIndex] : mirrorPairs) {
    matrix_t rightJacobian = matrix_t::Zero(6, model.nv);
    matrix_t leftJacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, model.getFrameId(modelSettings.contactNames3DoF[rightIndex]), pinocchio::LOCAL_WORLD_ALIGNED,
                                rightJacobian);
    pinocchio::getFrameJacobian(model, data, model.getFrameId(modelSettings.contactNames3DoF[leftIndex]), pinocchio::LOCAL_WORLD_ALIGNED,
                                leftJacobian);
    matrix3_t rightLegJacobian = matrix3_t::Zero();
    matrix3_t leftLegJacobian = matrix3_t::Zero();
    for (size_t jointOffset = 0; jointOffset < 3; ++jointOffset) {
      const auto rightJointId = model.getJointId(modelSettings.jointNames[3 * rightIndex + jointOffset]);
      const auto leftJointId = model.getJointId(modelSettings.jointNames[3 * leftIndex + jointOffset]);
      rightLegJacobian.col(static_cast<Eigen::Index>(jointOffset)) =
          rightJacobian.block<3, 1>(0, static_cast<Eigen::Index>(model.joints[rightJointId].idx_v()));
      leftLegJacobian.col(static_cast<Eigen::Index>(jointOffset)) =
          leftJacobian.block<3, 1>(0, static_cast<Eigen::Index>(model.joints[leftJointId].idx_v()));
    }
    const matrix3_t mirroredLeftLegJacobian = rowMirror * leftLegJacobian * jointMirror;
    const vector3_t rightPos = footPositions[rightIndex];
    const vector3_t leftPos = footPositions[leftIndex];
    const vector3_t positionMirrorError(rightPos.x() - leftPos.x(), rightPos.y() + leftPos.y(), rightPos.z() - leftPos.z());

    std::cout << "- pair: " << legNameForIndex(rightIndex) << " <-> " << legNameForIndex(leftIndex) << "\n";
    std::cout << "  position_mirror_error_m: ";
    printVector3(positionMirrorError);
    std::cout << "\n";
    std::cout << "  jacobian_mirror_error_fro: " << (rightLegJacobian - mirroredLeftLegJacobian).norm() << "\n";
  }

  return 0;
}

}  // namespace legged_robot
}  // namespace ocs2

int main(int argc, char** argv) {
  return ocs2::legged_robot::runAudit(argc, argv);
}
