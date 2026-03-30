/******************************************************************************
Copyright (c) 2026. All rights reserved.
******************************************************************************/

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_hexbot_legged_robot/common/HexbotActuatorJointLimits.h>
#include <ocs2_hexbot_legged_robot/common/utils.h>
#include <ocs2_hexbot_legged_robot/LeggedRobotInterface.h>
#include <ocs2_hexbot_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <pinocchio/multibody/model.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "ocs2_msgs/srv/reset.hpp"
#include "ocs2_hexbot_legged_robot_ros/visualization/LeggedRobotVisualizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace ocs2;
using namespace legged_robot;

namespace {

constexpr scalar_t kPi = 3.14159265358979323846;
constexpr scalar_t kTwoPi = 2.0 * kPi;

scalar_t wrapAngleToPi(scalar_t angle) {
  scalar_t wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

Eigen::Matrix<scalar_t, 3, 1> quaternionToEulerZyxPrincipal(
    const Eigen::Quaterniond& quaternion) {
  const scalar_t sinYawCosPitch =
      2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y());
  const scalar_t cosYawCosPitch =
      1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                   quaternion.z() * quaternion.z());
  const scalar_t yaw = std::atan2(sinYawCosPitch, cosYawCosPitch);

  const scalar_t sinPitch =
      std::clamp(2.0 * (quaternion.w() * quaternion.y() -
                        quaternion.z() * quaternion.x()),
                 -1.0, 1.0);
  const scalar_t pitch = std::asin(sinPitch);

  const scalar_t sinRollCosPitch =
      2.0 * (quaternion.w() * quaternion.x() + quaternion.y() * quaternion.z());
  const scalar_t cosRollCosPitch =
      1.0 - 2.0 * (quaternion.x() * quaternion.x() +
                   quaternion.y() * quaternion.y());
  const scalar_t roll = std::atan2(sinRollCosPitch, cosRollCosPitch);

  Eigen::Matrix<scalar_t, 3, 1> eulerZyx;
  eulerZyx << wrapAngleToPi(yaw), pitch, wrapAngleToPi(roll);
  return eulerZyx;
}

bool allFinite(const vector_t& values) {
  return values.array().isFinite().all();
}

bool odomHasFiniteValues(const nav_msgs::msg::Odometry& odom) {
  const auto& position = odom.pose.pose.position;
  const auto& orientation = odom.pose.pose.orientation;
  const auto& linear = odom.twist.twist.linear;
  const auto& angular = odom.twist.twist.angular;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
         std::isfinite(orientation.w) && std::isfinite(linear.x) &&
         std::isfinite(linear.y) && std::isfinite(linear.z) &&
         std::isfinite(angular.x) && std::isfinite(angular.y) &&
         std::isfinite(angular.z);
}

scalar_t selectNearestEquivalentAngle(scalar_t rawAngle, scalar_t referenceAngle,
                                      scalar_t lowerLimit,
                                      scalar_t upperLimit) {
  const bool hasFiniteLimits =
      std::isfinite(lowerLimit) && std::isfinite(upperLimit) &&
      lowerLimit <= upperLimit;

  const scalar_t centeredAngle =
      rawAngle + std::round((referenceAngle - rawAngle) / kTwoPi) * kTwoPi;
  if (!hasFiniteLimits) {
    return centeredAngle;
  }

  scalar_t bestAngle = std::clamp(centeredAngle, lowerLimit, upperLimit);
  scalar_t bestDistance = std::abs(bestAngle - referenceAngle);
  const scalar_t centerShift = std::round((referenceAngle - rawAngle) / kTwoPi);

  for (int offset = -3; offset <= 3; ++offset) {
    const scalar_t candidate =
        rawAngle + (centerShift + static_cast<scalar_t>(offset)) * kTwoPi;
    if (candidate < lowerLimit - 1e-9 || candidate > upperLimit + 1e-9) {
      continue;
    }

    const scalar_t distance = std::abs(candidate - referenceAngle);
    if (distance < bestDistance) {
      bestAngle = candidate;
      bestDistance = distance;
    }
  }

  return bestAngle;
}

class HexbotRuntimeMrtNode {
 public:
  explicit HexbotRuntimeMrtNode(rclcpp::Node::SharedPtr node)
      : node_(std::move(node)) {
    taskFile_ = node_->declare_parameter<std::string>("taskFile", "");
    urdfFile_ = node_->declare_parameter<std::string>("urdfFile", "");
    referenceFile_ = node_->declare_parameter<std::string>("referenceFile", "");
    inputJointStatesTopic_ = node_->declare_parameter<std::string>(
        "jointStatesTopic", "/hexbot_mpc/input/joint_states");
    inputOdomTopic_ = node_->declare_parameter<std::string>(
        "odomTopic", "/hexbot_mpc/input/odom");
    backendJointCommandTopic_ = node_->declare_parameter<std::string>(
        "backendJointCommandTopic", "/hexbot_mpc/output/joint_command");
    observationTimeout_ =
        node_->declare_parameter<double>("observationTimeout", 0.50);
    resetRetryPeriod_ =
        node_->declare_parameter<double>("resetRetryPeriod", 1.00);
    holdTargetDuration_ =
        node_->declare_parameter<double>("holdTargetDuration", 0.25);
    policyExpiryTolerance_ =
        node_->declare_parameter<double>("policyExpiryTolerance", 0.05);
    lockResetHeightToObservation_ =
        node_->declare_parameter<bool>("lockResetHeightToObservation", true);
    equivalentAngleWrapResetThreshold_ =
        node_->declare_parameter<double>("equivalentAngleWrapResetThreshold",
                                         kPi);
    commandTrackingResetThreshold_ =
        node_->declare_parameter<double>("commandTrackingResetThreshold", 1.00);
    enableJointBranchConsistencyGuard_ =
        node_->declare_parameter<bool>("enableJointBranchConsistencyGuard",
                                       true);
    jointBranchGuardAlignmentThreshold_ = node_->declare_parameter<double>(
        "jointBranchGuardAlignmentThreshold", 0.15);
    jointBranchGuardCommandDeviationThreshold_ =
        node_->declare_parameter<double>(
            "jointBranchGuardCommandDeviationThreshold", 0.60);
    enableVisualization_ =
        node_->declare_parameter<bool>("enableVisualization", true);
    const auto coxaLowerLimit =
        node_->declare_parameter<double>("coxaJointLowerLimit", -1.05);
    const auto coxaUpperLimit =
        node_->declare_parameter<double>("coxaJointUpperLimit", 1.05);
    const auto femurLowerLimit =
        node_->declare_parameter<double>("femurJointLowerLimit", -1.00);
    const auto femurUpperLimit =
        node_->declare_parameter<double>("femurJointUpperLimit", 1.00);
    const auto tarsusLowerLimit =
        node_->declare_parameter<double>("tarsusJointLowerLimit", -1.35);
    const auto tarsusUpperLimit =
        node_->declare_parameter<double>("tarsusJointUpperLimit", 1.35);

    if (taskFile_.empty() || urdfFile_.empty() || referenceFile_.empty()) {
      throw std::runtime_error(
          "[HexbotRuntimeMrtNode] Parameters 'taskFile', 'urdfFile', and "
          "'referenceFile' are required.");
    }

    interface_ = std::make_unique<LeggedRobotInterface>(taskFile_, urdfFile_,
                                                        referenceFile_);
    const auto& info = interface_->getCentroidalModelInfo();
    defaultJointState_ =
        centroidal_model::getJointAngles(interface_->getInitialState(), info);
    nominalComHeight_ =
        centroidal_model::getBasePose(interface_->getInitialState(), info)(2);
    try {
      loadData::loadEigenMatrix(referenceFile_, "defaultJointState",
                                defaultJointState_);
    } catch (const std::exception& e) {
      maybeWarn("[HexbotRuntimeMrtNode] Failed to load defaultJointState from "
                "reference file. Falling back to task initialState.");
    }
    try {
      loadData::loadCppDataType(referenceFile_, "comHeight", nominalComHeight_);
    } catch (const std::exception& e) {
      maybeWarn("[HexbotRuntimeMrtNode] Failed to load comHeight from "
                "reference file. Falling back to task initialState.");
    }
    const auto& model = interface_->getPinocchioInterface().getModel();
    if (model.lowerPositionLimit.size() >=
            static_cast<Eigen::Index>(info.generalizedCoordinatesNum) &&
        model.upperPositionLimit.size() >=
            static_cast<Eigen::Index>(info.generalizedCoordinatesNum)) {
      observationJointLowerLimits_ =
          model.lowerPositionLimit.tail(info.actuatedDofNum);
      observationJointUpperLimits_ =
          model.upperPositionLimit.tail(info.actuatedDofNum);
    } else {
      observationJointLowerLimits_ =
          vector_t::Constant(info.actuatedDofNum, -kPi);
      observationJointUpperLimits_ =
          vector_t::Constant(info.actuatedDofNum, kPi);
    }
    jointLowerLimits_ = observationJointLowerLimits_;
    jointUpperLimits_ = observationJointUpperLimits_;
    for (size_t index = 0; index < interface_->modelSettings().jointNames.size();
         ++index) {
      const auto& jointName = interface_->modelSettings().jointNames[index];
      if (jointName.rfind("coxa_joint_", 0) == 0) {
        jointLowerLimits_(static_cast<Eigen::Index>(index)) =
            std::max(jointLowerLimits_(static_cast<Eigen::Index>(index)),
                     coxaLowerLimit);
        jointUpperLimits_(static_cast<Eigen::Index>(index)) =
            std::min(jointUpperLimits_(static_cast<Eigen::Index>(index)),
                     coxaUpperLimit);
      } else if (jointName.rfind("femur_joint_", 0) == 0) {
        jointLowerLimits_(static_cast<Eigen::Index>(index)) =
            std::max(jointLowerLimits_(static_cast<Eigen::Index>(index)),
                     femurLowerLimit);
        jointUpperLimits_(static_cast<Eigen::Index>(index)) =
            std::min(jointUpperLimits_(static_cast<Eigen::Index>(index)),
                     femurUpperLimit);
      } else if (jointName.rfind("tarsus_joint_", 0) == 0) {
        jointLowerLimits_(static_cast<Eigen::Index>(index)) =
            std::max(jointLowerLimits_(static_cast<Eigen::Index>(index)),
                     tarsusLowerLimit);
        jointUpperLimits_(static_cast<Eigen::Index>(index)) =
            std::min(jointUpperLimits_(static_cast<Eigen::Index>(index)),
                     tarsusUpperLimit);
      }
    }
    jointConstraintLowerLimits_ = jointLowerLimits_;
    jointConstraintUpperLimits_ = jointUpperLimits_;
    const auto [guardedLowerLimits, guardedUpperLimits] =
        getHexbotActuatorGuardedJointLimits(interface_->modelSettings().jointNames);
    for (Eigen::Index i = 0; i < jointConstraintLowerLimits_.size(); ++i) {
      jointConstraintLowerLimits_(i) =
          std::max(jointConstraintLowerLimits_(i), guardedLowerLimits(i));
      jointConstraintUpperLimits_(i) =
          std::min(jointConstraintUpperLimits_(i), guardedUpperLimits(i));
    }
    std::ostringstream limitStream;
    limitStream << "[HexbotRuntimeMrtNode] Applied Hexbot actuator angle "
                   "limits for equivalent-angle normalization: "
                << "coxa=[" << coxaLowerLimit << ", " << coxaUpperLimit
                << "] femur=[" << femurLowerLimit << ", " << femurUpperLimit
                << "] tarsus=[" << tarsusLowerLimit << ", " << tarsusUpperLimit
                << "], guard_band={coxa=" << kHexbotCoxaGuardBand
                << ", femur=" << kHexbotFemurGuardBand
                << ", tarsus=" << kHexbotTarsusGuardBand << "}";
    RCLCPP_INFO(node_->get_logger(), "%s", limitStream.str().c_str());
    RCLCPP_INFO(
        node_->get_logger(),
        "[HexbotRuntimeMrtNode] Observation joint-angle normalization uses "
        "Pinocchio model limits, while backend command safety keeps actuator "
        "guarded limits.");
    rbdConversions_ = std::make_unique<CentroidalModelRbdConversions>(
        interface_->getPinocchioInterface(),
        interface_->getCentroidalModelInfo());
    nominalBaseHeightAboveSupportPlane_ =
        computeNominalBaseHeightAboveSupportPlane();
    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot runtime nominal base height above support plane: %.4f m",
        nominalBaseHeightAboveSupportPlane_);

    mrt_ = std::make_unique<MRT_ROS_Interface>(kRobotName);
    mrt_->initRollout(&interface_->getRollout());
    mrt_->launchNodes(node_);

    if (enableVisualization_) {
      CentroidalModelPinocchioMapping pinocchioMapping(
          interface_->getCentroidalModelInfo());
      PinocchioEndEffectorKinematics endEffectorKinematics(
          interface_->getPinocchioInterface(), pinocchioMapping,
          interface_->modelSettings().contactNames3DoF);
      visualizer_ = std::make_shared<LeggedRobotVisualizer>(
          interface_->getPinocchioInterface(),
          interface_->getCentroidalModelInfo(), endEffectorKinematics,
          interface_->modelSettings().jointNames, node_);
    }

    backendJointCommandPublisher_ =
        node_->create_publisher<sensor_msgs::msg::JointState>(
            backendJointCommandTopic_, 10);
    mpcResetClient_ =
        node_->create_client<ocs2_msgs::srv::Reset>(std::string(kRobotName) +
                                                    "_mpc_reset");
    jointStatesSubscriber_ =
        node_->create_subscription<sensor_msgs::msg::JointState>(
            inputJointStatesTopic_, 20,
            [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
              std::lock_guard<std::mutex> lock(dataMutex_);
              latestJointState_ = *msg;
              hasJointState_ = true;
              latestJointStateArrival_ = std::chrono::steady_clock::now();
            });
    odomSubscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        inputOdomTopic_, 20,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(dataMutex_);
          latestOdom_ = *msg;
          hasOdom_ = true;
          latestOdomArrival_ = std::chrono::steady_clock::now();
        });

    const double controlRate =
        std::max<double>(interface_->mpcSettings().mrtDesiredFrequency_, 1.0);
    const auto period = std::chrono::duration<double>(1.0 / controlRate);
    controlTimer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { controlLoop(); });

    lastResetAttempt_ = std::chrono::steady_clock::now() -
                        std::chrono::duration_cast<
                            std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(resetRetryPeriod_));

    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot runtime MRT node ready. topics: %s + %s -> %s | policy prefix=%s",
        inputJointStatesTopic_.c_str(), inputOdomTopic_.c_str(),
        backendJointCommandTopic_.c_str(), kRobotName);
  }

 private:
  enum class ResetReason {
    Bootstrap,
    EmptyPolicy,
    ExpiredPolicy,
    NonFinitePolicy,
    SafetyReject,
  };

  static const char* resetReasonLabel(ResetReason reason) {
    switch (reason) {
      case ResetReason::Bootstrap:
        return "bootstrap";
      case ResetReason::EmptyPolicy:
        return "empty_policy";
      case ResetReason::ExpiredPolicy:
        return "expired_policy";
      case ResetReason::NonFinitePolicy:
        return "nonfinite_policy";
      case ResetReason::SafetyReject:
        return "safety_reject";
    }
    return "unknown";
  }

  void controlLoop() {
    SystemObservation currentObservation;
    if (!buildObservation(currentObservation)) {
      return;
    }

    if (!mpcResetIssued_) {
      nextResetReason_ = ResetReason::Bootstrap;
      tryResetMpc(currentObservation);
      return;
    }

    mrt_->setCurrentObservation(currentObservation);

    if (!policyActive_) {
      if (mrt_->updatePolicy()) {
        policyActive_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "Initial MPC policy received. Runtime MRT loop is active.");
      }
      return;
    }

    mrt_->updatePolicy();
    const auto& activePolicy = mrt_->getPolicy();
    if (activePolicy.timeTrajectory_.empty()) {
      invalidateActivePolicy(
          "[HexbotRuntimeMrtNode] Active policy is empty. Dropping backend "
          "command and requesting a reset.",
          ResetReason::EmptyPolicy);
      return;
    }

    const scalar_t activePlanEndTime = activePolicy.timeTrajectory_.back();
    if (currentObservation.time > activePlanEndTime + policyExpiryTolerance_) {
      invalidateActivePolicy(
          "[HexbotRuntimeMrtNode] Active policy expired before a new policy "
          "arrived. Dropping backend command and requesting a reset.",
          ResetReason::ExpiredPolicy);
      return;
    }

    vector_t optimizedState;
    vector_t optimizedInput;
    size_t plannedMode = lastPlannedMode_;
    try {
      mrt_->evaluatePolicy(currentObservation.time, currentObservation.state,
                           optimizedState, optimizedInput, plannedMode);
    } catch (const std::exception& e) {
      maybeWarn("[HexbotRuntimeMrtNode] Failed to evaluate policy: " +
                std::string(e.what()));
      return;
    }
    if (!allFinite(optimizedState) || !allFinite(optimizedInput)) {
      invalidateActivePolicy(
          "[HexbotRuntimeMrtNode] MPC returned non-finite state/input. "
          "Dropping backend command and requesting a reset.",
          ResetReason::NonFinitePolicy);
      return;
    }

    lastPlannedMode_ = plannedMode;
    const vector_t measuredJointAngles = centroidal_model::getJointAngles(
        currentObservation.state, interface_->getCentroidalModelInfo());
    maybeEmitSolverTrace(currentObservation, optimizedState, optimizedInput,
                         measuredJointAngles, plannedMode);
    publishJointCommand(currentObservation.time, optimizedState, optimizedInput,
                        measuredJointAngles, plannedMode);
    if (!policyActive_) {
      return;
    }

    if (visualizer_) {
      visualizer_->update(currentObservation, mrt_->getPolicy(),
                          mrt_->getCommand());
    }
  }

  bool buildObservation(SystemObservation& observation) {
    sensor_msgs::msg::JointState jointState;
    nav_msgs::msg::Odometry odom;
    std::chrono::steady_clock::time_point jointStateArrival;
    std::chrono::steady_clock::time_point odomArrival;
    {
      std::lock_guard<std::mutex> lock(dataMutex_);
      if (!hasJointState_ || !hasOdom_) {
        maybeWarn("[HexbotRuntimeMrtNode] Waiting for joint_states and odom.");
        return false;
      }
      jointState = latestJointState_;
      odom = latestOdom_;
      jointStateArrival = latestJointStateArrival_;
      odomArrival = latestOdomArrival_;
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - jointStateArrival).count() >
            observationTimeout_ ||
        std::chrono::duration<double>(now - odomArrival).count() >
            observationTimeout_) {
      maybeWarn("[HexbotRuntimeMrtNode] Runtime observations are stale.");
      return false;
    }
    if (!odomHasFiniteValues(odom)) {
      maybeWarn(
          "[HexbotRuntimeMrtNode] Ignoring non-finite odom observation.");
      return false;
    }

    vector_t orderedJointPositions;
    vector_t orderedJointVelocities;
    if (!extractOrderedJointState(jointState, orderedJointPositions,
                                  orderedJointVelocities)) {
      maybeWarn(
          "[HexbotRuntimeMrtNode] JointState does not contain the expected Hexbot joint names.");
      return false;
    }
    if (!allFinite(orderedJointPositions) || !allFinite(orderedJointVelocities)) {
      maybeWarn(
          "[HexbotRuntimeMrtNode] Ignoring non-finite joint state observation.");
      return false;
    }
    orderedJointPositions = normalizeObservationJointAngles(orderedJointPositions);

    observation.state = rbdConversions_->computeCentroidalStateFromRbdModel(
        createRbdState(odom, orderedJointPositions, orderedJointVelocities));
    if (!allFinite(observation.state)) {
      maybeWarn(
          "[HexbotRuntimeMrtNode] Computed a non-finite centroidal observation.");
      return false;
    }
    observation.input =
        vector_t::Zero(interface_->getCentroidalModelInfo().inputDim);
    centroidal_model::getJointVelocities(observation.input,
                                         interface_->getCentroidalModelInfo()) =
        orderedJointVelocities;
    if (!allFinite(observation.input)) {
      maybeWarn(
          "[HexbotRuntimeMrtNode] Computed a non-finite observation input.");
      return false;
    }
    observation.mode = lastPlannedMode_;
    observation.time = getObservationTime(odom, jointState);
    return true;
  }

  vector_t createRbdState(const nav_msgs::msg::Odometry& odom,
                          const vector_t& jointPositions,
                          const vector_t& jointVelocities) const {
    const auto& info = interface_->getCentroidalModelInfo();
    vector_t rbdState = vector_t::Zero(2 * info.generalizedCoordinatesNum);

    Eigen::Quaterniond orientation(
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    if (orientation.norm() < 1e-9) {
      orientation = Eigen::Quaterniond::Identity();
    } else {
      orientation.normalize();
    }
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
    const Eigen::Vector3d eulerZyx =
        quaternionToEulerZyxPrincipal(orientation);

    const Eigen::Vector3d linearBody(odom.twist.twist.linear.x,
                                     odom.twist.twist.linear.y,
                                     odom.twist.twist.linear.z);
    const Eigen::Vector3d angularBody(odom.twist.twist.angular.x,
                                      odom.twist.twist.angular.y,
                                      odom.twist.twist.angular.z);
    const Eigen::Vector3d linearWorld = rotation * linearBody;
    const Eigen::Vector3d angularWorld = rotation * angularBody;

    rbdState.segment<3>(0) = eulerZyx;
    rbdState.segment<3>(3) << odom.pose.pose.position.x, odom.pose.pose.position.y,
        odom.pose.pose.position.z;
    rbdState.segment(6, info.actuatedDofNum) = jointPositions;
    rbdState.segment<3>(info.generalizedCoordinatesNum) = angularWorld;
    rbdState.segment<3>(info.generalizedCoordinatesNum + 3) = linearWorld;
    rbdState.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum) =
        jointVelocities;
    return rbdState;
  }

  bool extractOrderedJointState(const sensor_msgs::msg::JointState& jointState,
                                vector_t& jointPositions,
                                vector_t& jointVelocities) const {
    const auto& jointNames = interface_->modelSettings().jointNames;
    std::unordered_map<std::string, size_t> nameToIndex;
    nameToIndex.reserve(jointState.name.size());
    for (size_t i = 0; i < jointState.name.size(); ++i) {
      nameToIndex.emplace(jointState.name[i], i);
    }

    jointPositions = vector_t::Zero(jointNames.size());
    jointVelocities = vector_t::Zero(jointNames.size());
    for (size_t i = 0; i < jointNames.size(); ++i) {
      const auto it = nameToIndex.find(jointNames[i]);
      if (it == nameToIndex.end() || it->second >= jointState.position.size()) {
        return false;
      }
      const size_t index = it->second;
      jointPositions(static_cast<Eigen::Index>(i)) = jointState.position[index];
      if (index < jointState.velocity.size()) {
        jointVelocities(static_cast<Eigen::Index>(i)) =
            jointState.velocity[index];
      }
    }
    return true;
  }

  vector_t normalizeObservationJointAngles(const vector_t& jointPositions) {
    if (jointPositions.size() == 0 ||
        observationJointLowerLimits_.size() != jointPositions.size() ||
        observationJointUpperLimits_.size() != jointPositions.size()) {
      return jointPositions;
    }

    vector_t referenceAngles = jointPositions;
    if (lastNormalizedObservationJointAngles_.size() == jointPositions.size()) {
      referenceAngles = lastNormalizedObservationJointAngles_;
    } else if (defaultJointState_.size() == jointPositions.size()) {
      referenceAngles = defaultJointState_;
    }

    vector_t normalizedJointPositions = jointPositions;
    scalar_t maxNormalizationDelta = 0.0;
    size_t maxNormalizationJointIndex = 0;
    scalar_t maxNormalizationRawAngle = 0.0;
    scalar_t maxNormalizationReferenceAngle = 0.0;
    scalar_t maxNormalizationObservedAngle = 0.0;
    size_t normalizedJointCount = 0;

    for (Eigen::Index i = 0; i < jointPositions.size(); ++i) {
      const scalar_t normalizedAngle = selectNearestEquivalentAngle(
          jointPositions(i), referenceAngles(i),
          observationJointLowerLimits_(i), observationJointUpperLimits_(i));
      normalizedJointPositions(i) = normalizedAngle;
      const scalar_t normalizationDelta = std::abs(normalizedAngle - jointPositions(i));
      if (normalizationDelta > 1e-6) {
        ++normalizedJointCount;
        if (normalizationDelta > maxNormalizationDelta) {
          maxNormalizationDelta = normalizationDelta;
          maxNormalizationJointIndex = static_cast<size_t>(i);
          maxNormalizationRawAngle = jointPositions(i);
          maxNormalizationReferenceAngle = referenceAngles(i);
          maxNormalizationObservedAngle = normalizedAngle;
        }
      }
    }

    if (normalizedJointCount > 0) {
      std::ostringstream stream;
      stream.setf(std::ios::fixed);
      stream.precision(4);
      stream << "[HexbotRuntimeMrtNode] Normalized observation joint angles to the "
                "nearest OCS2-constraint-feasible equivalent. normalized_joints="
             << normalizedJointCount
             << " max_delta_rad=" << maxNormalizationDelta
             << " joint=" << interface_->modelSettings().jointNames[maxNormalizationJointIndex]
             << " raw=" << maxNormalizationRawAngle
             << " reference=" << maxNormalizationReferenceAngle
             << " normalized=" << maxNormalizationObservedAngle;
      maybeWarn(stream.str());
    }

    lastNormalizedObservationJointAngles_ = normalizedJointPositions;
    return normalizedJointPositions;
  }

  scalar_t getObservationTime(const nav_msgs::msg::Odometry& odom,
                              const sensor_msgs::msg::JointState& jointState) const {
    scalar_t rawTime = node_->get_clock()->now().seconds();
    if (odom.header.stamp.sec != 0 || odom.header.stamp.nanosec != 0) {
      rawTime = rclcpp::Time(odom.header.stamp).seconds();
    } else if (jointState.header.stamp.sec != 0 ||
               jointState.header.stamp.nanosec != 0) {
      rawTime = rclcpp::Time(jointState.header.stamp).seconds();
    }

    if (!observationTimeOriginInitialized_) {
      observationTimeOrigin_ = rawTime;
      observationTimeOriginInitialized_ = true;
    }

    const scalar_t relativeTime = std::max<scalar_t>(
        0.0, rawTime - observationTimeOrigin_);
    lastObservationTime_ = std::max(lastObservationTime_, relativeTime);
    return lastObservationTime_;
  }

  void tryResetMpc(const SystemObservation& currentObservation) {
    if (resetRequestPending_) {
      if (resetResponseFuture_.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
        try {
          const auto response = resetResponseFuture_.get();
          resetRequestPending_ = false;
          if (response && response->done) {
            mpcResetIssued_ = true;
            if (inFlightResetReason_ == ResetReason::Bootstrap) {
              RCLCPP_INFO(node_->get_logger(),
                          "Bootstrap reset request acknowledged by MPC node. Waiting for "
                          "the first policy.");
            } else {
              RCLCPP_INFO(node_->get_logger(),
                          "Fault reset request acknowledged by MPC node. Waiting for "
                          "the first policy. reason=%s",
                          resetReasonLabel(inFlightResetReason_));
            }
          } else {
            maybeWarn(
                "[HexbotRuntimeMrtNode] MPC reset service returned done=false.");
          }
        } catch (const std::exception& e) {
          resetRequestPending_ = false;
          maybeWarn("[HexbotRuntimeMrtNode] MPC reset service failed: " +
                    std::string(e.what()));
        }
      }
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - lastResetAttempt_).count() <
        resetRetryPeriod_) {
      return;
    }
    lastResetAttempt_ = now;

    if (!mpcResetClient_) {
      maybeWarn("[HexbotRuntimeMrtNode] MPC reset client is not initialized.");
      return;
    }

    if (!mpcResetClient_->wait_for_service(std::chrono::seconds(0))) {
      maybeWarn(
          "[HexbotRuntimeMrtNode] Waiting for MPC reset service to appear.");
      return;
    }

    auto request = std::make_shared<ocs2_msgs::srv::Reset::Request>();
    request->reset = true;
    request->target_trajectories = ros_msg_conversions::createTargetTrajectoriesMsg(
        createHoldTargetTrajectories(currentObservation));
    inFlightResetReason_ = nextResetReason_;
    resetResponseFuture_ =
        mpcResetClient_->async_send_request(request).future.share();
    resetRequestPending_ = true;
    if (inFlightResetReason_ == ResetReason::Bootstrap) {
      RCLCPP_INFO(node_->get_logger(),
                  "Bootstrap reset request sent to MPC node. Waiting for acknowledgement.");
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "Fault reset request sent to MPC node. Waiting for acknowledgement. reason=%s",
                  resetReasonLabel(inFlightResetReason_));
    }
  }

  TargetTrajectories createHoldTargetTrajectories(
      const SystemObservation& currentObservation) {
    const vector_t desiredState =
        createNominalResetState(currentObservation.state);
    const scalar_array_t timeTrajectory{
        currentObservation.time, currentObservation.time + holdTargetDuration_};
    const vector_array_t stateTrajectory{desiredState, desiredState};
    const vector_array_t inputTrajectory{
        vector_t::Zero(interface_->getCentroidalModelInfo().inputDim),
        vector_t::Zero(interface_->getCentroidalModelInfo().inputDim)};
    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  vector_t createNominalResetState(const vector_t& currentState) {
    const auto& info = interface_->getCentroidalModelInfo();
    if (currentState.size() != static_cast<Eigen::Index>(info.stateDim)) {
      return currentState;
    }

    vector_t desiredState = vector_t::Zero(currentState.size());
    const auto currentBasePose =
        centroidal_model::getBasePose(currentState, info);
    auto desiredBasePose = centroidal_model::getBasePose(desiredState, info);
    desiredBasePose = currentBasePose;
    desiredBasePose(2) = lockResetHeightToObservation_
                             ? computeSupportPlaneRelativeResetHeight(currentState)
                             : nominalComHeight_;
    desiredBasePose(4) = 0.0;
    desiredBasePose(5) = 0.0;

    const auto jointCount = std::min<Eigen::Index>(
        static_cast<Eigen::Index>(defaultJointState_.size()), info.actuatedDofNum);
    if (jointCount > 0) {
      auto resetJointAngles = defaultJointState_.head(jointCount);
      const auto currentJointAngles =
          centroidal_model::getJointAngles(currentState, info).head(jointCount);
      if (currentJointAngles.array().isFinite().all()) {
        resetJointAngles = currentJointAngles;
      }
      centroidal_model::getJointAngles(desiredState, info).head(jointCount) =
          resetJointAngles;
    }

    return desiredState;
  }

  scalar_t computeSupportPlaneRelativeResetHeight(const vector_t& currentState) {
    const auto& info = interface_->getCentroidalModelInfo();
    if (currentState.size() != static_cast<Eigen::Index>(info.stateDim)) {
      return nominalComHeight_;
    }

    const auto footPositionsInWorld = computeFootPositionsInWorld(
        interface_->getPinocchioInterface(), info,
        interface_->modelSettings().contactNames3DoF, currentState);
    const auto supportPlane = latchSupportPlaneBaseline(
        estimateSupportPlaneFromLowestFeet(footPositionsInWorld));
    const auto basePose = centroidal_model::getBasePose(currentState, info);
    return solveSupportPlaneHeightAtXY(
        supportPlane, basePose(0), basePose(1),
        nominalBaseHeightAboveSupportPlane_);
  }

  SupportPlane latchSupportPlaneBaseline(
      const SupportPlane& candidateSupportPlane) {
    if (!latchedSupportPlaneValid_) {
      latchedSupportPlane_ = candidateSupportPlane;
      latchedSupportPlaneValid_ = true;
      return latchedSupportPlane_;
    }

    if (candidateSupportPlane.heightAlongNormal >
        latchedSupportPlane_.heightAlongNormal) {
      latchedSupportPlane_ = candidateSupportPlane;
      return latchedSupportPlane_;
    }

    SupportPlane stabilizedSupportPlane = candidateSupportPlane;
    stabilizedSupportPlane.normalInWorld = latchedSupportPlane_.normalInWorld;
    stabilizedSupportPlane.worldToTerrainRotation =
        latchedSupportPlane_.worldToTerrainRotation;
    stabilizedSupportPlane.heightAlongNormal =
        latchedSupportPlane_.heightAlongNormal;
    stabilizedSupportPlane.numSupportFeet = candidateSupportPlane.numSupportFeet;
    return stabilizedSupportPlane;
  }

  scalar_t computeNominalBaseHeightAboveSupportPlane() {
    const auto& info = interface_->getCentroidalModelInfo();
    vector_t nominalState = vector_t::Zero(info.stateDim);
    auto nominalBasePose = centroidal_model::getBasePose(nominalState, info);
    nominalBasePose << 0.0, 0.0, nominalComHeight_, 0.0, 0.0, 0.0;

    const auto jointCount = std::min<Eigen::Index>(
        static_cast<Eigen::Index>(defaultJointState_.size()), info.actuatedDofNum);
    if (jointCount > 0) {
      centroidal_model::getJointAngles(nominalState, info).head(jointCount) =
          defaultJointState_.head(jointCount);
    }

    const auto footPositionsInWorld = computeFootPositionsInWorld(
        interface_->getPinocchioInterface(), info,
        interface_->modelSettings().contactNames3DoF, nominalState);
    const auto supportPlane =
        estimateSupportPlaneFromLowestFeet(footPositionsInWorld);
    return signedDistanceToSupportPlane(vector3_t(0.0, 0.0, nominalComHeight_),
                                        supportPlane);
  }

  void invalidateActivePolicy(const std::string& message, ResetReason reason) {
    maybeWarn(message);
    mrt_->reset();
    policyActive_ = false;
    mpcResetIssued_ = false;
    resetRequestPending_ = false;
    nextResetReason_ = reason;
    lastResetAttempt_ = std::chrono::steady_clock::now() -
                        std::chrono::duration_cast<
                            std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(resetRetryPeriod_));
  }

  struct JointDeltaSummary {
    scalar_t avgAbsDelta = 0.0;
    scalar_t maxAbsDelta = 0.0;
    size_t jointIndex = 0;
    scalar_t leftValue = 0.0;
    scalar_t rightValue = 0.0;
    size_t jointCount = 0;
  };

  struct StanceForceSummary {
    scalar_t maxRatio = std::numeric_limits<scalar_t>::quiet_NaN();
    std::vector<std::pair<std::string, scalar_t>> normalForces;
  };

  struct StanceLegForceComponents {
    std::string legName;
    vector3_t forceInWorld = vector3_t::Zero();
    vector3_t forceInTerrain = vector3_t::Zero();
    scalar_t tangentialNorm = 0.0;
  };

  struct FootDeltaSummary {
    scalar_t meanPositionError = 0.0;
    scalar_t maxPositionError = 0.0;
    size_t maxPositionLegIndex = 0;
    scalar_t maxAbsZDelta = 0.0;
    scalar_t signedZDeltaAtMaxAbs = 0.0;
    size_t maxAbsZLegIndex = 0;
    scalar_t minNormalDelta = std::numeric_limits<scalar_t>::quiet_NaN();
    size_t minNormalDeltaLegIndex = 0;
    scalar_t leftNormalDistAtMinDelta = 0.0;
    scalar_t rightNormalDistAtMinDelta = 0.0;
    size_t footCount = 0;
  };

  struct StanceAnchorSummary {
    scalar_t targetMeanPositionError = 0.0;
    scalar_t targetMaxPositionError = 0.0;
    size_t targetMaxLegIndex = 0;
    scalar_t currentMeanPositionError = 0.0;
    scalar_t currentMaxPositionError = 0.0;
    size_t currentMaxLegIndex = 0;
    scalar_t optimizedMeanPositionError = 0.0;
    scalar_t optimizedMaxPositionError = 0.0;
    size_t optimizedMaxLegIndex = 0;
    size_t anchoredFootCount = 0;
  };

  feet_array_t<scalar_t> computeTouchdownAnchorErrors(
      const contact_flag_t& contactFlags,
      const feet_array_t<vector3_t>& footPositionsInWorld) const {
    feet_array_t<scalar_t> errors;
    errors.fill(std::numeric_limits<scalar_t>::quiet_NaN());
    for (size_t index = 0; index < contactFlags.size(); ++index) {
      if (!contactFlags[index] || !hasTouchdownAnchor_[index]) {
        continue;
      }
      const auto& anchorPosition = touchdownAnchorPositionsInWorld_[index];
      if (!anchorPosition.allFinite() || !footPositionsInWorld[index].allFinite()) {
        continue;
      }
      errors[index] = (footPositionsInWorld[index] - anchorPosition).norm();
    }
    return errors;
  }

  JointDeltaSummary summarizeJointDelta(const vector_t& leftJointAngles,
                                        const vector_t& rightJointAngles) const {
    JointDeltaSummary summary;
    const auto jointCount = static_cast<size_t>(
        std::min(leftJointAngles.size(), rightJointAngles.size()));
    if (jointCount == 0) {
      return summary;
    }

    scalar_t deltaSum = 0.0;
    for (size_t index = 0; index < jointCount; ++index) {
      const scalar_t leftValue =
          leftJointAngles(static_cast<Eigen::Index>(index));
      const scalar_t rightValue =
          rightJointAngles(static_cast<Eigen::Index>(index));
      const scalar_t absDelta = std::abs(leftValue - rightValue);
      deltaSum += absDelta;
      if (absDelta > summary.maxAbsDelta) {
        summary.maxAbsDelta = absDelta;
        summary.jointIndex = index;
        summary.leftValue = leftValue;
        summary.rightValue = rightValue;
      }
    }

    summary.jointCount = jointCount;
    summary.avgAbsDelta = deltaSum / static_cast<scalar_t>(jointCount);
    return summary;
  }

  SupportPlane estimateSupportPlaneForState(
      const vector_t& state, const contact_flag_t& contactFlags) const {
    const auto footPositionsInWorld = computeFootPositionsInWorld(
        interface_->getPinocchioInterface(),
        interface_->getCentroidalModelInfo(),
        interface_->modelSettings().contactNames3DoF, state);
    auto supportPlane = estimateSupportPlane(footPositionsInWorld, contactFlags);
    if (supportPlane.numSupportFeet < 3) {
      supportPlane = estimateSupportPlaneFromLowestFeet(footPositionsInWorld);
    }
    return supportPlane;
  }

  scalar_t computeBaseHeightAboveSupportPlane(
      const vector_t& state, const contact_flag_t& contactFlags) const {
    const auto& info = interface_->getCentroidalModelInfo();
    if (state.size() != static_cast<Eigen::Index>(info.stateDim)) {
      return std::numeric_limits<scalar_t>::quiet_NaN();
    }

    const auto supportPlane = estimateSupportPlaneForState(state, contactFlags);
    const auto basePose = centroidal_model::getBasePose(state, info);
    return signedDistanceToSupportPlane(basePose.head<3>(), supportPlane);
  }

  FootDeltaSummary summarizeFootDelta(
      const feet_array_t<vector3_t>& leftFootPositions,
      const feet_array_t<vector3_t>& rightFootPositions,
      const SupportPlane& referenceSupportPlane) const {
    FootDeltaSummary summary;
    scalar_t totalPositionError = 0.0;
    bool hasNormalDelta = false;

    const auto footCount =
        std::min(leftFootPositions.size(), rightFootPositions.size());
    for (size_t index = 0; index < footCount; ++index) {
      const auto& leftFootPosition = leftFootPositions[index];
      const auto& rightFootPosition = rightFootPositions[index];
      if (!leftFootPosition.allFinite() || !rightFootPosition.allFinite()) {
        continue;
      }

      const vector3_t footDelta = leftFootPosition - rightFootPosition;
      const scalar_t positionError = footDelta.norm();
      totalPositionError += positionError;
      ++summary.footCount;

      if (positionError > summary.maxPositionError) {
        summary.maxPositionError = positionError;
        summary.maxPositionLegIndex = index;
      }

      const scalar_t zDelta = footDelta.z();
      if (std::abs(zDelta) > summary.maxAbsZDelta) {
        summary.maxAbsZDelta = std::abs(zDelta);
        summary.signedZDeltaAtMaxAbs = zDelta;
        summary.maxAbsZLegIndex = index;
      }

      const scalar_t rightNormalDist =
          signedDistanceToSupportPlane(rightFootPosition, referenceSupportPlane);
      const scalar_t leftNormalDist =
          signedDistanceToSupportPlane(leftFootPosition, referenceSupportPlane);
      const scalar_t normalDelta = leftNormalDist - rightNormalDist;
      if (!hasNormalDelta || normalDelta < summary.minNormalDelta) {
        hasNormalDelta = true;
        summary.minNormalDelta = normalDelta;
        summary.minNormalDeltaLegIndex = index;
        summary.leftNormalDistAtMinDelta = leftNormalDist;
        summary.rightNormalDistAtMinDelta = rightNormalDist;
      }
    }

    if (summary.footCount > 0) {
      summary.meanPositionError =
          totalPositionError / static_cast<scalar_t>(summary.footCount);
    }
    return summary;
  }

  void updateTouchdownAnchors(
      const contact_flag_t& contactFlags,
      const feet_array_t<vector3_t>& measuredFootPositions) {
    for (size_t index = 0; index < contactFlags.size(); ++index) {
      const bool isInContact = contactFlags[index];
      const bool wasInContact = touchdownAnchorContactFlags_[index];
      if (isInContact && (!wasInContact || !hasTouchdownAnchor_[index]) &&
          measuredFootPositions[index].allFinite()) {
        touchdownAnchorPositionsInWorld_[index] = measuredFootPositions[index];
        hasTouchdownAnchor_[index] = true;
      } else if (!isInContact) {
        hasTouchdownAnchor_[index] = false;
      }
      touchdownAnchorContactFlags_[index] = isInContact;
    }
  }

  StanceAnchorSummary summarizeStanceAnchorDrift(
      const contact_flag_t& contactFlags,
      const feet_array_t<vector3_t>& targetFootPositions,
      const feet_array_t<vector3_t>& currentFootPositions,
      const feet_array_t<vector3_t>& optimizedFootPositions) const {
    StanceAnchorSummary summary;
    scalar_t targetErrorSum = 0.0;
    scalar_t currentErrorSum = 0.0;
    scalar_t optimizedErrorSum = 0.0;

    for (size_t index = 0; index < contactFlags.size(); ++index) {
      if (!contactFlags[index] || !hasTouchdownAnchor_[index]) {
        continue;
      }

      const auto& anchorPosition = touchdownAnchorPositionsInWorld_[index];
      if (!anchorPosition.allFinite() || !targetFootPositions[index].allFinite() ||
          !currentFootPositions[index].allFinite() ||
          !optimizedFootPositions[index].allFinite()) {
        continue;
      }

      const scalar_t targetError =
          (targetFootPositions[index] - anchorPosition).norm();
      const scalar_t currentError =
          (currentFootPositions[index] - anchorPosition).norm();
      const scalar_t optimizedError =
          (optimizedFootPositions[index] - anchorPosition).norm();

      targetErrorSum += targetError;
      currentErrorSum += currentError;
      optimizedErrorSum += optimizedError;
      ++summary.anchoredFootCount;

      if (targetError > summary.targetMaxPositionError) {
        summary.targetMaxPositionError = targetError;
        summary.targetMaxLegIndex = index;
      }
      if (currentError > summary.currentMaxPositionError) {
        summary.currentMaxPositionError = currentError;
        summary.currentMaxLegIndex = index;
      }
      if (optimizedError > summary.optimizedMaxPositionError) {
        summary.optimizedMaxPositionError = optimizedError;
        summary.optimizedMaxLegIndex = index;
      }
    }

    if (summary.anchoredFootCount > 0) {
      const scalar_t count = static_cast<scalar_t>(summary.anchoredFootCount);
      summary.targetMeanPositionError = targetErrorSum / count;
      summary.currentMeanPositionError = currentErrorSum / count;
      summary.optimizedMeanPositionError = optimizedErrorSum / count;
    }

    return summary;
  }

  vector_t interpolateTargetStateAtTime(scalar_t time) const {
    const auto& targetTrajectories = mrt_->getCommand().mpcTargetTrajectories_;
    if (targetTrajectories.stateTrajectory.empty()) {
      return {};
    }
    if (targetTrajectories.stateTrajectory.size() == 1 ||
        targetTrajectories.timeTrajectory.size() !=
            targetTrajectories.stateTrajectory.size()) {
      return targetTrajectories.stateTrajectory.front();
    }

    const scalar_t clampedTime =
        std::clamp(time, targetTrajectories.timeTrajectory.front(),
                   targetTrajectories.timeTrajectory.back());
    return LinearInterpolation::interpolate(clampedTime,
                                            targetTrajectories.timeTrajectory,
                                            targetTrajectories.stateTrajectory);
  }

  StanceForceSummary summarizeStanceForces(
      const vector_t& optimizedInput, const contact_flag_t& contactFlags,
      const SupportPlane& supportPlane) const {
    StanceForceSummary summary;
    const auto& info = interface_->getCentroidalModelInfo();
    if (optimizedInput.size() < static_cast<Eigen::Index>(info.inputDim)) {
      return summary;
    }

    std::vector<scalar_t> positiveNormalForces;
    for (size_t index = 0; index < contactFlags.size(); ++index) {
      if (!contactFlags[index]) {
        continue;
      }

      const vector3_t contactForceInWorld =
          centroidal_model::getContactForces(optimizedInput, index, info);
      const scalar_t normalForce =
          supportPlane.normalInWorld.dot(contactForceInWorld);
      summary.normalForces.emplace_back(
          interface_->modelSettings().contactNames3DoF[index], normalForce);
      if (normalForce > 1.0e-6) {
        positiveNormalForces.push_back(normalForce);
      }
    }

    if (positiveNormalForces.size() >= 2) {
      const auto [minIt, maxIt] = std::minmax_element(
          positiveNormalForces.begin(), positiveNormalForces.end());
      summary.maxRatio = *maxIt / std::max<scalar_t>(*minIt, 1.0e-6);
    }
    return summary;
  }

  std::vector<StanceLegForceComponents> collectStanceForceComponents(
      const vector_t& input, const contact_flag_t& contactFlags,
      const SupportPlane& supportPlane) const {
    std::vector<StanceLegForceComponents> entries;
    const auto& info = interface_->getCentroidalModelInfo();
    if (input.size() < static_cast<Eigen::Index>(info.inputDim)) {
      return entries;
    }

    for (size_t index = 0; index < contactFlags.size(); ++index) {
      if (!contactFlags[index]) {
        continue;
      }

      StanceLegForceComponents entry;
      entry.legName = interface_->modelSettings().contactNames3DoF[index];
      entry.forceInWorld = centroidal_model::getContactForces(input, index, info);
      entry.forceInTerrain = supportPlane.worldToTerrainRotation * entry.forceInWorld;
      entry.tangentialNorm = entry.forceInTerrain.head<2>().norm();
      entries.push_back(entry);
    }

    return entries;
  }

  void maybeEmitSolverTrace(const SystemObservation& currentObservation,
                            const vector_t& optimizedState,
                            const vector_t& optimizedInput,
                            const vector_t& measuredJointAngles,
                            size_t plannedMode) {
    const bool observationRewound =
        currentObservation.time + 1.0e-6 < lastSolverTraceObservationTime_;
    const bool modeChanged = plannedMode != lastSolverTraceMode_;
    const bool periodicTrace =
        currentObservation.time - lastSolverTraceObservationTime_ >= 0.5;
    if (!observationRewound && !modeChanged && !periodicTrace) {
      return;
    }

    lastSolverTraceObservationTime_ = currentObservation.time;
    lastSolverTraceMode_ = plannedMode;

    const auto& info = interface_->getCentroidalModelInfo();
    const auto& jointNames = interface_->modelSettings().jointNames;
    const auto contactFlags = modeNumber2StanceLeg(plannedMode);
    const auto currentBasePose =
        centroidal_model::getBasePose(currentObservation.state, info);
    const auto currentSupportPlane =
        estimateSupportPlaneForState(currentObservation.state, contactFlags);
    const auto optimizedBasePose =
        centroidal_model::getBasePose(optimizedState, info);
    const scalar_t currentBaseHeightAboveSupport =
        signedDistanceToSupportPlane(currentBasePose.head<3>(), currentSupportPlane);
    const auto optimizedSupportPlane =
        estimateSupportPlaneForState(optimizedState, contactFlags);
    const scalar_t optimizedBaseHeightAboveSupport =
        signedDistanceToSupportPlane(optimizedBasePose.head<3>(),
                                     optimizedSupportPlane);
    const auto optimizedJointAngles =
        centroidal_model::getJointAngles(optimizedState, info);
    const auto optimizedVsMeasured =
        summarizeJointDelta(optimizedJointAngles, measuredJointAngles);
    const auto currentFootPositions = computeFootPositionsInWorld(
        interface_->getPinocchioInterface(), info,
        interface_->modelSettings().contactNames3DoF, currentObservation.state);
    const auto optimizedFootPositions = computeFootPositionsInWorld(
        interface_->getPinocchioInterface(), info,
        interface_->modelSettings().contactNames3DoF, optimizedState);

    scalar_t targetStartBaseZ = std::numeric_limits<scalar_t>::quiet_NaN();
    scalar_t targetGoalBaseZ = std::numeric_limits<scalar_t>::quiet_NaN();
    scalar_t targetInterpolatedBaseZ = std::numeric_limits<scalar_t>::quiet_NaN();
    scalar_t targetBaseHeightAboveSupport =
        std::numeric_limits<scalar_t>::quiet_NaN();
    scalar_t currentMinusTargetSupportHeight =
        std::numeric_limits<scalar_t>::quiet_NaN();
    scalar_t optimizedMinusTargetSupportHeight =
        std::numeric_limits<scalar_t>::quiet_NaN();
    JointDeltaSummary measuredVsTarget;
    JointDeltaSummary optimizedVsTarget;
    FootDeltaSummary currentVsTargetFeet;
    FootDeltaSummary optimizedVsTargetFeet;
    FootDeltaSummary optimizedVsCurrentFeet;
    StanceAnchorSummary stanceAnchorSummary;
    feet_array_t<scalar_t> targetAnchorErrors;
    feet_array_t<scalar_t> currentAnchorErrors;
    feet_array_t<scalar_t> optimizedAnchorErrors;
    targetAnchorErrors.fill(std::numeric_limits<scalar_t>::quiet_NaN());
    currentAnchorErrors.fill(std::numeric_limits<scalar_t>::quiet_NaN());
    optimizedAnchorErrors.fill(std::numeric_limits<scalar_t>::quiet_NaN());
    vector3_t nominalDesiredHorizontalForce = vector3_t::Zero();
    vector_t nominalInput = vector_t::Zero(optimizedInput.size());
    SupportPlane nominalSupportPlane;
    const auto& targetTrajectories = mrt_->getCommand().mpcTargetTrajectories_;
    if (!targetTrajectories.stateTrajectory.empty()) {
      const auto& startState = targetTrajectories.stateTrajectory.front();
      if (startState.size() == static_cast<Eigen::Index>(info.stateDim)) {
        targetStartBaseZ = centroidal_model::getBasePose(startState, info)(2);
      }

      const auto& goalState = targetTrajectories.stateTrajectory.back();
      if (goalState.size() == static_cast<Eigen::Index>(info.stateDim)) {
        targetGoalBaseZ = centroidal_model::getBasePose(goalState, info)(2);
      }

      const vector_t targetState =
          interpolateTargetStateAtTime(currentObservation.time);
      if (targetState.size() == static_cast<Eigen::Index>(info.stateDim)) {
        targetInterpolatedBaseZ =
            centroidal_model::getBasePose(targetState, info)(2);
        const auto targetSupportPlane =
            estimateSupportPlaneForState(targetState, contactFlags);
        targetBaseHeightAboveSupport =
            signedDistanceToSupportPlane(
                centroidal_model::getBasePose(targetState, info).head<3>(),
                targetSupportPlane);
        currentMinusTargetSupportHeight =
            currentSupportPlane.heightAlongNormal -
            targetSupportPlane.heightAlongNormal;
        optimizedMinusTargetSupportHeight =
            optimizedSupportPlane.heightAlongNormal -
            targetSupportPlane.heightAlongNormal;
        measuredVsTarget = summarizeJointDelta(
            measuredJointAngles,
            centroidal_model::getJointAngles(targetState, info));
        optimizedVsTarget = summarizeJointDelta(
            optimizedJointAngles,
            centroidal_model::getJointAngles(targetState, info));
        const auto targetFootPositions = computeFootPositionsInWorld(
            interface_->getPinocchioInterface(), info,
            interface_->modelSettings().contactNames3DoF, targetState);
        updateTouchdownAnchors(contactFlags, currentFootPositions);
        currentVsTargetFeet = summarizeFootDelta(
            currentFootPositions, targetFootPositions, targetSupportPlane);
        optimizedVsTargetFeet = summarizeFootDelta(
            optimizedFootPositions, targetFootPositions, targetSupportPlane);
        optimizedVsCurrentFeet = summarizeFootDelta(
            optimizedFootPositions, currentFootPositions, targetSupportPlane);
        stanceAnchorSummary = summarizeStanceAnchorDrift(
            contactFlags, targetFootPositions, currentFootPositions,
            optimizedFootPositions);
        targetAnchorErrors =
            computeTouchdownAnchorErrors(contactFlags, targetFootPositions);
        currentAnchorErrors =
            computeTouchdownAnchorErrors(contactFlags, currentFootPositions);
        optimizedAnchorErrors =
            computeTouchdownAnchorErrors(contactFlags, optimizedFootPositions);
        nominalSupportPlane = targetSupportPlane;
        nominalDesiredHorizontalForce = computeDesiredHorizontalForce(
            info, targetState, currentObservation.state,
            interface_->modelSettings().propulsionForceResponseTime,
            interface_->modelSettings().propulsionForceScale,
            interface_->modelSettings().propulsionForceMaxAcceleration,
            interface_->modelSettings().movingForceDeadzoneVelocity);
        nominalInput = weightCompensatingInput(
            info, contactFlags, targetFootPositions,
            approximateBasePositionInWorld(info, targetState), nominalSupportPlane,
            nominalDesiredHorizontalForce);
      }
    }

    const auto stanceForceSummary =
        summarizeStanceForces(optimizedInput, contactFlags, optimizedSupportPlane);
    const auto optimizedStanceForceComponents = collectStanceForceComponents(
        optimizedInput, contactFlags, optimizedSupportPlane);
    const auto nominalStanceForceComponents = collectStanceForceComponents(
        nominalInput, contactFlags, nominalSupportPlane);

    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(6);
    bool firstField = true;
    const auto appendFieldPrefix = [&stream, &firstField](const std::string& key) {
      if (!firstField) {
        stream << ",";
      }
      firstField = false;
      stream << "\"" << key << "\":";
    };
    const auto appendScalarOrNull =
        [&appendFieldPrefix, &stream](const std::string& key, scalar_t value) {
          appendFieldPrefix(key);
          if (std::isfinite(value)) {
            stream << value;
          } else {
            stream << "null";
          }
        };
    const auto appendSizeT =
        [&appendFieldPrefix, &stream](const std::string& key, size_t value) {
          appendFieldPrefix(key);
          stream << value;
        };
    const auto appendString =
        [&appendFieldPrefix, &stream](const std::string& key,
                                      const std::string& value) {
          appendFieldPrefix(key);
          stream << "\"" << value << "\"";
        };

    stream << "HEXBOT_SOLVER_TRACE {";
    appendScalarOrNull("time", currentObservation.time);
    appendSizeT("planned_mode", plannedMode);
    appendScalarOrNull("current_base_z", currentBasePose(2));
    appendScalarOrNull("optimized_base_z", optimizedBasePose(2));
    appendScalarOrNull("target_start_base_z", targetStartBaseZ);
    appendScalarOrNull("target_goal_base_z", targetGoalBaseZ);
    appendScalarOrNull("target_interpolated_base_z", targetInterpolatedBaseZ);
    appendScalarOrNull("current_base_height_above_support",
                       currentBaseHeightAboveSupport);
    appendScalarOrNull("optimized_base_height_above_support",
                       optimizedBaseHeightAboveSupport);
    appendScalarOrNull("target_base_height_above_support",
                       targetBaseHeightAboveSupport);
    appendScalarOrNull("current_minus_target_support_height",
                       currentMinusTargetSupportHeight);
    appendScalarOrNull("optimized_minus_target_support_height",
                       optimizedMinusTargetSupportHeight);
    appendScalarOrNull("optimized_vs_measured_joint_avg_abs_delta",
                       optimizedVsMeasured.avgAbsDelta);
    appendScalarOrNull("optimized_vs_measured_joint_max_abs_delta",
                       optimizedVsMeasured.maxAbsDelta);
    appendString("optimized_vs_measured_joint",
                 optimizedVsMeasured.jointCount > 0
                     ? jointNames[optimizedVsMeasured.jointIndex]
                     : std::string());
    appendScalarOrNull("optimized_vs_measured_joint_value",
                       optimizedVsMeasured.leftValue);
    appendScalarOrNull("measured_joint_value",
                       optimizedVsMeasured.rightValue);
    appendScalarOrNull("optimized_vs_target_joint_avg_abs_delta",
                       optimizedVsTarget.avgAbsDelta);
    appendScalarOrNull("optimized_vs_target_joint_max_abs_delta",
                       optimizedVsTarget.maxAbsDelta);
    appendString("optimized_vs_target_joint",
                 optimizedVsTarget.jointCount > 0
                     ? jointNames[optimizedVsTarget.jointIndex]
                     : std::string());
    appendScalarOrNull("optimized_target_joint_value",
                       optimizedVsTarget.leftValue);
    appendScalarOrNull("target_joint_value",
                       optimizedVsTarget.rightValue);
    appendScalarOrNull("measured_vs_target_joint_avg_abs_delta",
                       measuredVsTarget.avgAbsDelta);
    appendScalarOrNull("measured_vs_target_joint_max_abs_delta",
                       measuredVsTarget.maxAbsDelta);
    appendString("measured_vs_target_joint",
                 measuredVsTarget.jointCount > 0
                     ? jointNames[measuredVsTarget.jointIndex]
                     : std::string());
    appendScalarOrNull("measured_vs_target_measured_joint_value",
                       measuredVsTarget.leftValue);
    appendScalarOrNull("measured_vs_target_target_joint_value",
                       measuredVsTarget.rightValue);
    appendScalarOrNull("current_vs_target_foot_mean_position_error",
                       currentVsTargetFeet.meanPositionError);
    appendScalarOrNull("current_vs_target_foot_max_position_error",
                       currentVsTargetFeet.maxPositionError);
    appendString(
        "current_vs_target_foot_max_position_leg",
        currentVsTargetFeet.footCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[currentVsTargetFeet.maxPositionLegIndex]
            : std::string());
    appendScalarOrNull("current_vs_target_foot_max_abs_z_delta",
                       currentVsTargetFeet.maxAbsZDelta);
    appendString(
        "current_vs_target_foot_max_abs_z_leg",
        currentVsTargetFeet.footCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[currentVsTargetFeet.maxAbsZLegIndex]
            : std::string());
    appendScalarOrNull("current_vs_target_foot_signed_z_delta",
                       currentVsTargetFeet.signedZDeltaAtMaxAbs);
    appendScalarOrNull("current_vs_target_foot_most_negative_normal_delta",
                       currentVsTargetFeet.minNormalDelta);
    appendString(
        "current_vs_target_foot_most_negative_leg",
        std::isfinite(currentVsTargetFeet.minNormalDelta)
            ? interface_->modelSettings()
                  .contactNames3DoF[currentVsTargetFeet.minNormalDeltaLegIndex]
            : std::string());
    appendScalarOrNull(
        "current_vs_target_foot_worst_normal_current_dist",
        currentVsTargetFeet.leftNormalDistAtMinDelta);
    appendScalarOrNull("current_vs_target_foot_worst_normal_target_dist",
                       currentVsTargetFeet.rightNormalDistAtMinDelta);
    appendScalarOrNull("optimized_vs_target_foot_mean_position_error",
                       optimizedVsTargetFeet.meanPositionError);
    appendScalarOrNull("optimized_vs_target_foot_max_position_error",
                       optimizedVsTargetFeet.maxPositionError);
    appendString(
        "optimized_vs_target_foot_max_position_leg",
        optimizedVsTargetFeet.footCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[optimizedVsTargetFeet.maxPositionLegIndex]
            : std::string());
    appendScalarOrNull("optimized_vs_target_foot_max_abs_z_delta",
                       optimizedVsTargetFeet.maxAbsZDelta);
    appendString(
        "optimized_vs_target_foot_max_abs_z_leg",
        optimizedVsTargetFeet.footCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[optimizedVsTargetFeet.maxAbsZLegIndex]
            : std::string());
    appendScalarOrNull("optimized_vs_target_foot_signed_z_delta",
                       optimizedVsTargetFeet.signedZDeltaAtMaxAbs);
    appendScalarOrNull("optimized_vs_target_foot_most_negative_normal_delta",
                       optimizedVsTargetFeet.minNormalDelta);
    appendString(
        "optimized_vs_target_foot_most_negative_leg",
        std::isfinite(optimizedVsTargetFeet.minNormalDelta)
            ? interface_->modelSettings()
                  .contactNames3DoF[optimizedVsTargetFeet.minNormalDeltaLegIndex]
            : std::string());
    appendScalarOrNull(
        "optimized_vs_target_foot_worst_normal_optimized_dist",
        optimizedVsTargetFeet.leftNormalDistAtMinDelta);
    appendScalarOrNull("optimized_vs_target_foot_worst_normal_target_dist",
                       optimizedVsTargetFeet.rightNormalDistAtMinDelta);
    appendScalarOrNull("optimized_vs_current_foot_mean_position_error",
                       optimizedVsCurrentFeet.meanPositionError);
    appendScalarOrNull("optimized_vs_current_foot_max_position_error",
                       optimizedVsCurrentFeet.maxPositionError);
    appendString(
        "optimized_vs_current_foot_max_position_leg",
        optimizedVsCurrentFeet.footCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[optimizedVsCurrentFeet.maxPositionLegIndex]
            : std::string());
    appendScalarOrNull("optimized_vs_current_foot_max_abs_z_delta",
                       optimizedVsCurrentFeet.maxAbsZDelta);
    appendString(
        "optimized_vs_current_foot_max_abs_z_leg",
        optimizedVsCurrentFeet.footCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[optimizedVsCurrentFeet.maxAbsZLegIndex]
            : std::string());
    appendScalarOrNull("optimized_vs_current_foot_signed_z_delta",
                       optimizedVsCurrentFeet.signedZDeltaAtMaxAbs);
    appendScalarOrNull("optimized_vs_current_foot_most_negative_normal_delta",
                       optimizedVsCurrentFeet.minNormalDelta);
    appendString(
        "optimized_vs_current_foot_most_negative_leg",
        std::isfinite(optimizedVsCurrentFeet.minNormalDelta)
            ? interface_->modelSettings()
                  .contactNames3DoF[optimizedVsCurrentFeet.minNormalDeltaLegIndex]
            : std::string());
    appendScalarOrNull(
        "optimized_vs_current_foot_worst_normal_optimized_dist",
        optimizedVsCurrentFeet.leftNormalDistAtMinDelta);
    appendScalarOrNull("optimized_vs_current_foot_worst_normal_current_dist",
                       optimizedVsCurrentFeet.rightNormalDistAtMinDelta);
    appendSizeT("touchdown_anchor_stance_foot_count",
                stanceAnchorSummary.anchoredFootCount);
    appendScalarOrNull("target_vs_touchdown_anchor_stance_mean_position_error",
                       stanceAnchorSummary.targetMeanPositionError);
    appendScalarOrNull("target_vs_touchdown_anchor_stance_max_position_error",
                       stanceAnchorSummary.targetMaxPositionError);
    appendString(
        "target_vs_touchdown_anchor_stance_max_position_leg",
        stanceAnchorSummary.anchoredFootCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[stanceAnchorSummary.targetMaxLegIndex]
            : std::string());
    appendScalarOrNull("current_vs_touchdown_anchor_stance_mean_position_error",
                       stanceAnchorSummary.currentMeanPositionError);
    appendScalarOrNull("current_vs_touchdown_anchor_stance_max_position_error",
                       stanceAnchorSummary.currentMaxPositionError);
    appendString(
        "current_vs_touchdown_anchor_stance_max_position_leg",
        stanceAnchorSummary.anchoredFootCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[stanceAnchorSummary.currentMaxLegIndex]
            : std::string());
    appendScalarOrNull(
        "optimized_vs_touchdown_anchor_stance_mean_position_error",
        stanceAnchorSummary.optimizedMeanPositionError);
    appendScalarOrNull(
        "optimized_vs_touchdown_anchor_stance_max_position_error",
        stanceAnchorSummary.optimizedMaxPositionError);
    appendString(
        "optimized_vs_touchdown_anchor_stance_max_position_leg",
        stanceAnchorSummary.anchoredFootCount > 0
            ? interface_->modelSettings()
                  .contactNames3DoF[stanceAnchorSummary.optimizedMaxLegIndex]
            : std::string());
    appendScalarOrNull("stance_force_ratio", stanceForceSummary.maxRatio);
    appendFieldPrefix("stance_normal_forces");
    stream << "{";
    bool firstForce = true;
    for (const auto& [legName, normalForce] : stanceForceSummary.normalForces) {
      if (!firstForce) {
        stream << ",";
      }
      firstForce = false;
      stream << "\"" << legName << "\":" << normalForce;
    }
    stream << "}";
    appendFieldPrefix("touchdown_anchor_errors_per_leg");
    stream << "{";
    bool firstAnchorLeg = true;
    for (size_t index = 0; index < contactFlags.size(); ++index) {
      if (!contactFlags[index] || !hasTouchdownAnchor_[index]) {
        continue;
      }
      if (!firstAnchorLeg) {
        stream << ",";
      }
      firstAnchorLeg = false;
      stream << "\""
             << interface_->modelSettings().contactNames3DoF[index]
             << "\":{";
      stream << "\"target\":";
      if (std::isfinite(targetAnchorErrors[index])) {
        stream << targetAnchorErrors[index];
      } else {
        stream << "null";
      }
      stream << ",\"current\":";
      if (std::isfinite(currentAnchorErrors[index])) {
        stream << currentAnchorErrors[index];
      } else {
        stream << "null";
      }
      stream << ",\"optimized\":";
      if (std::isfinite(optimizedAnchorErrors[index])) {
        stream << optimizedAnchorErrors[index];
      } else {
        stream << "null";
      }
      stream << "}";
    }
    stream << "}";
    appendFieldPrefix("nominal_desired_horizontal_force_world");
    stream << "{"
           << "\"x\":" << nominalDesiredHorizontalForce.x()
           << ",\"y\":" << nominalDesiredHorizontalForce.y()
           << ",\"z\":" << nominalDesiredHorizontalForce.z() << "}";
    const auto appendForceEntries =
        [&stream, &appendFieldPrefix](
            const std::string& key,
            const std::vector<StanceLegForceComponents>& entries) {
          appendFieldPrefix(key);
          stream << "{";
          bool firstEntry = true;
          for (const auto& entry : entries) {
            if (!firstEntry) {
              stream << ",";
            }
            firstEntry = false;
            stream << "\"" << entry.legName << "\":{";
            stream << "\"world_fx\":" << entry.forceInWorld.x()
                   << ",\"world_fy\":" << entry.forceInWorld.y()
                   << ",\"world_fz\":" << entry.forceInWorld.z()
                   << ",\"terrain_fx\":" << entry.forceInTerrain.x()
                   << ",\"terrain_fy\":" << entry.forceInTerrain.y()
                   << ",\"terrain_fz\":" << entry.forceInTerrain.z()
                   << ",\"tangential_norm\":" << entry.tangentialNorm;
            stream << "}";
          }
          stream << "}";
        };
    appendForceEntries("optimized_stance_force_components",
                       optimizedStanceForceComponents);
    appendForceEntries("nominal_stance_force_components",
                       nominalStanceForceComponents);
    stream << "}";
    RCLCPP_INFO(node_->get_logger(), "%s", stream.str().c_str());
  }

  bool isJointBranchSensitive(const std::string& jointName) const {
    return jointName.rfind("femur_joint_", 0) == 0 ||
           jointName.rfind("tarsus_joint_", 0) == 0;
  }

  void publishJointCommand(scalar_t observationTime, const vector_t& optimizedState,
                           const vector_t& optimizedInput,
                           const vector_t& measuredJointAngles,
                           size_t plannedMode) {
    if (optimizedState.size() < 12) {
      maybeWarn(
          "[HexbotRuntimeMrtNode] Optimized state is smaller than the expected Hexbot layout.");
      return;
    }
    if (!allFinite(optimizedState) || !allFinite(optimizedInput)) {
      maybeWarn(
          "[HexbotRuntimeMrtNode] Refusing to publish a non-finite backend "
          "joint command.");
      return;
    }

    sensor_msgs::msg::JointState jointCommand;
    jointCommand.header.stamp = node_->get_clock()->now();
    jointCommand.name = interface_->modelSettings().jointNames;

    const vector_t jointAngles = centroidal_model::getJointAngles(
        optimizedState, interface_->getCentroidalModelInfo());
    vector_t targetJointAngles;
    bool hasTargetJointAngles = false;
    const vector_t targetState = interpolateTargetStateAtTime(observationTime);
    if (targetState.size() ==
        static_cast<Eigen::Index>(interface_->getCentroidalModelInfo().stateDim)) {
      targetJointAngles = centroidal_model::getJointAngles(
          targetState, interface_->getCentroidalModelInfo());
      hasTargetJointAngles =
          targetJointAngles.size() == jointAngles.size() &&
          targetJointAngles.array().isFinite().all();
    }
    jointCommand.position.resize(static_cast<size_t>(jointAngles.size()));
    bool normalizedEquivalentAngle = false;
    bool branchGuardApplied = false;
    scalar_t maxNormalizationDelta = 0.0;
    scalar_t normalizationDeltaSum = 0.0;
    size_t normalizedJointCount = 0;
    size_t maxNormalizationJointIndex = 0;
    scalar_t maxNormalizationRawAngle = 0.0;
    scalar_t maxNormalizationMeasuredAngle = 0.0;
    scalar_t maxNormalizationCommandAngle = 0.0;
    scalar_t maxTrackingDelta = 0.0;
    size_t maxTrackingJointIndex = 0;
    scalar_t maxTrackingMeasuredAngle = 0.0;
    scalar_t maxTrackingCommandAngle = 0.0;
    size_t branchGuardJointIndex = 0;
    scalar_t branchGuardMeasuredAngle = 0.0;
    scalar_t branchGuardTargetAngle = 0.0;
    scalar_t branchGuardOptimizedAngle = 0.0;
    scalar_t branchGuardTargetAlignment = 0.0;
    scalar_t branchGuardCommandDeviation = 0.0;
    struct GuardJointSnapshot {
      std::string jointName;
      scalar_t optimizedAngle = std::numeric_limits<scalar_t>::quiet_NaN();
      scalar_t targetAngle = std::numeric_limits<scalar_t>::quiet_NaN();
      scalar_t measuredAngle = std::numeric_limits<scalar_t>::quiet_NaN();
    };
    std::vector<GuardJointSnapshot> tarsusSnapshots;
    tarsusSnapshots.reserve(interface_->modelSettings().jointNames.size());

    for (Eigen::Index i = 0; i < jointAngles.size(); ++i) {
      scalar_t commandAngle = jointAngles(i);
      if (measuredJointAngles.size() == jointAngles.size() &&
          i < jointConstraintLowerLimits_.size() && i < jointConstraintUpperLimits_.size()) {
        const auto& jointName =
            interface_->modelSettings().jointNames[static_cast<size_t>(i)];
        const bool isTarsusJoint = jointName.rfind("tarsus_joint_", 0) == 0;
        const scalar_t measuredCommandReference = selectNearestEquivalentAngle(
            measuredJointAngles(i), measuredJointAngles(i),
            jointConstraintLowerLimits_(i), jointConstraintUpperLimits_(i));
        scalar_t normalizedTargetAngle =
            std::numeric_limits<scalar_t>::quiet_NaN();
        const bool hasNormalizedTargetAngle =
            hasTargetJointAngles && i < targetJointAngles.size();
        if (hasNormalizedTargetAngle) {
          normalizedTargetAngle = selectNearestEquivalentAngle(
              targetJointAngles(i), measuredCommandReference,
              jointConstraintLowerLimits_(i), jointConstraintUpperLimits_(i));
        }
        if (enableJointBranchConsistencyGuard_ && hasTargetJointAngles &&
            i < targetJointAngles.size() &&
            isJointBranchSensitive(jointName)) {
          const scalar_t targetAlignment =
              std::abs(normalizedTargetAngle - measuredCommandReference);
          const scalar_t commandDeviation =
              std::abs(commandAngle - normalizedTargetAngle);
          if (targetAlignment <= jointBranchGuardAlignmentThreshold_ &&
              commandDeviation >= jointBranchGuardCommandDeviationThreshold_) {
            branchGuardApplied = true;
            branchGuardJointIndex = static_cast<size_t>(i);
            branchGuardMeasuredAngle = measuredCommandReference;
            branchGuardTargetAngle = normalizedTargetAngle;
            branchGuardOptimizedAngle = commandAngle;
            branchGuardTargetAlignment = targetAlignment;
            branchGuardCommandDeviation = commandDeviation;
            commandAngle = normalizedTargetAngle;
          }
        }
        if (isTarsusJoint) {
          GuardJointSnapshot snapshot;
          snapshot.jointName = jointName;
          snapshot.optimizedAngle = jointAngles(i);
          snapshot.measuredAngle = measuredCommandReference;
          snapshot.targetAngle =
              hasNormalizedTargetAngle
                  ? normalizedTargetAngle
                  : std::numeric_limits<scalar_t>::quiet_NaN();
          tarsusSnapshots.push_back(std::move(snapshot));
        }

        commandAngle = selectNearestEquivalentAngle(
            commandAngle, measuredCommandReference, jointConstraintLowerLimits_(i),
            jointConstraintUpperLimits_(i));
        const scalar_t normalizationDelta =
            std::abs(commandAngle - jointAngles(i));
        if (normalizationDelta > 1e-6) {
          normalizedEquivalentAngle = true;
          normalizationDeltaSum += normalizationDelta;
          ++normalizedJointCount;
          if (normalizationDelta > maxNormalizationDelta) {
            maxNormalizationDelta = normalizationDelta;
            maxNormalizationJointIndex = static_cast<size_t>(i);
            maxNormalizationRawAngle = jointAngles(i);
            maxNormalizationMeasuredAngle = measuredCommandReference;
            maxNormalizationCommandAngle = commandAngle;
          }
        }

        const scalar_t trackingDelta =
            std::abs(commandAngle - measuredCommandReference);
        if (trackingDelta > maxTrackingDelta) {
          maxTrackingDelta = trackingDelta;
          maxTrackingJointIndex = static_cast<size_t>(i);
          maxTrackingMeasuredAngle = measuredCommandReference;
          maxTrackingCommandAngle = commandAngle;
        }
      }
      jointCommand.position[static_cast<size_t>(i)] = commandAngle;
    }
    if (maxNormalizationDelta > equivalentAngleWrapResetThreshold_ ||
        maxTrackingDelta > commandTrackingResetThreshold_) {
      std::ostringstream stream;
      stream.setf(std::ios::fixed);
      stream.precision(4);
      stream
          << "[HexbotRuntimeMrtNode] Rejecting backend joint command because "
             "the normalized output exceeded runtime safety thresholds. "
          << "normalized_joints=" << normalizedJointCount
          << " max_normalization_delta_rad=" << maxNormalizationDelta
          << " normalization_joint="
          << interface_->modelSettings().jointNames[maxNormalizationJointIndex]
          << " raw=" << maxNormalizationRawAngle
          << " normalized=" << maxNormalizationCommandAngle
          << " measured=" << maxNormalizationMeasuredAngle
          << " max_tracking_delta_rad=" << maxTrackingDelta
          << " tracking_joint="
          << interface_->modelSettings().jointNames[maxTrackingJointIndex]
          << " tracking_command=" << maxTrackingCommandAngle
          << " tracking_measured=" << maxTrackingMeasuredAngle
          << " normalization_threshold_rad="
          << equivalentAngleWrapResetThreshold_
          << " tracking_threshold_rad=" << commandTrackingResetThreshold_;
      invalidateActivePolicy(stream.str(), ResetReason::SafetyReject);
      return;
    }
    if (branchGuardApplied) {
      std::ostringstream guardTrace;
      guardTrace.setf(std::ios::fixed);
      guardTrace.precision(6);
      bool firstField = true;
      const auto appendFieldPrefix = [&guardTrace, &firstField](
                                         const std::string& key) {
        if (!firstField) {
          guardTrace << ",";
        }
        firstField = false;
        guardTrace << "\"" << key << "\":";
      };
      const auto appendScalarOrNull = [&appendFieldPrefix, &guardTrace](
                                          const std::string& key,
                                          scalar_t value) {
        appendFieldPrefix(key);
        if (std::isfinite(value)) {
          guardTrace << value;
        } else {
          guardTrace << "null";
        }
      };
      const auto appendSizeT = [&appendFieldPrefix, &guardTrace](
                                   const std::string& key, size_t value) {
        appendFieldPrefix(key);
        guardTrace << value;
      };
      const auto appendString = [&appendFieldPrefix, &guardTrace](
                                    const std::string& key,
                                    const std::string& value) {
        appendFieldPrefix(key);
        guardTrace << "\"" << value << "\"";
      };
      guardTrace << "HEXBOT_BRANCH_GUARD_TRACE {";
      appendScalarOrNull("time", observationTime);
      appendSizeT("planned_mode", plannedMode);
      appendString("guard_joint",
                   interface_->modelSettings().jointNames[branchGuardJointIndex]);
      appendScalarOrNull("guard_optimized", branchGuardOptimizedAngle);
      appendScalarOrNull("guard_target", branchGuardTargetAngle);
      appendScalarOrNull("guard_measured", branchGuardMeasuredAngle);
      appendScalarOrNull("target_alignment_rad", branchGuardTargetAlignment);
      appendScalarOrNull("command_deviation_rad", branchGuardCommandDeviation);
      appendFieldPrefix("tarsus_snapshot");
      guardTrace << "{";
      bool firstJoint = true;
      for (const auto& snapshot : tarsusSnapshots) {
        if (!firstJoint) {
          guardTrace << ",";
        }
        firstJoint = false;
        guardTrace << "\"" << snapshot.jointName << "\":{";
        guardTrace << "\"optimized\":";
        if (std::isfinite(snapshot.optimizedAngle)) {
          guardTrace << snapshot.optimizedAngle;
        } else {
          guardTrace << "null";
        }
        guardTrace << ",\"target\":";
        if (std::isfinite(snapshot.targetAngle)) {
          guardTrace << snapshot.targetAngle;
        } else {
          guardTrace << "null";
        }
        guardTrace << ",\"measured\":";
        if (std::isfinite(snapshot.measuredAngle)) {
          guardTrace << snapshot.measuredAngle;
        } else {
          guardTrace << "null";
        }
        guardTrace << "}";
      }
      guardTrace << "}";
      guardTrace << "}";
      RCLCPP_WARN(node_->get_logger(), "%s", guardTrace.str().c_str());

      std::ostringstream stream;
      stream.setf(std::ios::fixed);
      stream.precision(4);
      stream
          << "[HexbotRuntimeMrtNode] Applied joint branch-consistency guard. "
          << "joint="
          << interface_->modelSettings().jointNames[branchGuardJointIndex]
          << " optimized=" << branchGuardOptimizedAngle
          << " target=" << branchGuardTargetAngle
          << " measured=" << branchGuardMeasuredAngle
          << " target_alignment_rad=" << branchGuardTargetAlignment
          << " command_deviation_rad=" << branchGuardCommandDeviation;
      maybeWarn(stream.str());
    }
    if (normalizedEquivalentAngle) {
      std::ostringstream stream;
      stream.setf(std::ios::fixed);
      stream.precision(4);
      stream
          << "[HexbotRuntimeMrtNode] Normalized optimized joint angles to the "
             "nearest OCS2-constraint-feasible equivalent. normalized_joints="
          << normalizedJointCount
          << " avg_abs_delta_rad="
          << (normalizedJointCount > 0
                  ? normalizationDeltaSum /
                        static_cast<scalar_t>(normalizedJointCount)
                  : 0.0)
          << " max_delta_rad=" << maxNormalizationDelta
          << " joint="
          << interface_->modelSettings().jointNames[maxNormalizationJointIndex]
          << " raw=" << maxNormalizationRawAngle
          << " normalized=" << maxNormalizationCommandAngle
          << " measured=" << maxNormalizationMeasuredAngle;
      maybeWarn(stream.str());
    }

    if (optimizedInput.size() >= static_cast<Eigen::Index>(
                                     interface_->getCentroidalModelInfo().inputDim)) {
      const vector_t jointVelocities = centroidal_model::getJointVelocities(
          optimizedInput, interface_->getCentroidalModelInfo());
      jointCommand.velocity.resize(static_cast<size_t>(jointVelocities.size()));
      for (Eigen::Index i = 0; i < jointVelocities.size(); ++i) {
        jointCommand.velocity[static_cast<size_t>(i)] = jointVelocities(i);
      }
    }

    backendJointCommandPublisher_->publish(jointCommand);
  }

  void maybeWarn(const std::string& message) {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - lastWarnTime_).count() >= 2.0 ||
        message != lastWarningMessage_) {
      RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
      lastWarnTime_ = now;
      lastWarningMessage_ = message;
    }
  }

  static constexpr const char* kRobotName = "hexbot";

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<LeggedRobotInterface> interface_;
  std::unique_ptr<CentroidalModelRbdConversions> rbdConversions_;
  std::unique_ptr<MRT_ROS_Interface> mrt_;
  std::shared_ptr<LeggedRobotVisualizer> visualizer_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      jointStatesSubscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      backendJointCommandPublisher_;
  rclcpp::Client<ocs2_msgs::srv::Reset>::SharedPtr mpcResetClient_;
  rclcpp::TimerBase::SharedPtr controlTimer_;

  std::mutex dataMutex_;
  sensor_msgs::msg::JointState latestJointState_;
  nav_msgs::msg::Odometry latestOdom_;
  bool hasJointState_ = false;
  bool hasOdom_ = false;
  bool mpcResetIssued_ = false;
  bool resetRequestPending_ = false;
  bool policyActive_ = false;
  ResetReason nextResetReason_ = ResetReason::Bootstrap;
  ResetReason inFlightResetReason_ = ResetReason::Bootstrap;
  size_t lastPlannedMode_ = static_cast<size_t>(ModeNumber::STANCE);
  rclcpp::Client<ocs2_msgs::srv::Reset>::SharedFuture resetResponseFuture_;
  std::chrono::steady_clock::time_point latestJointStateArrival_{};
  std::chrono::steady_clock::time_point latestOdomArrival_{};
  std::chrono::steady_clock::time_point lastResetAttempt_{};
  std::chrono::steady_clock::time_point lastWarnTime_{};
  std::string lastWarningMessage_;
  mutable bool observationTimeOriginInitialized_ = false;
  mutable scalar_t observationTimeOrigin_ = 0.0;
  mutable scalar_t lastObservationTime_ = 0.0;
  scalar_t lastSolverTraceObservationTime_ =
      std::numeric_limits<scalar_t>::lowest();
  size_t lastSolverTraceMode_ = std::numeric_limits<size_t>::max();
  feet_array_t<vector3_t> touchdownAnchorPositionsInWorld_ = [] {
    feet_array_t<vector3_t> values;
    values.fill(vector3_t::Zero());
    return values;
  }();
  contact_flag_t touchdownAnchorContactFlags_ = [] {
    contact_flag_t values;
    values.fill(false);
    return values;
  }();
  feet_array_t<bool> hasTouchdownAnchor_ = [] {
    feet_array_t<bool> values;
    values.fill(false);
    return values;
  }();
  vector_t lastNormalizedObservationJointAngles_;
  vector_t observationJointLowerLimits_;
  vector_t observationJointUpperLimits_;
  vector_t jointLowerLimits_;
  vector_t jointUpperLimits_;
  vector_t jointConstraintLowerLimits_;
  vector_t jointConstraintUpperLimits_;
  vector_t defaultJointState_;
  scalar_t nominalComHeight_ = 0.0;
  scalar_t nominalBaseHeightAboveSupportPlane_ = 0.0;
  SupportPlane latchedSupportPlane_;
  bool latchedSupportPlaneValid_ = false;

  std::string taskFile_;
  std::string urdfFile_;
  std::string referenceFile_;
  std::string inputJointStatesTopic_;
  std::string inputOdomTopic_;
  std::string backendJointCommandTopic_;
  double observationTimeout_ = 0.5;
  double resetRetryPeriod_ = 1.0;
  double holdTargetDuration_ = 0.25;
  double policyExpiryTolerance_ = 0.05;
  bool lockResetHeightToObservation_ = true;
  double equivalentAngleWrapResetThreshold_ = kPi;
  double commandTrackingResetThreshold_ = 1.0;
  bool enableJointBranchConsistencyGuard_ = true;
  double jointBranchGuardAlignmentThreshold_ = 0.15;
  double jointBranchGuardCommandDeviationThreshold_ = 0.60;
  bool enableVisualization_ = true;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hexbot_runtime_mrt");
  auto app = std::make_shared<HexbotRuntimeMrtNode>(node);
  (void)app;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
