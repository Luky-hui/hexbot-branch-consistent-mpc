/******************************************************************************
Copyright (c) 2026. All rights reserved.
******************************************************************************/

#include <geometry_msgs/msg/twist.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/Types.h>
#include <ocs2_hexbot_legged_robot/common/HexbotActuatorJointLimits.h>
#include <ocs2_hexbot_legged_robot/common/ModelSettings.h>
#include <ocs2_hexbot_legged_robot/common/utils.h>
#include <ocs2_hexbot_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ocs2_hexbot_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include "rclcpp/rclcpp.hpp"

using namespace ocs2;
using namespace legged_robot;

namespace {

scalar_t wrapAngleToPi(scalar_t angle) {
  constexpr scalar_t kPi = 3.14159265358979323846;
  constexpr scalar_t kTwoPi = 2.0 * kPi;
  scalar_t wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

class HexbotCmdVelTargetNode {
 public:
  explicit HexbotCmdVelTargetNode(rclcpp::Node::SharedPtr node)
      : node_(std::move(node)), targetPublisher_(node_, kRobotName) {
    taskFile_ = node_->declare_parameter<std::string>("taskFile", "");
    urdfFile_ = node_->declare_parameter<std::string>("urdfFile", "");
    referenceFile_ = node_->declare_parameter<std::string>("referenceFile", "");
    if (referenceFile_.empty()) {
      throw std::runtime_error(
          "[HexbotCmdVelTargetNode] Parameter 'referenceFile' is required.");
    }

    cmdVelTopic_ = node_->declare_parameter<std::string>(
        "cmdVelTopic", "/hexbot_mpc/input/cmd_vel");
    modeScheduleTopic_ = node_->declare_parameter<std::string>(
        "modeScheduleTopic", std::string(kRobotName) + "_mpc_mode_schedule");
    observationTopic_ = node_->declare_parameter<std::string>(
        "observationTopic", std::string(kRobotName) + "_mpc_observation");
    lookaheadTime_ = node_->declare_parameter<double>("targetLookahead", 0.35);
    translationLookaheadTime_ = node_->declare_parameter<double>(
        "translationLookaheadTime", std::max(lookaheadTime_, 1.20));
    rotationLookaheadTime_ =
        node_->declare_parameter<double>("rotationLookaheadTime", 1.20);
    publishRate_ = node_->declare_parameter<double>("publishRate", 20.0);
    minTargetHorizon_ =
        node_->declare_parameter<double>("minTargetHorizon", 0.20);
    cmdVelTimeout_ = node_->declare_parameter<double>("cmdVelTimeout", 0.50);
    movingCmdHoldTime_ = std::max<scalar_t>(
        0.0, node_->declare_parameter<double>("movingCmdHoldTime", 3.00));
    lockNominalHeightToObservation_ =
        node_->declare_parameter<bool>("lockNominalHeightToObservation", true);
    useLatchedObservedJointTargetWhenStationary_ =
        node_->declare_parameter<bool>(
            "useLatchedObservedJointTargetWhenStationary", true);
    stationaryJointTargetBlend_ = std::clamp<scalar_t>(
        node_->declare_parameter<double>("stationaryJointTargetBlend", 0.20),
        0.0, 1.0);
    stationaryJointTargetMaxJointSpeedRadS_ = std::max<scalar_t>(
        0.0, node_->declare_parameter<double>(
                 "stationaryJointTargetMaxJointSpeedRadS", 0.80));
    useObservedJointTargetWhileMoving_ =
        node_->declare_parameter<bool>("useObservedJointTargetWhileMoving",
                                       true);
    useContactConsistentTargetStateWhileMoving_ =
        node_->declare_parameter<bool>(
            "useContactConsistentTargetStateWhileMoving", true);
    movingTargetKnotCount_ =
        std::max<int>(2, node_->declare_parameter<int>("movingTargetKnotCount", 4));
    wholeBodyIkMaxIterations_ =
        std::max<int>(1, node_->declare_parameter<int>("wholeBodyIkMaxIterations", 12));
    wholeBodyIkDamping_ =
        std::max<scalar_t>(1.0e-5, node_->declare_parameter<double>("wholeBodyIkDamping", 0.01));
    wholeBodyIkJointRegularization_ = std::max<scalar_t>(
        1.0e-5, node_->declare_parameter<double>("wholeBodyIkJointRegularization", 0.05));
    wholeBodyIkStanceFootWeight_ = std::max<scalar_t>(
        1.0e-5, node_->declare_parameter<double>("wholeBodyIkStanceFootWeight", 1.0));
    wholeBodyIkSwingFootWeight_ = std::max<scalar_t>(
        0.0, node_->declare_parameter<double>("wholeBodyIkSwingFootWeight", 0.50));
    wholeBodyIkMaxDeltaPerIteration_ = std::max<scalar_t>(
        1.0e-4, node_->declare_parameter<double>("wholeBodyIkMaxDeltaPerIteration", 0.18));
    wholeBodyIkFootTolerance_ = std::max<scalar_t>(
        1.0e-5, node_->declare_parameter<double>("wholeBodyIkFootTolerance", 0.0025));
    phaseTransitionStanceTime_ = std::max<scalar_t>(
        0.0, node_->declare_parameter<double>("phaseTransitionStanceTime", 0.20));

    loadData::loadCppDataType(referenceFile_, "comHeight", comHeight_);
    loadData::loadEigenMatrix(referenceFile_, "defaultJointState",
                              defaultJointState_);
    loadData::loadCppDataType(referenceFile_, "targetRotationVelocity",
                              targetRotationVelocity_);
    loadData::loadCppDataType(referenceFile_, "targetDisplacementVelocity",
                              targetDisplacementVelocity_);
    loadMovingGoalJointState();

    initializeCentroidalTargetState();

    latestCmdVelTime_ = std::chrono::steady_clock::now();

    cmdVelSubscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopic_, 10,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latestCmdVel_ = *msg;
          latestCmdVelTime_ = std::chrono::steady_clock::now();
          if (isMovingCommand(*msg)) {
            lastMovingCmdVel_ = *msg;
            lastMovingCmdVelTime_ = latestCmdVelTime_;
            hasSeenMovingCmdVel_ = true;
          }
        });

    observationSubscriber_ =
        node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
            observationTopic_, 1,
            [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              latestObservation_ =
                  ros_msg_conversions::readObservationMsg(*msg);
              if (!hasObservation_ ||
                  latestObservation_.time < lastObservationTime_ - 1.0e-6 ||
                  latestObservation_.mode != lastObservedMode_) {
                lastObservedMode_ = latestObservation_.mode;
                lastObservedModeChangeTime_ = latestObservation_.time;
                hasObservedModeChangeTime_ = true;
              }
              lastObservationTime_ = latestObservation_.time;
              hasObservation_ = true;
            });
    modeScheduleSubscriber_ =
        node_->create_subscription<ocs2_msgs::msg::ModeSchedule>(
            modeScheduleTopic_, 1,
            [this](const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr& msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              try {
                const auto modeSequenceTemplate =
                    readModeSequenceTemplateMsg(*msg);
                if (!hasUsableModeSequenceTemplate(modeSequenceTemplate)) {
                  RCLCPP_WARN_THROTTLE(
                      node_->get_logger(), *node_->get_clock(), 2000,
                      "[HexbotCmdVelTargetNode] Ignoring invalid mode-sequence template on %s.",
                      modeScheduleTopic_.c_str());
                  return;
                }
                latestModeSequenceTemplate_ = modeSequenceTemplate;
                hasModeSequenceTemplate_ = true;
              } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "[HexbotCmdVelTargetNode] Failed to read mode-sequence template on %s: %s",
                    modeScheduleTopic_.c_str(), e.what());
              }
            });

    const auto period = std::chrono::duration<double>(1.0 / publishRate_);
    publishTimer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishTarget(); });

    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot cmd_vel target node ready. topic chain: %s + %s -> %s_mpc_target",
        cmdVelTopic_.c_str(), observationTopic_.c_str(), kRobotName);
    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot cmd_vel target lookahead windows: translation=%.3fs rotation=%.3fs min_horizon=%.3fs",
        translationLookaheadTime_, rotationLookaheadTime_, minTargetHorizon_);
    RCLCPP_INFO(node_->get_logger(),
                "Hexbot cmd_vel target base-height mode: %s",
                lockNominalHeightToObservation_
                    ? "track nominal comHeight relative to live support plane"
                    : "track reference comHeight");
    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot cmd_vel target joint target mode while moving: %s",
        useObservedJointTargetWhileMoving_
            ? "reuse live observation joints"
            : "reuse reference default joints");
    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot cmd_vel target joint target mode while stationary: %s (blend=%.2f max_joint_speed=%.2f rad/s)",
        useLatchedObservedJointTargetWhenStationary_
            ? "latch live observation joints"
            : "reuse reference default joints",
        stationaryJointTargetBlend_, stationaryJointTargetMaxJointSpeedRadS_);
    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot cmd_vel target contact-consistent whole-body target mode while moving: %s (knots=%d, ik_iters=%d)",
        useContactConsistentTargetStateWhileMoving_ ? "enabled" : "disabled",
        movingTargetKnotCount_, wholeBodyIkMaxIterations_);
    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot cmd_vel target moving contact projection: mode_schedule=%s transition_stance=%.3fs",
        modeScheduleTopic_.c_str(), phaseTransitionStanceTime_);
    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot cmd_vel target moving cmd hold: timeout=%.3fs hold=%.3fs",
        cmdVelTimeout_, movingCmdHoldTime_);
    if (hasMovingGoalJointState_) {
      RCLCPP_INFO(
          node_->get_logger(),
          "Hexbot cmd_vel target moving-goal joint reference: %s",
          movingGoalReferenceFile_.c_str());
    }
  }

 private:
  struct ModeProjectionContext {
    ModeSequenceTemplate modeSequenceTemplate{
        {0.0, 0.5}, {static_cast<size_t>(ModeNumber::STANCE)}};
    bool hasModeSequenceTemplate = false;
    scalar_t lastObservedModeChangeTime = 0.0;
    size_t lastObservedMode = static_cast<size_t>(ModeNumber::STANCE);
    bool hasObservedModeChangeTime = false;
  };

  void publishTarget() {
    SystemObservation observation;
    geometry_msgs::msg::Twist cmdVel;
    geometry_msgs::msg::Twist lastMovingCmdVel;
    bool hasObservation = false;
    std::chrono::steady_clock::time_point latestCmdVelTime;
    std::chrono::steady_clock::time_point lastMovingCmdVelTime;
    bool hasSeenMovingCmdVel = false;
    ModeProjectionContext modeProjectionContext;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      hasObservation = hasObservation_;
      if (hasObservation) {
        observation = latestObservation_;
      }
      cmdVel = latestCmdVel_;
      lastMovingCmdVel = lastMovingCmdVel_;
      latestCmdVelTime = latestCmdVelTime_;
      lastMovingCmdVelTime = lastMovingCmdVelTime_;
      hasSeenMovingCmdVel = hasSeenMovingCmdVel_;
      modeProjectionContext.modeSequenceTemplate = latestModeSequenceTemplate_;
      modeProjectionContext.hasModeSequenceTemplate = hasModeSequenceTemplate_;
      modeProjectionContext.lastObservedModeChangeTime =
          lastObservedModeChangeTime_;
      modeProjectionContext.lastObservedMode = lastObservedMode_;
      modeProjectionContext.hasObservedModeChangeTime =
          hasObservedModeChangeTime_;
    }

    if (!hasObservation) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool cmdVelStale =
        std::chrono::duration<double>(now - latestCmdVelTime).count() >
        cmdVelTimeout_;
    const bool recentMovingCommand =
        hasSeenMovingCmdVel &&
        std::chrono::duration<double>(now - lastMovingCmdVelTime).count() <=
            movingCmdHoldTime_;
    if ((cmdVelStale || !isMovingCommand(cmdVel)) && recentMovingCommand) {
      cmdVel = lastMovingCmdVel;
    } else if (cmdVelStale) {
      cmdVel = geometry_msgs::msg::Twist();
    }

    targetPublisher_.publishTargetTrajectories(
        createTargetTrajectories(cmdVel, observation, modeProjectionContext));
  }

  TargetTrajectories createTargetTrajectories(
      const geometry_msgs::msg::Twist& cmdVel,
      const SystemObservation& observation,
      const ModeProjectionContext& modeProjectionContext) const {
    if (observation.state.size() < 12) {
      throw std::runtime_error(
          "[HexbotCmdVelTargetNode] Observation state is smaller than the "
          "expected base state layout.");
    }

    const vector_t nominalPose = createNominalBasePose(observation);
    vector_t targetPose = nominalPose;
    maybeUpdateStationaryJointTarget(cmdVel, observation);
    const vector_t startJointTarget =
        createStartJointTargetState(cmdVel, observation);
    const vector_t goalJointTarget =
        createGoalJointTargetState(cmdVel, observation);

    const scalar_t yaw = wrapAngleToPi(nominalPose(3));
    const scalar_t translationLookahead =
        std::max<scalar_t>(translationLookaheadTime_, 1e-3);
    const scalar_t rotationLookahead =
        std::max<scalar_t>(rotationLookaheadTime_, translationLookahead);
    const scalar_t dxBody = cmdVel.linear.x * translationLookahead;
    const scalar_t dyBody = cmdVel.linear.y * translationLookahead;
    const scalar_t dxWorld = std::cos(yaw) * dxBody - std::sin(yaw) * dyBody;
    const scalar_t dyWorld = std::sin(yaw) * dxBody + std::cos(yaw) * dyBody;
    const scalar_t dyaw = cmdVel.angular.z * rotationLookahead;

    targetPose(0) += dxWorld;
    targetPose(1) += dyWorld;
    targetPose(2) = nominalPose(2);
    targetPose(3) = wrapAngleToPi(yaw + dyaw);
    targetPose(4) = 0.0;
    targetPose(5) = 0.0;

    const scalar_t translationHorizon =
        (std::abs(cmdVel.linear.x) > 1e-6 || std::abs(cmdVel.linear.y) > 1e-6)
            ? translationLookahead
            : scalar_t{0.0};
    const scalar_t rotationHorizon =
        std::abs(cmdVel.angular.z) > 1e-6 ? rotationLookahead : scalar_t{0.0};
    const scalar_t targetReachingTime =
        observation.time +
        std::max<scalar_t>(minTargetHorizon_,
                           std::max(translationHorizon, rotationHorizon));

    if (shouldUseContactConsistentTargetState(cmdVel, observation,
                                              targetReachingTime,
                                              modeProjectionContext)) {
      return createContactConsistentTargetTrajectories(
          cmdVel, observation, nominalPose, targetPose, startJointTarget,
          goalJointTarget, targetReachingTime, modeProjectionContext);
    }

    const scalar_array_t timeTrajectory{observation.time, targetReachingTime};
    vector_array_t stateTrajectory(2,
                                   vector_t::Zero(observation.state.size()));
    stateTrajectory[0] = createDesiredState(nominalPose, startJointTarget, cmdVel,
                                            observation.state.size());
    stateTrajectory[1] = createDesiredState(targetPose, goalJointTarget, cmdVel,
                                            observation.state.size());

    const vector_array_t inputTrajectory(
        2, vector_t::Zero(observation.input.size()));

    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  TargetTrajectories createContactConsistentTargetTrajectories(
      const geometry_msgs::msg::Twist& cmdVel,
      const SystemObservation& observation, const vector_t& startPose,
      const vector_t& goalPose, const vector_t& startJointTarget,
      const vector_t& nominalGoalJointTarget,
      scalar_t targetReachingTime,
      const ModeProjectionContext& modeProjectionContext) const {
    const size_t knotCount =
        static_cast<size_t>(std::max(2, movingTargetKnotCount_));
    const auto startFootPositionsInWorld =
        estimateFootPositionsInWorld(observation);

    scalar_array_t timeTrajectory(knotCount, observation.time);
    vector_array_t stateTrajectory(
        knotCount, vector_t::Zero(observation.state.size()));
    vector_array_t inputTrajectory(
        knotCount, vector_t::Zero(observation.input.size()));

    vector_t previousJointTarget = startJointTarget;
    auto previousContactFlags = projectContactFlagsAtTime(
        observation.time, cmdVel, observation, modeProjectionContext);
    feet_array_t<vector3_t> predictedTouchdownAnchorPositionsInWorld =
        startFootPositionsInWorld;
    feet_array_t<bool> hasPredictedTouchdownAnchor;
    hasPredictedTouchdownAnchor.fill(false);
    for (size_t leg = 0; leg < contactNames3DoF_.size(); ++leg) {
      hasPredictedTouchdownAnchor[leg] = previousContactFlags[leg];
    }

    for (size_t knot = 0; knot < knotCount; ++knot) {
      const scalar_t alpha =
          knotCount > 1
              ? static_cast<scalar_t>(knot) /
                    static_cast<scalar_t>(knotCount - 1)
              : scalar_t{1.0};
      timeTrajectory[knot] =
          observation.time + alpha * (targetReachingTime - observation.time);
      const vector_t pose = interpolatePose(startPose, goalPose, alpha);
      const vector_t nominalJointTarget =
          interpolateJointTargets(startJointTarget, nominalGoalJointTarget,
                                  alpha);
      const auto nominalFootPositionsInWorld = computeFootPositionsInWorld(
          *pinocchioInterface_, centroidalModelInfo_, contactNames3DoF_,
          composeKinematicState(pose, nominalJointTarget));
      const auto contactFlags = projectContactFlagsAtTime(
          timeTrajectory[knot], cmdVel, observation, modeProjectionContext);

      vector_t reconstructedJointTarget = nominalJointTarget;
      if (knot == 0) {
        reconstructedJointTarget = startJointTarget;
      } else {
        feet_array_t<vector3_t> desiredFootPositionsInWorld;
        feet_array_t<scalar_t> desiredFootWeights;
        desiredFootPositionsInWorld.fill(vector3_t::Zero());
        desiredFootWeights.fill(0.0);
        for (size_t leg = 0; leg < contactNames3DoF_.size(); ++leg) {
          const vector3_t swingFootPosition = interpolateFootPosition(
              startFootPositionsInWorld[leg], nominalFootPositionsInWorld[leg],
              alpha);
          if (contactFlags[leg]) {
            if (!previousContactFlags[leg] ||
                !hasPredictedTouchdownAnchor[leg]) {
              predictedTouchdownAnchorPositionsInWorld[leg] = swingFootPosition;
              hasPredictedTouchdownAnchor[leg] = true;
            }
            desiredFootPositionsInWorld[leg] =
                predictedTouchdownAnchorPositionsInWorld[leg];
            desiredFootWeights[leg] = wholeBodyIkStanceFootWeight_;
          } else {
            desiredFootPositionsInWorld[leg] = swingFootPosition;
            desiredFootWeights[leg] = wholeBodyIkSwingFootWeight_;
            hasPredictedTouchdownAnchor[leg] = false;
          }
        }
        reconstructedJointTarget = solveContactConsistentWholeBodyIk(
            pose, previousJointTarget, nominalJointTarget,
            desiredFootPositionsInWorld, desiredFootWeights);
      }

      stateTrajectory[knot] = createDesiredState(
          pose, reconstructedJointTarget, cmdVel, observation.state.size());
      previousJointTarget = reconstructedJointTarget;
      previousContactFlags = contactFlags;
    }

    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  vector_t createDesiredState(const vector_t& pose, const vector_t& jointTarget,
                              const geometry_msgs::msg::Twist& cmdVel,
                              size_t stateDim) const {
    if (useCentroidalTargetState_) {
      return createDesiredCentroidalState(pose, jointTarget, cmdVel, stateDim);
    }
    return createFallbackDesiredState(pose, jointTarget, stateDim);
  }

  vector_t createDesiredCentroidalState(const vector_t& pose,
                                        const vector_t& jointTarget,
                                        const geometry_msgs::msg::Twist& cmdVel,
                                        size_t stateDim) const {
    if (!rbdConversions_ ||
        stateDim != static_cast<size_t>(centroidalModelInfo_.stateDim)) {
      return createFallbackDesiredState(pose, jointTarget, stateDim);
    }

    const auto& info = centroidalModelInfo_;
    vector_t rbdState = vector_t::Zero(2 * info.generalizedCoordinatesNum);
    rbdState.segment<3>(0) = pose.segment<3>(3);
    rbdState.segment<3>(3) = pose.segment<3>(0);

    const auto jointCount = std::min<Eigen::Index>(
        static_cast<Eigen::Index>(jointTarget.size()), info.actuatedDofNum);
    if (jointCount > 0) {
      rbdState.segment(6, jointCount) = jointTarget.head(jointCount);
    }

    const scalar_t yaw = pose(3);
    const scalar_t cosYaw = std::cos(yaw);
    const scalar_t sinYaw = std::sin(yaw);
    const scalar_t linearWorldX =
        cosYaw * cmdVel.linear.x - sinYaw * cmdVel.linear.y;
    const scalar_t linearWorldY =
        sinYaw * cmdVel.linear.x + cosYaw * cmdVel.linear.y;

    rbdState.segment<3>(info.generalizedCoordinatesNum) << 0.0, 0.0,
        cmdVel.angular.z;
    rbdState.segment<3>(info.generalizedCoordinatesNum + 3)
        << linearWorldX, linearWorldY, 0.0;

    vector_t desiredState =
        rbdConversions_->computeCentroidalStateFromRbdModel(rbdState);
    centroidal_model::getBasePose(desiredState, info) = pose;
    if (jointCount > 0) {
      centroidal_model::getJointAngles(desiredState, info).head(jointCount) = 
          jointTarget.head(jointCount);
    }
    return desiredState;
  }

  vector_t createFallbackDesiredState(const vector_t& pose,
                                      const vector_t& jointTarget,
                                      size_t stateDim) const {
    vector_t desiredState = vector_t::Zero(stateDim);
    desiredState.segment(6, 6) = pose;
    if (stateDim > 12 && jointTarget.size() > 0) {
      const auto jointCount =
          std::min<int>(jointTarget.size(), static_cast<int>(stateDim) - 12);
      desiredState.segment(12, jointCount) = jointTarget.head(jointCount);
    }
    return desiredState;
  }

  vector_t composeKinematicState(const vector_t& pose,
                                 const vector_t& jointTarget) const {
    vector_t state = vector_t::Zero(centroidalModelInfo_.stateDim);
    centroidal_model::getBasePose(state, centroidalModelInfo_) = pose;
    const auto jointCount = std::min<Eigen::Index>(
        centroidalModelInfo_.actuatedDofNum,
        static_cast<Eigen::Index>(jointTarget.size()));
    if (jointCount > 0) {
      centroidal_model::getJointAngles(state, centroidalModelInfo_)
          .head(jointCount) = jointTarget.head(jointCount);
    }
    return state;
  }

  vector_t interpolatePose(const vector_t& startPose, const vector_t& goalPose,
                           scalar_t alpha) const {
    vector_t pose = startPose;
    pose.head<3>() = (1.0 - alpha) * startPose.head<3>() +
                     alpha * goalPose.head<3>();
    pose(3) =
        wrapAngleToPi(startPose(3) + alpha * wrapAngleToPi(goalPose(3) - startPose(3)));
    pose(4) = (1.0 - alpha) * startPose(4) + alpha * goalPose(4);
    pose(5) = (1.0 - alpha) * startPose(5) + alpha * goalPose(5);
    return pose;
  }

  vector_t interpolateJointTargets(const vector_t& startJointTarget,
                                   const vector_t& goalJointTarget,
                                   scalar_t alpha) const {
    if (startJointTarget.size() != goalJointTarget.size()) {
      return goalJointTarget;
    }
    return (1.0 - alpha) * startJointTarget + alpha * goalJointTarget;
  }

  vector3_t interpolateFootPosition(const vector3_t& startPosition,
                                    const vector3_t& goalPosition,
                                    scalar_t alpha) const {
    return (1.0 - alpha) * startPosition + alpha * goalPosition;
  }

  bool isMovingCommand(const geometry_msgs::msg::Twist& cmdVel) const {
    return std::abs(cmdVel.linear.x) > 1e-6 ||
           std::abs(cmdVel.linear.y) > 1e-6 ||
           std::abs(cmdVel.angular.z) > 1e-6;
  }

  bool shouldUseContactConsistentTargetState(
      const geometry_msgs::msg::Twist& cmdVel,
      const SystemObservation& observation, scalar_t targetReachingTime,
      const ModeProjectionContext& modeProjectionContext) const {
    if (!useContactConsistentTargetStateWhileMoving_ ||
        !isMovingCommand(cmdVel) || !useCentroidalTargetState_ ||
        !pinocchioInterface_ ||
        observation.state.size() !=
            static_cast<Eigen::Index>(centroidalModelInfo_.stateDim)) {
      return false;
    }

    const size_t knotCount =
        static_cast<size_t>(std::max(2, movingTargetKnotCount_));
    for (size_t knot = 0; knot < knotCount; ++knot) {
      const scalar_t alpha =
          knotCount > 1
              ? static_cast<scalar_t>(knot) /
                    static_cast<scalar_t>(knotCount - 1)
              : scalar_t{1.0};
      const scalar_t queryTime =
          observation.time + alpha * (targetReachingTime - observation.time);
      const auto contactFlags = projectContactFlagsAtTime(
          queryTime, cmdVel, observation, modeProjectionContext);
      const size_t stanceFootCount = numberOfClosedContacts(contactFlags);
      if (stanceFootCount > 0 && stanceFootCount < contactFlags.size()) {
        return true;
      }
    }

    return false;
  }

  bool hasUsableModeSequenceTemplate(
      const ModeSequenceTemplate& modeSequenceTemplate) const {
    if (modeSequenceTemplate.modeSequence.empty() ||
        modeSequenceTemplate.switchingTimes.size() !=
            modeSequenceTemplate.modeSequence.size() + 1) {
      return false;
    }

    for (size_t index = 0; index < modeSequenceTemplate.modeSequence.size();
         ++index) {
      const scalar_t phaseDuration =
          modeSequenceTemplate.switchingTimes[index + 1] -
          modeSequenceTemplate.switchingTimes[index];
      if (!std::isfinite(phaseDuration) || phaseDuration <= 1.0e-6) {
        return false;
      }
    }

    return true;
  }

  std::vector<scalar_t> computeModeSequencePhaseDurations(
      const ModeSequenceTemplate& modeSequenceTemplate) const {
    std::vector<scalar_t> phaseDurations;
    phaseDurations.reserve(modeSequenceTemplate.modeSequence.size());
    for (size_t index = 0; index < modeSequenceTemplate.modeSequence.size();
         ++index) {
      phaseDurations.push_back(modeSequenceTemplate.switchingTimes[index + 1] -
                               modeSequenceTemplate.switchingTimes[index]);
    }
    return phaseDurations;
  }

  int findModeIndexInSequence(
      const ModeSequenceTemplate& modeSequenceTemplate, size_t mode) const {
    const auto it = std::find(modeSequenceTemplate.modeSequence.begin(),
                              modeSequenceTemplate.modeSequence.end(), mode);
    if (it == modeSequenceTemplate.modeSequence.end()) {
      return -1;
    }
    return static_cast<int>(
        std::distance(modeSequenceTemplate.modeSequence.begin(), it));
  }

  size_t advanceModeSequence(
      const ModeSequenceTemplate& modeSequenceTemplate,
      const std::vector<scalar_t>& phaseDurations, size_t startIndex,
      scalar_t dt) const {
    if (modeSequenceTemplate.modeSequence.empty() || phaseDurations.empty()) {
      return static_cast<size_t>(ModeNumber::STANCE);
    }

    const scalar_t cycleDuration =
        std::accumulate(phaseDurations.begin(), phaseDurations.end(),
                        scalar_t{0.0});
    if (cycleDuration > 1.0e-6 && dt > cycleDuration) {
      dt = std::fmod(dt, cycleDuration);
    }

    size_t index = startIndex % modeSequenceTemplate.modeSequence.size();
    size_t guardCount = 0;
    while (guardCount < phaseDurations.size() &&
           dt > phaseDurations[index] + 1.0e-6) {
      dt -= phaseDurations[index];
      index = (index + 1) % phaseDurations.size();
      ++guardCount;
    }

    return modeSequenceTemplate.modeSequence[index];
  }

  size_t projectModeAtTime(
      scalar_t queryTime, const geometry_msgs::msg::Twist& cmdVel,
      const SystemObservation& observation,
      const ModeProjectionContext& modeProjectionContext) const {
    if (queryTime <= observation.time + 1.0e-6 ||
        !modeProjectionContext.hasModeSequenceTemplate ||
        !hasUsableModeSequenceTemplate(
            modeProjectionContext.modeSequenceTemplate)) {
      return observation.mode;
    }

    const auto phaseDurations = computeModeSequencePhaseDurations(
        modeProjectionContext.modeSequenceTemplate);
    scalar_t dt = queryTime - observation.time;
    const int currentModeIndex = findModeIndexInSequence(
        modeProjectionContext.modeSequenceTemplate, observation.mode);
    if (currentModeIndex >= 0) {
      scalar_t elapsedInCurrentMode = 0.0;
      if (modeProjectionContext.hasObservedModeChangeTime &&
          modeProjectionContext.lastObservedMode == observation.mode &&
          std::isfinite(modeProjectionContext.lastObservedModeChangeTime)) {
        elapsedInCurrentMode = std::clamp(
            observation.time - modeProjectionContext.lastObservedModeChangeTime,
            scalar_t{0.0}, phaseDurations[static_cast<size_t>(currentModeIndex)]);
      }

      const scalar_t remainingInCurrentMode =
          std::max<scalar_t>(
              0.0,
              phaseDurations[static_cast<size_t>(currentModeIndex)] -
                  elapsedInCurrentMode);
      if (dt <= remainingInCurrentMode + 1.0e-6) {
        return observation.mode;
      }

      dt -= remainingInCurrentMode;
      return advanceModeSequence(modeProjectionContext.modeSequenceTemplate,
                                 phaseDurations,
                                 static_cast<size_t>(currentModeIndex + 1), dt);
    }

    if (observation.mode == static_cast<size_t>(ModeNumber::STANCE) &&
        isMovingCommand(cmdVel)) {
      if (dt <= phaseTransitionStanceTime_ + 1.0e-6) {
        return observation.mode;
      }
      dt -= phaseTransitionStanceTime_;
      return advanceModeSequence(modeProjectionContext.modeSequenceTemplate,
                                 phaseDurations, 0, dt);
    }

    return observation.mode;
  }

  contact_flag_t projectContactFlagsAtTime(
      scalar_t queryTime, const geometry_msgs::msg::Twist& cmdVel,
      const SystemObservation& observation,
      const ModeProjectionContext& modeProjectionContext) const {
    return modeNumber2StanceLeg(projectModeAtTime(
        queryTime, cmdVel, observation, modeProjectionContext));
  }

  vector_t extractObservedJointTargetState(
      const SystemObservation& observation) const {
    if (useCentroidalTargetState_ &&
        observation.state.size() ==
            static_cast<Eigen::Index>(centroidalModelInfo_.stateDim)) {
      const auto observedJointAngles =
          centroidal_model::getJointAngles(observation.state,
                                           centroidalModelInfo_);
      if (observedJointAngles.array().isFinite().all()) {
        return observedJointAngles;
      }
    }

    if (observation.state.size() > 12) {
      const auto jointCount = std::min<Eigen::Index>(
          observation.state.size() - 12,
          static_cast<Eigen::Index>(defaultJointState_.size()));
      if (jointCount > 0) {
        vector_t observedJointAngles = defaultJointState_;
        observedJointAngles.head(jointCount) =
            observation.state.segment(12, jointCount);
        if (observedJointAngles.array().isFinite().all()) {
          return observedJointAngles;
        }
      }
    }

    return defaultJointState_;
  }

  vector_t createObservedJointTargetState(
      const SystemObservation& observation) const {
    if (!useObservedJointTargetWhileMoving_) {
      return defaultJointState_;
    }

    return extractObservedJointTargetState(observation);
  }

  scalar_t computeMaxObservedJointSpeed(
      const SystemObservation& observation) const {
    if (useCentroidalTargetState_ &&
        observation.input.size() ==
            static_cast<Eigen::Index>(centroidalModelInfo_.inputDim)) {
      const auto jointVelocities =
          centroidal_model::getJointVelocities(observation.input,
                                               centroidalModelInfo_);
      if (jointVelocities.size() > 0 &&
          jointVelocities.array().isFinite().all()) {
        return jointVelocities.array().abs().maxCoeff();
      }
    }

    return 0.0;
  }

  bool canLatchStationaryJointTarget(
      const SystemObservation& observation) const {
    const auto contactFlags = modeNumber2StanceLeg(observation.mode);
    if (numberOfClosedContacts(contactFlags) != contactFlags.size()) {
      return false;
    }

    const scalar_t maxObservedJointSpeed =
        computeMaxObservedJointSpeed(observation);
    return std::isfinite(maxObservedJointSpeed) &&
           maxObservedJointSpeed <= stationaryJointTargetMaxJointSpeedRadS_;
  }

  void maybeUpdateStationaryJointTarget(
      const geometry_msgs::msg::Twist& cmdVel,
      const SystemObservation& observation) const {
    if (!useLatchedObservedJointTargetWhenStationary_ ||
        isMovingCommand(cmdVel) || !canLatchStationaryJointTarget(observation)) {
      return;
    }

    const vector_t observedJointTarget =
        extractObservedJointTargetState(observation);
    if (observedJointTarget.size() != defaultJointState_.size() ||
        !observedJointTarget.array().isFinite().all()) {
      return;
    }

    if (!hasStationaryJointTarget_) {
      stationaryJointTarget_ = observedJointTarget;
      hasStationaryJointTarget_ = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "Hexbot cmd_vel target latched stationary joint target from live observation.");
      return;
    }

    stationaryJointTarget_ =
        (1.0 - stationaryJointTargetBlend_) * stationaryJointTarget_ +
        stationaryJointTargetBlend_ * observedJointTarget;
  }

  vector_t createStationaryJointTargetState(
      const SystemObservation& observation) const {
    if (!useLatchedObservedJointTargetWhenStationary_) {
      return defaultJointState_;
    }

    if (hasStationaryJointTarget_) {
      return stationaryJointTarget_;
    }

    return extractObservedJointTargetState(observation);
  }

  vector_t createStartJointTargetState(
      const geometry_msgs::msg::Twist& cmdVel,
      const SystemObservation& observation) const {
    if (!isMovingCommand(cmdVel)) {
      return createStationaryJointTargetState(observation);
    }
    return createObservedJointTargetState(observation);
  }

  vector_t createGoalJointTargetState(
      const geometry_msgs::msg::Twist& cmdVel,
      const SystemObservation& observation) const {
    if (!isMovingCommand(cmdVel)) {
      return createStationaryJointTargetState(observation);
    }

    if (hasMovingGoalJointState_ &&
        movingGoalJointState_.size() == defaultJointState_.size()) {
      return movingGoalJointState_;
    }

    return createObservedJointTargetState(observation);
  }

  vector_t createNominalBasePose(const SystemObservation& observation) const {
    const vector_t observationPose = observation.state.segment<6>(6);
    vector_t nominalPose = observationPose;
    if (!lockNominalHeightToObservation_) {
      nominalPose(2) = comHeight_;
    } else {
      nominalPose(2) = computeSupportPlaneRelativeBaseHeight(observation);
    }
    nominalPose(3) = wrapAngleToPi(observationPose(3));
    nominalPose(4) = 0.0;
    nominalPose(5) = 0.0;
    return nominalPose;
  }

  scalar_t computeSupportPlaneRelativeBaseHeight(
      const SystemObservation& observation) const {
    if (!useCentroidalTargetState_ || !pinocchioInterface_ ||
        observation.state.size() !=
            static_cast<Eigen::Index>(centroidalModelInfo_.stateDim)) {
      return observation.state.size() >= 9 ? observation.state(8) : comHeight_;
    }

    const auto footPositionsInWorld = estimateFootPositionsInWorld(observation);
    const auto supportPlane = latchSupportPlaneBaseline(
        estimateSupportPlaneFromLowestFeet(footPositionsInWorld));
    const auto& basePose =
        centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
    return solveSupportPlaneHeightAtXY(
        supportPlane, basePose(0), basePose(1),
        nominalBaseHeightAboveSupportPlane_);
  }

  SupportPlane latchSupportPlaneBaseline(
      const SupportPlane& candidateSupportPlane) const {
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

  feet_array_t<vector3_t> estimateFootPositionsInWorld(
      const SystemObservation& observation) const {
    return computeFootPositionsInWorld(*pinocchioInterface_, centroidalModelInfo_,
                                       contactNames3DoF_, observation.state);
  }

  vector_t solveContactConsistentWholeBodyIk(
      const vector_t& pose, const vector_t& initialJointTarget,
      const vector_t& nominalJointTarget,
      const feet_array_t<vector3_t>& desiredFootPositionsInWorld,
      const feet_array_t<scalar_t>& desiredFootWeights) const {
    if (!pinocchioInterface_ || jointNames_.size() != defaultJointState_.size() ||
        nominalJointTarget.size() != defaultJointState_.size()) {
      return nominalJointTarget;
    }

    const Eigen::Index jointCount = std::min<Eigen::Index>(
        centroidalModelInfo_.actuatedDofNum,
        static_cast<Eigen::Index>(defaultJointState_.size()));
    vector_t jointTarget = nominalJointTarget;
    if (initialJointTarget.size() == nominalJointTarget.size()) {
      jointTarget = initialJointTarget;
    }
    jointTarget = jointTarget.head(jointCount);
    const vector_t nominalJointTargetClamped = nominalJointTarget.head(jointCount);

    vector_t bestJointTarget = jointTarget;
    scalar_t bestMaxFootError = std::numeric_limits<scalar_t>::infinity();

    auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    for (int iteration = 0; iteration < wholeBodyIkMaxIterations_; ++iteration) {
      const auto candidateState = composeKinematicState(pose, jointTarget);
      const auto q = centroidal_model::getGeneralizedCoordinates(
          candidateState, centroidalModelInfo_);
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      pinocchio::computeJointJacobians(model, data, q);

      const auto currentFootPositionsInWorld = computeFootPositionsInWorld(
          *pinocchioInterface_, centroidalModelInfo_, contactNames3DoF_,
          candidateState);

      size_t trackedLegCount = 0;
      scalar_t maxFootError = 0.0;
      for (size_t leg = 0; leg < desiredFootWeights.size(); ++leg) {
        if (desiredFootWeights[leg] > 1.0e-9) {
          ++trackedLegCount;
          maxFootError = std::max(
              maxFootError, (desiredFootPositionsInWorld[leg] -
                             currentFootPositionsInWorld[leg])
                                .norm());
        }
      }
      if (maxFootError < bestMaxFootError) {
        bestMaxFootError = maxFootError;
        bestJointTarget = jointTarget;
      }
      if (trackedLegCount == 0 || maxFootError <= wholeBodyIkFootTolerance_) {
        break;
      }

      const Eigen::Index taskRows =
          static_cast<Eigen::Index>(3 * trackedLegCount);
      matrix_t A = matrix_t::Zero(taskRows + jointCount, jointCount);
      vector_t b = vector_t::Zero(taskRows + jointCount);

      Eigen::Index row = 0;
      for (size_t leg = 0; leg < desiredFootWeights.size(); ++leg) {
        const scalar_t weight = desiredFootWeights[leg];
        if (weight <= 1.0e-9) {
          continue;
        }

        const vector3_t footErrorInWorld =
            desiredFootPositionsInWorld[leg] - currentFootPositionsInWorld[leg];
        matrix_t frameJacobian = matrix_t::Zero(6, model.nv);
        pinocchio::getFrameJacobian(
            model, data, model.getFrameId(contactNames3DoF_[leg]),
            pinocchio::LOCAL_WORLD_ALIGNED, frameJacobian);

        for (size_t jointOffset = 0; jointOffset < 3; ++jointOffset) {
          const Eigen::Index jointIndex =
              static_cast<Eigen::Index>(3 * leg + jointOffset);
          const auto jointId =
              model.getJointId(jointNames_[static_cast<size_t>(jointIndex)]);
          const auto velocityIndex =
              static_cast<Eigen::Index>(model.joints[jointId].idx_v());
          A.block(row, jointIndex, 3, 1) =
              weight * frameJacobian.block<3, 1>(0, velocityIndex);
        }
        b.segment(row, 3) = weight * footErrorInWorld;
        row += 3;
      }

      A.block(taskRows, 0, jointCount, jointCount).setIdentity();
      A.block(taskRows, 0, jointCount, jointCount) *=
          wholeBodyIkJointRegularization_;
      b.segment(taskRows, jointCount) =
          wholeBodyIkJointRegularization_ *
          (nominalJointTargetClamped - jointTarget);

      const matrix_t hessian =
          A.transpose() * A +
          wholeBodyIkDamping_ * wholeBodyIkDamping_ *
              matrix_t::Identity(jointCount, jointCount);
      const vector_t gradient = A.transpose() * b;
      vector_t deltaJoint = hessian.ldlt().solve(gradient);
      if (!deltaJoint.array().isFinite().all()) {
        break;
      }

      for (Eigen::Index jointIndex = 0; jointIndex < jointCount; ++jointIndex) {
        const scalar_t clampedDelta = std::clamp(
            deltaJoint(jointIndex), -wholeBodyIkMaxDeltaPerIteration_,
            wholeBodyIkMaxDeltaPerIteration_);
        jointTarget(jointIndex) = std::clamp(
            jointTarget(jointIndex) + clampedDelta,
            guardedJointLowerLimits_(jointIndex),
            guardedJointUpperLimits_(jointIndex));
      }
    }

    return bestJointTarget.size() == nominalJointTarget.size()
               ? bestJointTarget
               : nominalJointTarget;
  }

  void initializeCentroidalTargetState() {
    if (taskFile_.empty() || urdfFile_.empty()) {
      RCLCPP_WARN(
          node_->get_logger(),
          "[HexbotCmdVelTargetNode] taskFile/urdfFile not provided. Falling "
          "back to pose-only target states.");
      return;
    }

    try {
      try {
        phaseTransitionStanceTime_ = std::max<scalar_t>(
            0.0, loadModelSettings(taskFile_, "model_settings", false)
                     .phaseTransitionStanceTime);
      } catch (const std::exception&) {
      }
      std::vector<std::string> jointNames;
      std::vector<std::string> contactNames3DoF;
      std::vector<std::string> contactNames6DoF;
      loadData::loadStdVector(taskFile_, "model_settings.jointNames", jointNames);
      loadData::loadStdVector(taskFile_, "model_settings.contactNames3DoF",
                              contactNames3DoF);
      auto pinocchioInterface =
          centroidal_model::createPinocchioInterface(urdfFile_, jointNames);
      centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
          pinocchioInterface, centroidal_model::loadCentroidalType(taskFile_),
          centroidal_model::loadDefaultJointState(
              pinocchioInterface.getModel().nq - 6, referenceFile_),
          contactNames3DoF, contactNames6DoF);
      rbdConversions_ = std::make_unique<CentroidalModelRbdConversions>(
          pinocchioInterface, centroidalModelInfo_);
      pinocchioInterface_ =
          std::make_unique<PinocchioInterface>(std::move(pinocchioInterface));
      jointNames_ = jointNames;
      contactNames3DoF_ = contactNames3DoF;
      auto guardedJointLimits =
          getHexbotActuatorGuardedJointLimits(jointNames_);
      guardedJointLowerLimits_ = std::move(guardedJointLimits.first);
      guardedJointUpperLimits_ = std::move(guardedJointLimits.second);
      nominalBaseHeightAboveSupportPlane_ =
          computeNominalBaseHeightAboveSupportPlane();
      useCentroidalTargetState_ = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "Hexbot cmd_vel target node enabled centroidal target-state "
          "conversion for velocity-aware references.");
      RCLCPP_INFO(node_->get_logger(),
                  "Hexbot cmd_vel target nominal base height above support plane: %.4f m",
                  nominalBaseHeightAboveSupportPlane_);
    } catch (const std::exception& e) {
      useCentroidalTargetState_ = false;
      rbdConversions_.reset();
      RCLCPP_WARN(
          node_->get_logger(),
          "[HexbotCmdVelTargetNode] Failed to initialize centroidal target "
          "state conversion: %s. Falling back to pose-only targets.",
          e.what());
    }
  }

  void loadMovingGoalJointState() {
    movingGoalJointState_ = defaultJointState_;
    hasMovingGoalJointState_ = false;

    std::string candidateFile =
        node_->declare_parameter<std::string>("movingGoalReferenceFile", "");
    const bool autoLoadMovingGoalReferenceFile =
        node_->declare_parameter<bool>("autoLoadMovingGoalReferenceFile", false);
    if (candidateFile.empty() && autoLoadMovingGoalReferenceFile) {
      const auto siblingPath = std::filesystem::path(referenceFile_)
                                   .parent_path() /
                               "reference_loaded_equilibrium.info";
      candidateFile = siblingPath.string();
    }

    if (candidateFile.empty() || candidateFile == referenceFile_ ||
        !std::filesystem::exists(candidateFile)) {
      return;
    }

    try {
      vector_t candidateJointState = defaultJointState_;
      loadData::loadEigenMatrix(candidateFile, "defaultJointState",
                                candidateJointState);
      if (candidateJointState.size() != defaultJointState_.size() ||
          !candidateJointState.array().isFinite().all()) {
        RCLCPP_WARN(
            node_->get_logger(),
            "[HexbotCmdVelTargetNode] Ignoring moving-goal joint reference %s due to invalid size/values.",
            candidateFile.c_str());
        return;
      }
      movingGoalJointState_ = std::move(candidateJointState);
      movingGoalReferenceFile_ = candidateFile;
      hasMovingGoalJointState_ = true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(
          node_->get_logger(),
          "[HexbotCmdVelTargetNode] Failed to load moving-goal reference %s: %s",
          candidateFile.c_str(), e.what());
    }
  }

  scalar_t computeNominalBaseHeightAboveSupportPlane() const {
    if (!pinocchioInterface_) {
      return comHeight_;
    }

    vector_t nominalState = vector_t::Zero(centroidalModelInfo_.stateDim);
    auto nominalBasePose =
        centroidal_model::getBasePose(nominalState, centroidalModelInfo_);
    nominalBasePose << 0.0, 0.0, comHeight_, 0.0, 0.0, 0.0;
    const auto jointCount = std::min<Eigen::Index>(
        static_cast<Eigen::Index>(defaultJointState_.size()),
        centroidalModelInfo_.actuatedDofNum);
    if (jointCount > 0) {
      centroidal_model::getJointAngles(nominalState, centroidalModelInfo_)
          .head(jointCount) = defaultJointState_.head(jointCount);
    }

    const auto footPositionsInWorld =
        computeFootPositionsInWorld(*pinocchioInterface_, centroidalModelInfo_,
                                    contactNames3DoF_, nominalState);
    const auto supportPlane =
        estimateSupportPlaneFromLowestFeet(footPositionsInWorld);
    return signedDistanceToSupportPlane(vector3_t(0.0, 0.0, comHeight_),
                                        supportPlane);
  }

  static constexpr const char* kRobotName = "hexbot";

  rclcpp::Node::SharedPtr node_;
  TargetTrajectoriesRosPublisher targetPublisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_;
  rclcpp::Subscription<ocs2_msgs::msg::ModeSchedule>::SharedPtr
      modeScheduleSubscriber_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observationSubscriber_;
  rclcpp::TimerBase::SharedPtr publishTimer_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist latestCmdVel_;
  geometry_msgs::msg::Twist lastMovingCmdVel_;
  std::chrono::steady_clock::time_point latestCmdVelTime_;
  std::chrono::steady_clock::time_point lastMovingCmdVelTime_{};
  SystemObservation latestObservation_;
  bool hasObservation_ = false;
  bool hasSeenMovingCmdVel_ = false;
  scalar_t lastObservationTime_ = 0.0;
  size_t lastObservedMode_ = static_cast<size_t>(ModeNumber::STANCE);
  scalar_t lastObservedModeChangeTime_ = 0.0;
  bool hasObservedModeChangeTime_ = false;
  ModeSequenceTemplate latestModeSequenceTemplate_{
      {0.0, 0.5}, {static_cast<size_t>(ModeNumber::STANCE)}};
  bool hasModeSequenceTemplate_ = false;

  std::string taskFile_;
  std::string urdfFile_;
  std::string referenceFile_;
  std::string cmdVelTopic_;
  std::string modeScheduleTopic_;
  std::string observationTopic_;
  scalar_t lookaheadTime_ = 0.35;
  scalar_t translationLookaheadTime_ = 1.20;
  scalar_t rotationLookaheadTime_ = 1.20;
  scalar_t publishRate_ = 20.0;
  scalar_t minTargetHorizon_ = 0.2;
  scalar_t cmdVelTimeout_ = 0.5;
  scalar_t movingCmdHoldTime_ = 3.0;
  bool lockNominalHeightToObservation_ = true;
  bool useLatchedObservedJointTargetWhenStationary_ = true;
  scalar_t stationaryJointTargetBlend_ = 0.20;
  scalar_t stationaryJointTargetMaxJointSpeedRadS_ = 0.80;
  bool useObservedJointTargetWhileMoving_ = true;
  bool useContactConsistentTargetStateWhileMoving_ = true;
  int movingTargetKnotCount_ = 4;
  int wholeBodyIkMaxIterations_ = 12;
  scalar_t wholeBodyIkDamping_ = 0.01;
  scalar_t wholeBodyIkJointRegularization_ = 0.05;
  scalar_t wholeBodyIkStanceFootWeight_ = 1.0;
  scalar_t wholeBodyIkSwingFootWeight_ = 0.50;
  scalar_t wholeBodyIkMaxDeltaPerIteration_ = 0.18;
  scalar_t wholeBodyIkFootTolerance_ = 0.0025;
  scalar_t phaseTransitionStanceTime_ = 0.20;
  scalar_t targetDisplacementVelocity_ = 0.15;
  scalar_t targetRotationVelocity_ = 0.6;
  scalar_t comHeight_ = 0.12;
  scalar_t nominalBaseHeightAboveSupportPlane_ = 0.12;
  mutable SupportPlane latchedSupportPlane_;
  mutable bool latchedSupportPlaneValid_ = false;
  vector_t defaultJointState_ = vector_t::Zero(18);
  vector_t movingGoalJointState_ = vector_t::Zero(18);
  mutable vector_t stationaryJointTarget_ = vector_t::Zero(18);
  mutable bool hasStationaryJointTarget_ = false;
  std::string movingGoalReferenceFile_;
  bool hasMovingGoalJointState_ = false;
  CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<CentroidalModelRbdConversions> rbdConversions_;
  mutable std::unique_ptr<PinocchioInterface> pinocchioInterface_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> contactNames3DoF_;
  vector_t guardedJointLowerLimits_ = vector_t::Constant(18, -3.14);
  vector_t guardedJointUpperLimits_ = vector_t::Constant(18, 3.14);
  bool useCentroidalTargetState_ = false;
};

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hexbot_cmd_vel_target");
  auto app = std::make_shared<HexbotCmdVelTargetNode>(node);
  (void)app;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
