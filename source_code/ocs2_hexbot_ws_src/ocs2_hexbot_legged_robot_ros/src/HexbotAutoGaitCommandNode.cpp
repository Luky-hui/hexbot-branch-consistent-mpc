/******************************************************************************
Copyright (c) 2026. All rights reserved.
******************************************************************************/

#include <geometry_msgs/msg/twist.hpp>
#include <ocs2_msgs/msg/mode_schedule.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "ocs2_hexbot_legged_robot/gait/ModeSequenceTemplate.h"
#include "ocs2_hexbot_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include "rclcpp/rclcpp.hpp"

using namespace ocs2;
using namespace ocs2::legged_robot;

namespace {

class HexbotAutoGaitCommandNode {
 public:
  explicit HexbotAutoGaitCommandNode(rclcpp::Node::SharedPtr node)
      : node_(std::move(node)) {
    gaitCommandFile_ =
        node_->declare_parameter<std::string>("gaitCommandFile", "");
    if (gaitCommandFile_.empty()) {
      throw std::runtime_error(
          "[HexbotAutoGaitCommandNode] Parameter 'gaitCommandFile' is "
          "required.");
    }

    cmdVelTopic_ = node_->declare_parameter<std::string>(
        "cmdVelTopic", "/hexbot_mpc/input/cmd_vel");
    stanceGaitName_ =
        node_->declare_parameter<std::string>("stanceGait", "stance");
    movingGaitName_ =
        node_->declare_parameter<std::string>("movingGait", "tripod_cycle");
    stationaryLinearThreshold_ =
        node_->declare_parameter<double>("stationaryLinearThreshold", 0.02);
    stationaryAngularThreshold_ =
        node_->declare_parameter<double>("stationaryAngularThreshold", 0.05);
    cmdVelTimeout_ = node_->declare_parameter<double>("cmdVelTimeout", 0.50);
    movingHoldTime_ = node_->declare_parameter<double>("movingHoldTime", 3.00);
    publishRate_ = node_->declare_parameter<double>("publishRate", 10.0);
    republishPeriod_ = node_->declare_parameter<double>("republishPeriod", 1.0);

    stanceGait_ = loadModeSequenceTemplate(gaitCommandFile_, stanceGaitName_,
                                          false);
    movingGait_ = loadModeSequenceTemplate(gaitCommandFile_, movingGaitName_,
                                          false);

    publisher_ = node_->create_publisher<ocs2_msgs::msg::ModeSchedule>(
        std::string(kRobotName) + "_mpc_mode_schedule", 1);
    cmdVelSubscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopic_, 10,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latestCmdVel_ = *msg;
          latestCmdVelTime_ = std::chrono::steady_clock::now();
          if (isMovingCommand(*msg)) {
            lastMovingCmdVelTime_ = latestCmdVelTime_;
            hasSeenMovingCmdVel_ = true;
          }
        });

    latestCmdVelTime_ = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / publishRate_);
    publishTimer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publishSelectedGait(); });

    RCLCPP_INFO(
        node_->get_logger(),
        "Hexbot auto gait node ready. stance=%s moving=%s cmd_vel=%s topic=%s_mpc_mode_schedule",
        stanceGaitName_.c_str(), movingGaitName_.c_str(), cmdVelTopic_.c_str(),
        kRobotName);
  }

 private:
  bool isMovingCommand(const geometry_msgs::msg::Twist& cmdVel) const {
    return std::hypot(cmdVel.linear.x, cmdVel.linear.y) >
               stationaryLinearThreshold_ ||
           std::abs(cmdVel.angular.z) > stationaryAngularThreshold_;
  }

  void publishSelectedGait() {
    geometry_msgs::msg::Twist cmdVel;
    std::chrono::steady_clock::time_point latestCmdVelTime;
    std::chrono::steady_clock::time_point lastMovingCmdVelTime;
    bool hasSeenMovingCmdVel = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmdVel = latestCmdVel_;
      latestCmdVelTime = latestCmdVelTime_;
      lastMovingCmdVelTime = lastMovingCmdVelTime_;
      hasSeenMovingCmdVel = hasSeenMovingCmdVel_;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool cmdVelStale =
        std::chrono::duration<double>(now - latestCmdVelTime).count() >
        cmdVelTimeout_;
    const bool recentMovingCommand =
        hasSeenMovingCmdVel &&
        std::chrono::duration<double>(now - lastMovingCmdVelTime).count() <=
            movingHoldTime_;
    if (cmdVelStale && !recentMovingCommand) {
      cmdVel = geometry_msgs::msg::Twist();
    }

    const bool moving = isMovingCommand(cmdVel) || recentMovingCommand;
    const std::string desiredGaitName =
        moving ? movingGaitName_ : stanceGaitName_;
    const auto& desiredGait = moving ? movingGait_ : stanceGait_;

    const bool gaitChanged = desiredGaitName != lastPublishedGaitName_;
    const bool republishExpired =
        std::chrono::duration<double>(now - lastPublishTime_).count() >
        republishPeriod_;

    if (!gaitChanged && !republishExpired) {
      return;
    }

    publisher_->publish(createModeSequenceTemplateMsg(desiredGait));
    if (gaitChanged) {
      RCLCPP_INFO(
          node_->get_logger(),
          "[HexbotAutoGaitCommandNode] Switching gait to %s. cmd_vel=(%.3f, %.3f, %.3f) stale=%s hold_active=%s",
          desiredGaitName.c_str(), cmdVel.linear.x, cmdVel.linear.y,
          cmdVel.angular.z, cmdVelStale ? "true" : "false",
          recentMovingCommand ? "true" : "false");
    }
    lastPublishedGaitName_ = desiredGaitName;
    lastPublishTime_ = now;
  }

  static constexpr const char* kRobotName = "hexbot";

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_;
  rclcpp::TimerBase::SharedPtr publishTimer_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist latestCmdVel_;
  std::chrono::steady_clock::time_point latestCmdVelTime_;
  std::chrono::steady_clock::time_point lastMovingCmdVelTime_{};
  std::chrono::steady_clock::time_point lastPublishTime_{};

  std::string gaitCommandFile_;
  std::string cmdVelTopic_;
  std::string stanceGaitName_;
  std::string movingGaitName_;
  std::string lastPublishedGaitName_;
  scalar_t stationaryLinearThreshold_ = 0.02;
  scalar_t stationaryAngularThreshold_ = 0.05;
  scalar_t cmdVelTimeout_ = 0.5;
  scalar_t movingHoldTime_ = 3.0;
  scalar_t publishRate_ = 10.0;
  scalar_t republishPeriod_ = 1.0;
  bool hasSeenMovingCmdVel_ = false;

  ModeSequenceTemplate stanceGait_{{0.0, 1.0}, {0}};
  ModeSequenceTemplate movingGait_{{0.0, 1.0}, {0}};
};

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hexbot_auto_gait_command");
  auto app = std::make_shared<HexbotAutoGaitCommandNode>(node);
  (void)app;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
