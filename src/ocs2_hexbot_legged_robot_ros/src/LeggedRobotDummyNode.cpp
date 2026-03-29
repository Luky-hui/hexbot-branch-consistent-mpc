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

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_hexbot_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ocs2_hexbot_legged_robot_ros/visualization/LeggedRobotVisualizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace ocs2;
using namespace legged_robot;

namespace {

class HexbotJointCommandObserver final : public DummyObserver {
 public:
  HexbotJointCommandObserver(
      const rclcpp::Node::SharedPtr& node,
      CentroidalModelInfo centroidalModelInfo,
      std::vector<std::string> jointNames,
      std::string topicName)
      : node_(node),
        centroidalModelInfo_(std::move(centroidalModelInfo)),
        jointNames_(std::move(jointNames)),
        publisher_(node_->create_publisher<sensor_msgs::msg::JointState>(
            std::move(topicName), 10)) {}

  void update(const SystemObservation& observation,
              const PrimalSolution& primalSolution,
              const CommandData&) override {
    const vector_t& state = selectCommandState(observation, primalSolution);
    const vector_t jointAngles =
        centroidal_model::getJointAngles(state, centroidalModelInfo_);

    sensor_msgs::msg::JointState jointCommand;
    jointCommand.header.stamp = node_->get_clock()->now();
    jointCommand.name = jointNames_;
    jointCommand.position.resize(jointAngles.size());
    for (Eigen::Index i = 0; i < jointAngles.size(); ++i) {
      jointCommand.position[static_cast<size_t>(i)] = jointAngles(i);
    }
    publisher_->publish(jointCommand);
  }

 private:
  const vector_t& selectCommandState(const SystemObservation& observation,
                                     const PrimalSolution& primalSolution) const {
    if (primalSolution.stateTrajectory_.size() > 1) {
      return primalSolution.stateTrajectory_[1];
    }
    if (!primalSolution.stateTrajectory_.empty()) {
      return primalSolution.stateTrajectory_.front();
    }
    return observation.state;
  }

  rclcpp::Node::SharedPtr node_;
  CentroidalModelInfo centroidalModelInfo_;
  std::vector<std::string> jointNames_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char** argv) {
  const std::string robotName = "hexbot";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared(robotName + "_mrt");

  const std::string taskFile =
      node->declare_parameter<std::string>("taskFile", "");
  const std::string urdfFile =
      node->declare_parameter<std::string>("urdfFile", "");
  const std::string referenceFile =
      node->declare_parameter<std::string>("referenceFile", "");
  const std::string backendJointCommandTopic =
      node->declare_parameter<std::string>(
          "backendJointCommandTopic", "/hexbot_mpc/output/joint_command");
  if (taskFile.empty() || urdfFile.empty() || referenceFile.empty()) {
    throw std::runtime_error(
        "[LeggedRobotDummyNode] Parameters 'taskFile', 'urdfFile', and "
        "'referenceFile' are required.");
  }

  // Robot interface
  LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(node);

  // Visualization
  CentroidalModelPinocchioMapping pinocchioMapping(
      interface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(
      interface.getPinocchioInterface(), pinocchioMapping,
      interface.modelSettings().contactNames3DoF);
  auto leggedRobotVisualizer = std::make_shared<LeggedRobotVisualizer>(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
      endEffectorKinematics, interface.modelSettings().jointNames, node);
  auto jointCommandObserver = std::make_shared<HexbotJointCommandObserver>(
      node, interface.getCentroidalModelInfo(), interface.modelSettings().jointNames,
      backendJointCommandTopic);

  // Dummy legged robot
  MRT_ROS_Dummy_Loop leggedRobotDummySimulator(
      mrt, interface.mpcSettings().mrtDesiredFrequency_,
      interface.mpcSettings().mpcDesiredFrequency_);
  leggedRobotDummySimulator.subscribeObservers(
      {leggedRobotVisualizer, jointCommandObserver});

  // Initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input =
      vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  initObservation.mode = ModeNumber::STANCE;

  // Initial command
  TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state},
                                            {initObservation.input});

  // run dummy
  leggedRobotDummySimulator.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
