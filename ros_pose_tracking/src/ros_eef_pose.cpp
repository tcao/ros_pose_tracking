//
// Copyright 2022 Ting Cao <cao_ting@yahoo.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and

///
/// "General" ROS2 Target Pose Tracking Application
/// @remark "General" is a relative term, I try my best
///
/// @remark This code deals with a robot with moveit move group defined
///

// STD
#include <memory>
#include <chrono>

#include "ros_pose_tracking/ros_pose_tracking.hpp"

namespace ros_eef_pose
{
namespace
{
static const char defaultNodeName[] = "eef_pose";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ros_eef_pose");
}  // anonymous namespace
}  // namespace ros_eef_pose

bool ros_pose_tracking::PoseTracking::stopped_ = false;

int main(int argc, char ** argv)
{
  // Parameters:
  // 1) needed before ROS is initiated, or
  // 2) not related to ROS
  static ros_pose_tracking::cli_parameters_t non_ros_parameters = {
    .group_name = ros_pose_tracking::defaultMoveGroupName,
  };
  if (!ros_pose_tracking::PoseTracking::arguments_parse(argc, argv, &non_ros_parameters)) {
    return 0;
  }

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const eef_pose_querier = rclcpp::Node::make_shared(
    ros_eef_pose::defaultNodeName,
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(eef_pose_querier);
  std::thread([&executor]() {executor.spin();}).detach();

  // MoveGroupInterface instantiation takes long time
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(
    eef_pose_querier, non_ros_parameters.group_name);

  std::string planning_frame(move_group_interface_arm.getPlanningFrame());
  RCLCPP_INFO(ros_eef_pose::LOGGER, "Planning frame: %s", planning_frame.c_str());
  std::string eef_link_name(move_group_interface_arm.getEndEffectorLink());
  RCLCPP_INFO(ros_eef_pose::LOGGER, "End effector link: %s", eef_link_name.c_str());

  geometry_msgs::msg::PoseStamped current_pose;
  current_pose = move_group_interface_arm.getCurrentPose(eef_link_name);
  geometry_msgs::msg::Pose eef_pose;
  eef_pose = current_pose.pose;
  RCLCPP_INFO(ros_eef_pose::LOGGER, "EEF pose:");
  std::cout << eef_pose.position.x << "/" << eef_pose.position.y << "/" << eef_pose.position.z <<
    ", " << eef_pose.orientation.x << "/" << eef_pose.orientation.y << "/" <<
    eef_pose.orientation.z << "/" << eef_pose.orientation.w << std::endl << std::endl;

  rclcpp::shutdown();
  return 0;
}
