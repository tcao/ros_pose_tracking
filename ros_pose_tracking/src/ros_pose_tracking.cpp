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
///
/// @remark This code deals with a robot with moveit move group defined
///
/// @test
/// geometry_msgs::msg::Pose target_pose;
/// target_pose.orientation.w = 1.0;
/// target_pose.position.x = 0.28;
/// target_pose.position.y = -0.2;
/// target_pose.position.z = 0.5;
///

// STD
#include <memory>
#include <chrono>

#include "ros_pose_tracking/ros_pose_tracking.hpp"

namespace ros_pose_tracking
{
namespace
{
static const char defaultNodeName[] = "pose_tracker";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ros_pose_tracking");
static const double VISUAL_AXIS_LENGTH(0.2);
static const double VISUAL_AXIS_RADIUS(0.01);
static const double SPEED_SCALE(1.0);
}  // anonymous namespace

PoseTracking::PoseTracking(
  const std::string name, std::string group_name,
  rclcpp::NodeOptions node_options,
  moveit_pose_request_t plan_execution_callback,
  moveit_pose_request_t execution_callback)
: Node(name, node_options),
  group_name_(group_name),
  plan_execution_callback_(plan_execution_callback),
  execution_callback_(execution_callback)
{
  RCLCPP_INFO(LOGGER, "%s node initialized for group: %s", name.c_str(), group_name_.c_str());
}

void PoseTracking::configure()
{
  RCLCPP_INFO(LOGGER, "%s node configured", this->get_name());
  commanding_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    defaultPoseCommandingTopic,
    rclcpp::QoS(1),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      RCLCPP_INFO(LOGGER, "Commanding pose received");
      if (execution_callback_) {
        // No need to check return status (already taken care of)
        execution_callback_(msg->pose);
      } else {
        RCLCPP_ERROR(LOGGER, "No executor is registered, so ignore pose command");
      }
    });
}
}  // namespace ros_pose_tracking

static void signal_handler(int signal)
{
  (void)signal;
  rclcpp::shutdown();
}

namespace
{
moveit::planning_interface::MoveGroupInterface * move_group_interface_ptr = nullptr;
moveit_visual_tools::MoveItVisualTools * moveit_visual_tools_ptr = nullptr;
}  // anonymous namespace

bool request_plan_execution(const geometry_msgs::msg::Pose & pose)
{
  move_group_interface_ptr->setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto success = static_cast<bool>(move_group_interface_ptr->plan(msg));

  // Execute the plan
  if (success) {
    RCLCPP_INFO(ros_pose_tracking::LOGGER, "Planning succeeded!");
    if (!static_cast<bool>(move_group_interface_ptr->execute(msg))) {
      RCLCPP_ERROR(ros_pose_tracking::LOGGER, "Execution failed!");
      success = false;
    }
  } else {
    RCLCPP_ERROR(ros_pose_tracking::LOGGER, "Planning failed!");
  }

  return success;
}

bool request_execution(const geometry_msgs::msg::Pose & pose)
{
  // show target pose visual
  moveit_visual_tools_ptr->deleteAllMarkers();
  // Eigen::Isometry3d could be used for tf transform
  moveit_visual_tools_ptr->publishAxis(
    pose,
    ros_pose_tracking::VISUAL_AXIS_LENGTH,
    ros_pose_tracking::VISUAL_AXIS_RADIUS);
  moveit_visual_tools_ptr->trigger();

  move_group_interface_ptr->setPoseTarget(pose);

  if (static_cast<bool>(move_group_interface_ptr->move())) {
    RCLCPP_INFO(ros_pose_tracking::LOGGER, "Move succeeded!");
  } else {
    RCLCPP_ERROR(ros_pose_tracking::LOGGER, "Move failed!");
    return false;
  }

  return true;
}

int main(int argc, char ** argv)
{
  static ros_pose_tracking::cli_parameters_t non_ros_parameters = {
    .group_name = ros_pose_tracking::defaultMoveGroupName,
  };
  if (!ros_pose_tracking::PoseTracking::arguments_parse(argc, argv, &non_ros_parameters)) {
    return 0;
  }

  // Install user break handler
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const tracker = std::make_shared<ros_pose_tracking::PoseTracking>(
    ros_pose_tracking::defaultNodeName,
    non_ros_parameters.group_name,
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true),
    &request_plan_execution,
    &request_execution);

  // MoveGroupInterface instantiation takes long time (loading robot description, etc).
  // When the instance is a class member, it's tricky to bring it up and release resources later
  moveit::planning_interface::MoveGroupInterface move_group_interface(
    tracker, non_ros_parameters.group_name);
  move_group_interface_ptr = &move_group_interface;

  // Use full speed
  move_group_interface.setMaxAccelerationScalingFactor(ros_pose_tracking::SPEED_SCALE);
  move_group_interface.setMaxVelocityScalingFactor(ros_pose_tracking::SPEED_SCALE);

  tracker->configure();
  std::string reference_frame(move_group_interface.getPoseReferenceFrame());
  std::string eef_link_name(move_group_interface.getEndEffectorLink());
  RCLCPP_INFO(
    ros_pose_tracking::LOGGER,
    "Reference Frame: %s, End effector link: %s",
    reference_frame.c_str(),
    eef_link_name.c_str());

  moveit_visual_tools::MoveItVisualTools visual_tools(
    tracker,
    reference_frame,
    // default viz marker_topic
    rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel());
  moveit_visual_tools_ptr = &visual_tools;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
