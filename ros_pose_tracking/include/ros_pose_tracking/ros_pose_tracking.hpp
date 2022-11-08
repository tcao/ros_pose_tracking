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
#ifndef ROS_POSE_TRACKING__ROS_POSE_TRACKING_HPP_
#define ROS_POSE_TRACKING__ROS_POSE_TRACKING_HPP_

// C++
#include <stdbool.h>

// STD
#include <string>
#include <vector>

// ROS2 includes
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

// geometry_msgs - PoseStamped, PointStamp
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.h>

// MoveIt
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace ros_pose_tracking
{
namespace
{
// Default subscription topic name
static const char defaultPoseCommandingTopic[] = "/tracking_pose";
static const char defaultMoveGroupName[] = "panda_arm"; // For Panda 7DOF ARM
                                                        // For Universal Robots "ur_manipulator"
}  // anonymous namespace

// Non-ROS parameters
struct cli_parameters_t
{
  std::string group_name;
};

typedef bool (* moveit_pose_request_t)(const geometry_msgs::msg::Pose &);

class PoseTracking : public rclcpp::Node
{
public:
  PoseTracking(
    const std::string name, std::string group_name,
    rclcpp::NodeOptions node_options,
    moveit_pose_request_t plan_execution_callback = nullptr,
    moveit_pose_request_t execution_callback = nullptr);

  void configure();

  static bool arguments_parse(
    int argc, char * argv[], ros_pose_tracking::cli_parameters_t * parameters)
  {
    if (1 == argc) {
      // Nothing to parse
      return true;
    }
    bool help = false;

    // arguments parsing loop
    std::vector<std::string> args(argv + 1, argv + argc);
    for (auto i = args.begin(); i != args.end(); i++) {
      if ("--help" == *i) {
        help = true;
        break;
      }
      if ("-g" == *i) {
        // No checking what's specified
        parameters->group_name = *++i;
        std::cout << "group name specified: " << parameters->group_name << std::endl;
      }
    }

    if (help) {
      std::cout << argv[0] << " [--help] [-g <group_name>]" << std::endl;
      std::cout << "-g (optional) to specify robot move group's name, default=" <<
        parameters->group_name << std::endl;
    }
    return !help;
  }

private:
  std::string group_name_;
  // Move group interface for the robot - It is very picky for way it is instantiated
  // moveit::planning_interface::MoveGroupInterface move_group_interface_;
  // Subscriber to target pose target_pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr commanding_pose_sub_;

  moveit_pose_request_t plan_execution_callback_;
  moveit_pose_request_t execution_callback_;
};

}  // namespace ros_pose_tracking

#endif  //  ROS_POSE_TRACKING__ROS_POSE_TRACKING_HPP_
