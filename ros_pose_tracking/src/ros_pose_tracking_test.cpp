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
/// ROS2 Target Pose Tracking Test Application
///
/// @remark This code submits pose tracking commands repeatly in a cycle
/// according poses listed in the input yaml
/// @remark See pose_tracking_test.yaml for the format (fields and values)
///
#include <stdint.h>
#include <thread>
#include <chrono>

#include "ros_pose_tracking/ros_pose_tracking.hpp"

namespace ros_pose_tracking_test
{
namespace
{
using namespace std::chrono_literals;
// @remark This node name must match up with root element's name from the passing yaml data
static const char defaultNodeName[] = "pose_tracking_test";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ros_pose_tracking_test");
static const std::chrono::milliseconds kStartingWait(100);
static const int kDefaultPoseWait(1000);
static const char testingPoseNames[] = "pose_names";
static const char testingPoseDurationPostfix[] = "_wait";
static const char kDefaultPoseFrameID[] = "map";
static const uint8_t kPoseValueCount(7);
}  // anonymous namespace

class PoseTrackingTest : public rclcpp::Node
{
public:
  struct PoseTrackingTestEntry
  {
    std::shared_ptr<geometry_msgs::msg::PoseStamped> pose;
    std::chrono::microseconds wait_ms;
  };

public:
  PoseTrackingTest(const std::string name, rclcpp::NodeOptions node_options)
  : Node(name, node_options),
    starting_(false)
  {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      ros_pose_tracking::defaultPoseCommandingTopic,
      1
    );
    thread_ = std::thread(std::bind(&PoseTrackingTest::loop, this));

    // Don't block node instantiation
    // Let it spins in a separate thread to load all testing data parameters
    std::thread(
      [this]() {
        // Run the streaming for this amount of time in seconds
        std::this_thread::sleep_for(10 * kStartingWait);

        declare_parameter("controller_name", "");
        std::string controller_name;
        rclcpp::Parameter current_param = get_parameter("controller_name");
        // Or the following
        // get_parameter("controller_name", controller_name);
        std::cout << "controller_name: " << current_param.as_string() << std::endl;

        std::vector<std::string> pose_names{};
        declare_parameter(testingPoseNames, pose_names);
        current_param = get_parameter(testingPoseNames);
        if (0 == current_param.as_string_array().size()) {
          std::cerr << "no pose names are found, check testing data yaml file" << std::endl;
          return;
        }
        pose_names = current_param.as_string_array();
        pose_entries_.resize(pose_names.size());
        int pose_entry = 0;
        for (std::string pose_name : pose_names) {
          // declare and assign indivisual pose
          std::vector<double> pose{};
          declare_parameter(pose_name, pose);
          current_param = get_parameter(pose_name);
          pose = current_param.as_double_array();
          if (kPoseValueCount > pose.size()) {
            std::cerr << "Expecting " << kPoseValueCount <<
              " values for each pose, check testing data yaml file" << std::endl;
            return;
          }
          std::cout << pose_name << ": ";
          int count = pose.size(), i = 1;
          for (double pose_value : pose) {
            if (i >= count) {
              std::cout << pose_value << std::endl;
            } else {
              std::cout << pose_value << ", ";
            }
            i++;
          }
          // declare and assign pose_wait
          std::string pose_wait_param = pose_name + testingPoseDurationPostfix;
          declare_parameter(pose_wait_param, kDefaultPoseWait);
          current_param = get_parameter(pose_wait_param);
          int pose_wait = current_param.as_int();
          std::cout << "\twait: " << pose_wait << std::endl;

          PoseTrackingTestEntry * entry = &pose_entries_[pose_entry++];
          entry->pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
          // make sure don't store an actual stamp
          // since that will become stale can potentially cause tf errors
          entry->pose->header.stamp = rclcpp::Time(0);
          entry->pose->header.frame_id = kDefaultPoseFrameID;
          entry->pose->pose.position.x = pose[0];
          entry->pose->pose.position.y = pose[1];
          entry->pose->pose.position.x = pose[2];
          entry->pose->pose.orientation.x = pose[3];
          entry->pose->pose.orientation.y = pose[4];
          entry->pose->pose.orientation.z = pose[5];
          entry->pose->pose.orientation.w = pose[6];
          entry->wait_ms = std::chrono::milliseconds(pose_wait);
        }
        // All data is prepared, notify publishing thread to submit pose tracking commands
        starting_ = true;
      }).detach();
  }

  virtual ~PoseTrackingTest()
  {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void loop()
  {
    while (!starting_) {
      // wait till it receives the signal to proceed
      std::this_thread::sleep_for(kStartingWait);
    }
    uint32_t loop_count = 0;
    while (rclcpp::ok()) {
      uint32_t entry_index = 0;
      for (PoseTrackingTestEntry & entry : pose_entries_) {
        // Quit if signaled
        if (ros_pose_tracking::PoseTracking::stopped_) {
          return;
        }

        RCLCPP_INFO(
          ros_pose_tracking_test::LOGGER,
          "%d/%d:\nposition: %f/%f/%f, orientation: %f/%f/%f/%f\n",
          loop_count, entry_index,
          entry.pose->pose.position.x,
          entry.pose->pose.position.y,
          entry.pose->pose.position.z,
          entry.pose->pose.orientation.x,
          entry.pose->pose.orientation.y,
          entry.pose->pose.orientation.z,
          entry.pose->pose.orientation.w
        );
        pub_->publish(*entry.pose);
        std::this_thread::sleep_for(entry.wait_ms);
        entry_index++;
      }
      loop_count++;
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  std::thread thread_;
  bool starting_;
  std::vector<PoseTrackingTestEntry> pose_entries_;
};
}  // namespace ros_pose_tracking_test

bool ros_pose_tracking::PoseTracking::stopped_ = false;

int main(int argc, char ** argv)
{
  // Install user break handler
  std::signal(SIGINT, ros_pose_tracking::PoseTracking::signal_handler);
  std::signal(SIGTERM, ros_pose_tracking::PoseTracking::signal_handler);

  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<ros_pose_tracking_test::PoseTrackingTest>(
      ros_pose_tracking_test::defaultNodeName,
      rclcpp::NodeOptions().allow_undeclared_parameters(true)
  ));
  rclcpp::shutdown();
}
