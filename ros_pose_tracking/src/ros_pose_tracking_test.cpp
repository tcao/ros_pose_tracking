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

#include <thread>
#include "ros_pose_tracking/ros_pose_tracking.hpp"

namespace ros_pose_tracking_test
{
namespace
{
static const char defaultNodeName[] = "ros_pose_tracking_test";
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ros_pose_tracking_test");
}  // anonymous namespace

class PoseTrackingTest : public rclcpp::Node {
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

}

virtual ~PoseTrackingTest()
{
  if (thread_.joinable()) {
    thread_.join();
  }

}

void loop()
{
  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped::UniquePtr msg(new geometry_msgs::msg::PoseStamped());    
  }
}

private:
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
std::thread thread_;
bool starting_;
};
} // namespace ros_pose_tracking_test

int main(int argc, char ** argv)
{
  // Install user break handler
  std::signal(SIGINT, ros_pose_tracking::signal_handler);
  std::signal(SIGTERM, ros_pose_tracking::signal_handler);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTrackingTest>(
    ros_pose_tracking_test::defaultNodeName,
    rclcpp::NodeOptions().allow_undeclared_parameters(true)
  ));
  rclcpp::shutdown();
}
