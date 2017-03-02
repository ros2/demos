// Copyright 2017 Open Source Robotics Foundation, Inc.
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
// limitations under the License.

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#define DEG2RAD M_PI/180.0

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("dummy_joint_states");

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states");

  rclcpp::WallRate loop_rate(50);

  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->name.push_back("single_rrbot_joint1");
  msg->name.push_back("single_rrbot_joint2");
  msg->position.push_back(0.0);
  msg->position.push_back(0.0);

  auto counter = 0.0;
  auto joint_value = 0.0;
  while (rclcpp::ok()) {
    counter += 0.1;
    joint_value = std::sin(counter);

    for (size_t i = 0; i < msg->name.size(); ++i) {
      msg->position[i] = joint_value;
    }

    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    if (now <= std::chrono::nanoseconds(0)) {
      msg->header.stamp.sec = msg->header.stamp.nanosec = 0;
    } else {
      msg->header.stamp.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
      msg->header.stamp.nanosec = now.count() % 1000000000;
    }

    joint_state_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
