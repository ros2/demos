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
#include <cmath>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("dummy_joint_states");

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  rclcpp::WallRate loop_rate(50);

  sensor_msgs::msg::JointState msg;
  msg.name.push_back("single_rrbot_joint1");
  msg.name.push_back("single_rrbot_joint2");
  msg.position.push_back(0.0);
  msg.position.push_back(0.0);

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  auto counter = 0.0;
  auto joint_value = 0.0;
  while (rclcpp::ok()) {
    counter += 0.1;
    joint_value = std::sin(counter);

    for (size_t i = 0; i < msg.name.size(); ++i) {
      msg.position[i] = joint_value;
    }

    msg.header.stamp = clock->now();

    joint_state_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
