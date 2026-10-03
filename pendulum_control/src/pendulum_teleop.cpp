// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <fstream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rttest/utils.hpp"

#include "pendulum_msgs/msg/joint_command.hpp"

using namespace std::chrono_literals;

// Non real-time safe node for publishing a user-specified pendulum setpoint exactly once

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  double command = M_PI / 2;
  if (argc < 2) {
    fprintf(
      stderr,
      "Command argument not specified. Setting command to 90 degrees (PI/2 radians).\n");
  } else {
    command = atof(argv[1]);
  }

  auto teleop_node = rclcpp::Node::make_shared("pendulum_teleop");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();

  auto pub =
    teleop_node->create_publisher<pendulum_msgs::msg::JointCommand>("pendulum_setpoint", qos);

  auto msg = std::make_unique<pendulum_msgs::msg::JointCommand>();
  msg->position = command;

  rclcpp::sleep_for(500ms);
  pub->publish(std::move(msg));
  rclcpp::spin_some(teleop_node);
  printf("Teleop message published.\n");
  rclcpp::sleep_for(1s);
  printf("Teleop node exited.\n");

  rclcpp::shutdown();

  return 0;
}
