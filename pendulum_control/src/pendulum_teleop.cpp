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

#include "rclcpp/rclcpp.hpp"

#include "rttest/utils.h"

#include "pendulum_msgs/msg/joint_command.hpp"

using namespace std::chrono_literals;

// Non real-time safe node for publishing a user-specified pendulum setpoint exactly once

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  double command = M_PI / 2;
  if (argc < 2) {
    fprintf(stderr,
      "Command argument not specified. Setting command to 90 degrees (PI/2 radians).\n");
  } else {
    command = atof(argv[1]);
  }

  auto teleop_node = rclcpp::node::Node::make_shared("pendulum_teleop");

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  auto pub = teleop_node->create_publisher<pendulum_msgs::msg::JointCommand>(
    "pendulum_setpoint", qos_profile);

  auto msg = std::make_shared<pendulum_msgs::msg::JointCommand>();
  msg->position = command;

  rclcpp::sleep_for(500ms);
  pub->publish(msg);
  rclcpp::spin_some(teleop_node);
  std::cout << "Teleop node exited." << std::endl;

  rclcpp::shutdown();
}
