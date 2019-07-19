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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rttest/utils.h"

#include "pendulum_msgs/msg/joint_command.hpp"

using namespace std::chrono_literals;

namespace pendulum_control
{
class PendulumTeleop : public rclcpp::Node
{
public:
  explicit PendulumTeleop(const rclcpp::NodeOptions & options)
  : Node("pendulum_teleop", options)
  {
    double command  = M_PI/2;
    std::vector<std::string> args = options.arguments();
    if (args.size() < 2) {
    fprintf(stderr,
      "Command argument not specified. Setting command to 90 degrees (PI/2 radians).\n");
    } else {
      command = atof(args[1]);
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();

    auto pub =
      this->create_publisher<pendulum_msgs::msg::JointCommand>("pendulum_setpoint", qos);

    auto msg = std::make_unique<pendulum_msgs::msg::JointCommand>();
    msg->position = command;

    rclcpp::sleep_for(500ms);
    pub->publish(std::move(msg));
    rclcpp::spin_some(this);
    printf("Teleop message published.\n");
    rclcpp::sleep_for(1s);
    printf("Teleop node exited.\n");

    rclcpp::shutdown();
  }
};

}  // namespace pendulum_control

RCLCPP_COMPONENTS_REGISTER_NODE(pendulum_control::PendulumTeleop)
