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
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{

class OneOffTimerNode : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit OneOffTimerNode(const rclcpp::NodeOptions & options)
  : Node("one_off_timer", options), count(0)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    periodic_timer = this->create_wall_timer(
      2s,
      [this]() {
        RCLCPP_INFO(this->get_logger(), "in periodic_timer callback");
        if (this->count++ % 3 == 0) {
          RCLCPP_INFO(this->get_logger(), "  resetting one off timer");
          this->one_off_timer = this->create_wall_timer(
            1s, [this]() {
              RCLCPP_INFO(this->get_logger(), "in one_off_timer callback");
              this->one_off_timer->cancel();
            });
        } else {
          RCLCPP_INFO(this->get_logger(), "  not resetting one off timer");
        }
      });
  }

  rclcpp::TimerBase::SharedPtr periodic_timer;
  rclcpp::TimerBase::SharedPtr one_off_timer;
  size_t count;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::OneOffTimerNode)
