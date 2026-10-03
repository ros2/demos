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

#ifndef LOGGING_DEMO__LOGGER_USAGE_COMPONENT_HPP_
#define LOGGING_DEMO__LOGGER_USAGE_COMPONENT_HPP_

#include <string>

#include "logging_demo/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace logging_demo
{

class LoggerUsage : public rclcpp::Node
{
public:
  LOGGING_DEMO_PUBLIC
  explicit LoggerUsage(rclcpp::NodeOptions options);

protected:
  void on_timer();

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_, timer_;
  std::function<bool()> debug_function_to_evaluate_;
};

bool is_divisor_of_twelve(size_t val, rclcpp::Logger logger);
}  // namespace logging_demo

#endif  // LOGGING_DEMO__LOGGER_USAGE_COMPONENT_HPP_
