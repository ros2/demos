// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "logging_demo/logger_usage_component.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace logging_demo
{

LoggerUsage::LoggerUsage()
: Node("logger_usage_demo"), count_(0)
{
  pub_ = create_publisher<std_msgs::msg::String>("logging_demo_count");
  timer_ = create_wall_timer(1s, std::bind(&LoggerUsage::on_timer, this));
  debug_function_to_evaluate_ = std::bind(divides_into_twelve, std::cref(count_), get_name());
}

void LoggerUsage::on_timer()
{
  // This message will be printed only the first time this line is reached.
  RCLCPP_INFO_ONCE(get_name(), "Timer callback called")

  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "Current count: " + std::to_string(count_);

  // This message will be printed each time it is reached.
  RCLCPP_INFO(get_name(), "Publishing: '%s'", msg->data.c_str())
  pub_->publish(msg);

  // This message will be printed when the function evaluates to true.
  // The function will only be evaluated when the severity is enabled.
  // This is useful if calculation of debug output is computationally expensive.
  RCLCPP_DEBUG_FUNCTION(get_name(), &debug_function_to_evaluate_, "Count divides into 12")

  // This message will be printed when the expression evaluates to true.
  // The expression will only be evaluated when the severity is enabled.
  RCLCPP_DEBUG_EXPRESSION(get_name(), (count_ % 2) == 0, "Count is even")
  std::flush(std::cout);
  if (count_++ >= 15) {
    RCLCPP_WARN(get_name(), "Reseting count to 0")
    count_ = 0;
  }
}

bool divides_into_twelve(int x, std::string logger_name) {
  // This method is called from within a RCLCPP_DEBUG_FUNCTION() call.
  // Therefore it will only be called when DEBUG log messages are enabled.

  if (x == 0) {
    RCLCPP_ERROR(logger_name, "Modulo divisor cannot be 0")
    return false;
  }
  return (12 % x) == 0;
}

}  // namespace logging_demo

#include "class_loader/class_loader_register_macro.h"

CLASS_LOADER_REGISTER_CLASS(logging_demo::LoggerUsage, rclcpp::Node)
