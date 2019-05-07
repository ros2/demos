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

#include "logging_demo/logger_usage_component.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace logging_demo
{

LoggerUsage::LoggerUsage(rclcpp::NodeOptions options)
: Node("logger_usage_demo", options), count_(0)
{
  pub_ = create_publisher<std_msgs::msg::String>("logging_demo_count", 10);
  timer_ = create_wall_timer(500ms, std::bind(&LoggerUsage::on_timer, this));
  debug_function_to_evaluate_ = std::bind(is_divisor_of_twelve, std::cref(count_), get_logger());

  // After 10 iterations the severity will be set to DEBUG.
  auto on_one_shot_timer =
    [this]() -> void {
      one_shot_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Setting severity threshold to DEBUG");
      // TODO(dhood): allow configuration through rclcpp
      auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }
    };
  one_shot_timer_ = create_wall_timer(5500ms, on_one_shot_timer);
}

void LoggerUsage::on_timer()
{
  // This message will be logged only the first time this line is reached.
  RCLCPP_INFO_ONCE(get_logger(), "Timer callback called (this will only log once)");

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Current count: " + std::to_string(count_);

  // This message will be logged each time it is reached.
  RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg->data.c_str());
  pub_->publish(std::move(msg));

  // This message will be logged when the function evaluates to true.
  // The function will only be evaluated when DEBUG severity is enabled.
  // This is useful if calculation of debug output is computationally expensive.
  RCLCPP_DEBUG_FUNCTION(
    get_logger(), &debug_function_to_evaluate_,
    "Count divides into 12 (function evaluated to true)");

  // This message will be logged when the expression evaluates to true.
  // The expression will only be evaluated when DEBUG severity is enabled.
  RCLCPP_DEBUG_EXPRESSION(
    get_logger(), (count_ % 2) == 0, "Count is even (expression evaluated to true)");
  if (count_++ >= 15) {
    RCLCPP_WARN(get_logger(), "Reseting count to 0");
    count_ = 0;
  }
}

bool is_divisor_of_twelve(size_t val, rclcpp::Logger logger)
{
  // This method is called from within a RCLCPP_DEBUG_FUNCTION() call.
  // Therefore it will only be called when DEBUG log messages are enabled.

  if (val == 0) {
    RCLCPP_ERROR(logger, "Modulo divisor cannot be 0");
    return false;
  }
  return (12 % val) == 0;
}

}  // namespace logging_demo

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(logging_demo::LoggerUsage)
