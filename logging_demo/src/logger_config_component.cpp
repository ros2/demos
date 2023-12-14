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

#include "logging_demo/logger_config_component.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "logging_demo/srv/config_logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/logging.h"

namespace logging_demo
{

LoggerConfig::LoggerConfig(rclcpp::NodeOptions options)
: Node("logger_config", options)
{
  srv_ = create_service<logging_demo::srv::ConfigLogger>(
    "config_logger", [this](
      const std::shared_ptr<logging_demo::srv::ConfigLogger::Request> request,
      std::shared_ptr<logging_demo::srv::ConfigLogger::Response> response) {
      return this->handle_logger_config_request(request, response);
    }
  );
}

void
LoggerConfig::handle_logger_config_request(
  const std::shared_ptr<logging_demo::srv::ConfigLogger::Request> request,
  std::shared_ptr<logging_demo::srv::ConfigLogger::Response> response)
{
  const char * severity_string = request->level.c_str();
  RCLCPP_INFO(
    this->get_logger(), "Incoming request: logger '%s', severity '%s'",
    request->logger_name.c_str(), severity_string);
  std::flush(std::cout);

  int severity;
  rcutils_ret_t ret = rcutils_logging_severity_level_from_string(
    severity_string, rcl_get_default_allocator(), &severity);
  if (RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID == ret) {
    RCLCPP_ERROR(
      this->get_logger(), "Unknown severity '%s'", severity_string);
    response->success = false;
    return;
  }
  if (RCUTILS_RET_OK != ret) {
    RCLCPP_ERROR(
      this->get_logger(), "Error %d getting severity level from request: %s", ret,
      rcl_get_error_string().str);
    rcl_reset_error();
    response->success = false;
    return;
  }

  // TODO(dhood): allow configuration through rclcpp
  ret = rcutils_logging_set_logger_level(request->logger_name.c_str(), severity);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(
      this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
    response->success = false;
  }
  response->success = true;
}

}  // namespace logging_demo

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(logging_demo::LoggerConfig)
