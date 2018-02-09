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

namespace logging_demo
{

LoggerConfig::LoggerConfig()
: Node("logger_config")
{
  srv_ = create_service<logging_demo::srv::ConfigLogger>(
    "config_logger", std::bind(
      &LoggerConfig::handle_logger_config_request,
      this, std::placeholders::_1, std::placeholders::_2));
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
  if (strcmp("DEBUG", severity_string) == 0) {
    severity = RCUTILS_LOG_SEVERITY_DEBUG;
  } else if (strcmp("INFO", severity_string) == 0) {
    severity = RCUTILS_LOG_SEVERITY_INFO;
  } else if (strcmp("WARN", severity_string) == 0) {
    severity = RCUTILS_LOG_SEVERITY_WARN;
  } else if (strcmp("ERROR", severity_string) == 0) {
    severity = RCUTILS_LOG_SEVERITY_ERROR;
  } else if (strcmp("FATAL", severity_string) == 0) {
    severity = RCUTILS_LOG_SEVERITY_FATAL;
  } else if (strcmp("UNSET", severity_string) == 0) {
    severity = RCUTILS_LOG_SEVERITY_UNSET;
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Unknown severity '%s'", severity_string);
    response->success = false;
    return;
  }

  // TODO(dhood): allow configuration through rclcpp
  auto ret = rcutils_logging_set_logger_level(request->logger_name.c_str(), severity);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(
      this->get_logger(), "Error setting severity: %s", rcutils_get_error_string_safe());
    rcutils_reset_error();
    response->success = false;
  }
  response->success = true;
}

}  // namespace logging_demo

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(logging_demo::LoggerConfig, rclcpp::Node)
