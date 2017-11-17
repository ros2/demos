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

#include "logging_demo/logger_config_component.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "logging_demo/srv/config_logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace logging_demo
{

LoggerConfig::LoggerConfig()
: Node("logger_config")
{
  auto handle_logger_config_req =
    [this](
    const std::shared_ptr<logging_demo::srv::ConfigLogger::Request> request,
    std::shared_ptr<logging_demo::srv::ConfigLogger::Response> response
    ) -> void
    {
      const char * severity_string = request->severity_threshold.c_str();
      RCLCPP_INFO(
        this->get_name(), "Incoming request: logger '%s', severity '%s'",
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
          this->get_name(), "Unknown severity '%s'", severity_string);
        response->success = false;
        return;
      }

      rcutils_logging_set_logger_severity_threshold(request->logger_name.c_str(), severity);
      response->success = true;
    };

  srv_ = create_service<logging_demo::srv::ConfigLogger>("config_logger", handle_logger_config_req);
}

}  // namespace logging_demo

#include "class_loader/class_loader_register_macro.h"

CLASS_LOADER_REGISTER_CLASS(logging_demo::LoggerConfig, rclcpp::Node)
