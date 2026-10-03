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

#ifndef LOGGING_DEMO__LOGGER_CONFIG_COMPONENT_HPP_
#define LOGGING_DEMO__LOGGER_CONFIG_COMPONENT_HPP_

#include <memory>

#include "logging_demo/srv/config_logger.hpp"
#include "logging_demo/visibility_control.h"

#include "rclcpp/rclcpp.hpp"

namespace logging_demo
{

class LoggerConfig : public rclcpp::Node
{
public:
  LOGGING_DEMO_PUBLIC
  explicit LoggerConfig(rclcpp::NodeOptions options);

  LOGGING_DEMO_PUBLIC
  void
  handle_logger_config_request(
    const std::shared_ptr<logging_demo::srv::ConfigLogger::Request> request,
    std::shared_ptr<logging_demo::srv::ConfigLogger::Response> response);

private:
  rclcpp::Service<logging_demo::srv::ConfigLogger>::SharedPtr srv_;
};

}  // namespace logging_demo

#endif  // LOGGING_DEMO__LOGGER_CONFIG_COMPONENT_HPP_
