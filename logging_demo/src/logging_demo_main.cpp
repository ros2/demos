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

#include <memory>

#include "logging_demo/logger_config_component.hpp"
#include "logging_demo/logger_usage_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  // Create a node that processes logger configuration requests
  auto logger_config = std::make_shared<logging_demo::LoggerConfig>(options);
  exec.add_node(logger_config);
  // Create a node that has examples of different logger usage
  auto logger_usage = std::make_shared<logging_demo::LoggerUsage>(options);
  exec.add_node(logger_usage);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
