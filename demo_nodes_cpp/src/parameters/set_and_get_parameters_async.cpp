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

#include <memory>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("set_and_get_parameters_async");

  // TODO(wjwwood): Make the parameter service automatically start with the node.
  auto parameter_service = std::make_shared<rclcpp::ParameterService>(node);

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.")
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...")
  }

  // Set several different types of parameters.
  auto results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("foo", 2),
    rclcpp::parameter::ParameterVariant("bar", "hello"),
    rclcpp::parameter::ParameterVariant("baz", 1.45),
    rclcpp::parameter::ParameterVariant("foobar", true),
    rclcpp::parameter::ParameterVariant("foobarbaz", std::vector<bool>({true, false})),
    rclcpp::parameter::ParameterVariant("toto", std::vector<uint8_t>({0xff, 0x7f})),
  });
  // Wait for the results.
  if (rclcpp::spin_until_future_complete(node, results) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "set_parameters service call failed. Exiting tutorial.")
    return -1;
  }
  // Check to see if they were set.
  for (auto & result : results.get()) {
    if (!result.successful) {
      RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", result.reason.c_str())
    }
  }

  // Get a few of the parameters just set.
  auto parameters = parameters_client->get_parameters({"foo", "baz", "foobarbaz", "toto"});
  if (rclcpp::spin_until_future_complete(node, parameters) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get_parameters service call failed. Exiting tutorial.")
    return -1;
  }
  std::stringstream ss;
  for (auto & parameter : parameters.get()) {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string();
  }
  RCLCPP_INFO(node->get_logger(), ss.str().c_str())

  rclcpp::shutdown();

  return 0;
}
