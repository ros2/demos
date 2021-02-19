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

#include <chrono>
#include <memory>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{

class SetAndGetParameters : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit SetAndGetParameters(const rclcpp::NodeOptions & options)
  : Node("set_and_get_parameters", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    this->declare_parameter("foo", 0);
    this->declare_parameter("bar", "");
    this->declare_parameter("baz", 0.);
    this->declare_parameter("foobar", false);
    this->declare_parameter("foobarbaz", std::vector<bool>{});
    this->declare_parameter("toto", std::vector<uint8_t>{});

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Set several different types of parameters.
    auto set_parameters_results = parameters_client->set_parameters(
      {
        rclcpp::Parameter("foo", 2),
        rclcpp::Parameter("bar", "hello"),
        rclcpp::Parameter("baz", 1.45),
        rclcpp::Parameter("foobar", true),
        rclcpp::Parameter("foobarbaz", std::vector<bool>({true, false})),
        rclcpp::Parameter("toto", std::vector<uint8_t>({0xff, 0x7f})),
      });
    // Check to see if they were set.
    for (auto & result : set_parameters_results) {
      if (!result.successful) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
      }
    }

    std::stringstream ss;
    // Get a few of the parameters just set.
    for (
      auto & parameter : parameters_client->get_parameters(
        {"foo", "baz", "foobarbaz", "toto"}))
    {
      ss << "\nParameter name: " << parameter.get_name();
      ss << "\nParameter value (" << parameter.get_type_name() << "): " <<
        parameter.value_to_string();
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    rclcpp::shutdown();
  }
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::SetAndGetParameters)
