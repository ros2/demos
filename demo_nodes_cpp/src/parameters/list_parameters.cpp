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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{

class ListParameters : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ListParameters(const rclcpp::NodeOptions & options)
  : Node("list_paramters", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    this->declare_parameter("foo", 0);
    this->declare_parameter("bar", "");
    this->declare_parameter("baz", 0.);
    this->declare_parameter("foo.first", 0);
    this->declare_parameter("foo.second", 0);
    this->declare_parameter("foobar", false);

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    RCLCPP_INFO(this->get_logger(), "Setting parameters...");
    // Set several different types of parameters.
    auto set_parameters_results = parameters_client->set_parameters(
      {
        rclcpp::Parameter("foo", 2),
        rclcpp::Parameter("bar", "hello"),
        rclcpp::Parameter("baz", 1.45),
        rclcpp::Parameter("foo.first", 8),
        rclcpp::Parameter("foo.second", 42),
        rclcpp::Parameter("foobar", true),
      });

    RCLCPP_INFO(this->get_logger(), "Listing parameters...");
    // List the details of a few parameters up to a namespace depth of 10.
    auto parameters_and_prefixes = parameters_client->list_parameters({"foo", "bar"}, 10);

    std::stringstream ss;
    ss << "\nParameter names:";
    for (auto & name : parameters_and_prefixes.names) {
      ss << "\n " << name;
    }
    ss << "\nParameter prefixes:";
    for (auto & prefix : parameters_and_prefixes.prefixes) {
      ss << "\n " << prefix;
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    rclcpp::shutdown();
  }
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ListParameters)
