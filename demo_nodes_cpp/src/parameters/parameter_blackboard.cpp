// Copyright 2019 Open Source Robotics Foundation, Inc.
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
#include <string>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{

class ParameterBlackboard : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ParameterBlackboard(
    rclcpp::NodeOptions options
  )
  : Node(
      "parameter_blackboard",
      options.allow_undeclared_parameters(true).
      automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Parameter blackboard node named '%s' ready, and serving '%zu' parameters already!",
      this->get_fully_qualified_name(), this->list_parameters(
        {}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE).names.size());
  }
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ParameterBlackboard)
