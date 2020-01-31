// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{
class EvenParameterNode : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit EvenParameterNode(rclcpp::NodeOptions options)
  : Node("even_parameters_node", options.allow_undeclared_parameters(true))
  {
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Declare a parameter change request callback
    // This function will enforce that only setting even integer parameters is allowed
    // any other change will be discarded
    auto param_change_callback =
      [this](std::vector<rclcpp::Parameter> parameters)
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto parameter : parameters) {
          rclcpp::ParameterType parameter_type = parameter.get_type();
          if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
            RCLCPP_INFO(
              this->get_logger(), "parameter '%s' deleted successfully",
              parameter.get_name().c_str()
            );
            result.successful &= true;
          } else if (rclcpp::ParameterType::PARAMETER_INTEGER == parameter_type) {
            if (parameter.as_int() % 2 != 0) {
              RCLCPP_INFO(
                this->get_logger(), "Requested value '%d' for parameter '%s' "
                "is not an even number: rejecting change...",
                parameter.as_int(),
                parameter.get_name().c_str()
              );
              result.successful = false;
            } else {
              RCLCPP_INFO(
                this->get_logger(),
                "parameter '%s' has changed and is now: %s",
                parameter.get_name().c_str(),
                parameter.value_to_string().c_str()
              );
              result.successful &= true;
            }
          } else {
            RCLCPP_INFO(
              this->get_logger(), "only integer parameters can be set\n"
              "requested value for parameter '%s' is not an even number, rejecting change...",
              parameter.get_name().c_str()
            );
            result.successful = false;
          }
        }
        return result;
      };
    // callback_handler needs to be alive to keep the callback functional
    callback_handler = this->add_on_set_parameters_callback(param_change_callback);
  }

  OnSetParametersCallbackHandle::SharedPtr callback_handler;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::EvenParameterNode)
