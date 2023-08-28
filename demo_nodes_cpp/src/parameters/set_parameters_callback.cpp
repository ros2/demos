// Copyright 2022 Open Source Robotics Foundation, Inc.
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

/**
 * Example usage: changing param1 successfully will result in setting of param2.
 * ros2 service call /set_parameters_callback/set_parameters rcl_interfaces/srv/SetParameters
                    "{parameters: [{name: "param1", value: {type: 3, double_value: 1.0}}]}"
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

/**
 * node for demonstrating correct usage of pre_set, on_set
 * and post_set parameter callbacks
 */
namespace demo_nodes_cpp
{
class SetParametersCallback : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit SetParametersCallback(const rclcpp::NodeOptions & options)
  : Node("set_parameters_callback", options)
  {
    // tracks "param1" value
    internal_tracked_class_parameter_1_ = this->declare_parameter("param1", 0.0);
    // tracks "param2" value
    internal_tracked_class_parameter_2_ = this->declare_parameter("param2", 0.0);

    // setting another parameter from the callback is possible
    // we expect the callback to be called for param2
    auto pre_set_parameter_callback =
      [](std::vector<rclcpp::Parameter> & parameters) {
        for (auto & param : parameters) {
          // if "param1" is being set try setting "param2" as well.
          if (param.get_name() == "param1") {
            parameters.push_back(rclcpp::Parameter("param2", 4.0));
          }
        }
      };

    // validation callback
    auto on_set_parameter_callback =
      [](std::vector<rclcpp::Parameter> parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : parameters) {
          if (param.get_name() == "param1") {
            // Arbitrarily reject updates setting param1 > 5.0
            if (param.get_value<double>() > 5.0) {
              result.successful = false;
              result.reason = "cannot set param1 > 5.0";
              break;
            }
          } else if (param.get_name() == "param2") {
            // Arbitrarily reject updates setting param2 < -5.0
            if (param.get_value<double>() < -5.0) {
              result.successful = false;
              result.reason = "cannot set param2 < -5.0";
              break;
            }
          }
        }

        return result;
      };

    // can change internally tracked class attributes
    auto post_set_parameter_callback =
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        for (const auto & param : parameters) {
          if (param.get_name() == "param1") {
            internal_tracked_class_parameter_1_ = param.get_value<double>();
          }

          // the class member can be changed after successful set to param2.
          if (param.get_name() == "param2") {
            internal_tracked_class_parameter_2_ = param.get_value<double>();
          }
        }
      };

    pre_set_parameters_callback_handle_ = this->add_pre_set_parameters_callback(
      pre_set_parameter_callback);
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
      on_set_parameter_callback);
    post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(
      post_set_parameter_callback);

    RCLCPP_INFO(get_logger(), "This node shows off parameter callbacks.");
    RCLCPP_INFO(get_logger(), "To do that, it exhibits the following behavior:");
    RCLCPP_INFO(
      get_logger(),
      " * Two parameters of type double are declared on the node, param1 and param2");
    RCLCPP_INFO(get_logger(), " * param1 cannot be set to a value > 5.0");
    RCLCPP_INFO(get_logger(), " * param2 cannot be set to a value < -5.0");
    RCLCPP_INFO(get_logger(), " * any time param1 is set, param2 is automatically set to 4.0");
  }

private:
  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
    pre_set_parameters_callback_handle_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_parameters_callback_handle_;

  double internal_tracked_class_parameter_1_;
  double internal_tracked_class_parameter_2_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::SetParametersCallback)
