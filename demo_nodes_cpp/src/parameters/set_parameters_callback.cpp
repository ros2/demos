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
 * Example usage: changing 'param1' successfully will result in setting of 'param2'.
 * ros2 service call /set_param_callback_node/set_parameters rcl_interfaces/srv/SetParameters
                    "{parameters: [{name: "param1", value: {type: 3, double_value: 1.0}}]}"
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

/**
 * This example node demonstrates the usage of pre_set, on_set
 * and post_set parameter callbacks
 */
namespace demo_nodes_cpp
{
class SetParametersCallback : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit SetParametersCallback(const rclcpp::NodeOptions & options)
  : Node("set_param_callback_node", options)
  {
    // Declare a parameter named "param1" in this node, with default value 1.0:
    this->declare_parameter("param1", 1.0);
    // Retrieve the value of 'param1' into a member variable 'value_1_'.
    value_1_ = this->get_parameter("param1").as_double();

    // Following statement does the same for 'param2' and 'value_2_', but in a more concise way:
    value_2_ = this->declare_parameter("param2", 2.0);

    // Define a callback function that will be registered as the 'pre_set_parameters_callback':
    //   This callback is passed the list of the Parameter objects that are intended to be changed,
    //   and returns nothing.
    //   Through this callback it is possible to modify the upcoming changes by changing,
    //   adding or removing entries of the Parameter list.
    //
    //   This callback should not change the state of the node (i.e. in this example
    //   the callback should not change 'value_1_' and 'value_2_').
    //
    auto modify_upcoming_parameters_callback =
      [](std::vector<rclcpp::Parameter> & parameters) {
        // As an example: whenever "param1" is changed, "param2" is set to 4.0:
        for (auto & param : parameters) {
          if (param.get_name() == "param1") {
            parameters.push_back(rclcpp::Parameter("param2", 4.0));
          }
        }
      };


    // Define a callback function that will be registered as the 'on_set_parameters_callback':
    //   The purpose of this callback is to allow the node to inspect the upcoming change
    //   to the parameters and explicitly approve or reject the change.
    //   If the change is rejected, no parameters are changed.
    //
    //   This callback should not change the state of the node (i.e. in this example
    //   the callback should not change 'value_1_' and 'value_2_').
    //
    auto validate_upcoming_parameters_callback =
      [](std::vector<rclcpp::Parameter> parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : parameters) {
          // As an example: no parameters are changed if a value > 5.0 is specified for 'param1',
          // or a value < -5.0 for 'param2'.
          if (param.get_name() == "param1") {
            if (param.get_value<double>() > 5.0) {
              result.successful = false;
              result.reason = "cannot set 'param1' > 5.0";
              break;
            }
          } else if (param.get_name() == "param2") {
            if (param.get_value<double>() < -5.0) {
              result.successful = false;
              result.reason = "cannot set 'param2' < -5.0";
              break;
            }
          }
        }

        return result;
      };

    // Define a callback function that will be registered as the 'post_set_parameters_callback':
    //   This callback is passed a list of immutable Parameter objects, and returns nothing.
    //   The purpose of this callback is to react to changes from parameters
    //   that have successfully been accepted.
    //
    //   This callback can change the internal state of the node. E.g.:
    //     - In this example the callback updates the local copies 'value_1_' and 'value_2_',
    //     - Another example could be to trigger recalculation of a kinematic model due to
    //       the change of link length parameters,
    //     - Yet another example could be to emit a signal for an HMI update,
    //     - Etc.
    //
    auto react_to_updated_parameters_callback =
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        for (const auto & param : parameters) {
          if (param.get_name() == "param1") {
            value_1_ = param.get_value<double>();
            RCLCPP_INFO(get_logger(), "Member variable 'value_1_' set to: %f.", value_1_);
          }
          if (param.get_name() == "param2") {
            value_2_ = param.get_value<double>();
            RCLCPP_INFO(get_logger(), "Member variable 'value_2_' set to: %f.", value_2_);
          }
        }
      };


    // Register the callbacks:
    // In this example all three callbacks are registered, but this is not mandatory
    // The handles (i.e. the returned shared pointers) must be kept, as the callback
    // is only registered as long as the shared pointer is alive.
    pre_set_parameters_callback_handle_ = this->add_pre_set_parameters_callback(
      modify_upcoming_parameters_callback);
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
      validate_upcoming_parameters_callback);
    post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(
      react_to_updated_parameters_callback);


    // Output some info:
    RCLCPP_INFO(get_logger(), "This node demonstrates the use of parameter callbacks.");
    RCLCPP_INFO(get_logger(), "As an example, it exhibits the following behavior:");
    RCLCPP_INFO(
      get_logger(),
      " * Two parameters of type double are declared in the node: 'param1' and 'param2'");
    RCLCPP_INFO(get_logger(), " * 'param1' cannot be set to a value > 5.0");
    RCLCPP_INFO(get_logger(), " * 'param2' cannot be set to a value < -5.0");
    RCLCPP_INFO(get_logger(), " * Each time 'param1' is set, 'param2' is automatically set to 4.0");
    RCLCPP_INFO(
      get_logger(),
      " * Member variables 'value_1_' and 'value_2_' are updated upon change of the parameters.");
    RCLCPP_INFO(get_logger(), "To try it out set a parameter, e.g.:");
    RCLCPP_INFO(get_logger(), "  ros2 param set set_param_callback_node param1 10.0");
    RCLCPP_INFO(get_logger(), "  ros2 param set set_param_callback_node param1 3.0");
    RCLCPP_INFO(get_logger(), "The first command will fail.");
    RCLCPP_INFO(get_logger(), "The 2nd command will set 'param1' to 3.0 and 'param2' to 4.0.");
  }

private:
  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
    pre_set_parameters_callback_handle_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_parameters_callback_handle_;

  double value_1_;
  double value_2_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::SetParametersCallback)
