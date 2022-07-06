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
namespace demo_nodes_cpp {

class SetParametersCallback: public rclcpp::Node {
 public:
  DEMO_NODES_CPP_PUBLIC
  explicit SetParametersCallback(const rclcpp::NodeOptions & options)
      : Node("set_parameters_callback", options) {
    this->declare_parameter<double>("param1", 1.0);
    this->declare_parameter<double>("param2", 2.0);

    // tracks "param1" value
    internal_tracked_class_parameter_1_ = this->get_parameter("param1").as_double();

    // tracks "param2" value
    internal_tracked_class_parameter_2_ = this->get_parameter("param2").as_double();

    // setting another parameter from the callback is possible
    // we expect the callback to be called for param2
    auto preSetParameterCallback =
        [this](std::vector<rclcpp::Parameter> & parameters) {
      for(auto & param : parameters){
        // if "param1" is being set try setting "param2" as well.
        if(param.get_name() == "param1"){
          auto newParam = rclcpp::Parameter("param2", 4.0);
          auto it = std::find(parameters.begin(),
                            parameters.end(), newParam);
          if(it == parameters.end()){
            parameters.push_back(newParam);
          }else{
            *it = newParam;
          }
        }
      }
    };

    // validation callback
    auto onSetParameterCallback =
        [this](std::vector<rclcpp::Parameter> parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      for(const auto & param: parameters) {
        if(param.get_name() == "param1"){
          result.successful = true;
          result.reason = "success param1";
        }
        if(param.get_name() == "param2"){
          result.successful = true;
          result.reason = "success param2";
        }
      }

      return result;
    };

    // can change internally tracked class attributes
    auto postSetParameterCallback=
        [this](const std::vector<rclcpp::Parameter> & parameters) {
      for(const auto & param: parameters){
        if(param.get_name() == "param1"){
          internal_tracked_class_parameter_1_ = param.get_value<double>();
        }

        // the class member can be changed after successful set to param2.
        if(param.get_name() == "param2"){
          internal_tracked_class_parameter_2_ = param.get_value<double>();
        }
      }
    };

    pre_set_parameters_callback_handle_ = this->add_pre_set_parameters_callback(preSetParameterCallback);
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(onSetParameterCallback);
    post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(postSetParameterCallback);
  }

 private:
  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr pre_set_parameters_callback_handle_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_parameters_callback_handle_;

  double internal_tracked_class_parameter_1_;
  double internal_tracked_class_parameter_2_;
};

} // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::SetParametersCallback)