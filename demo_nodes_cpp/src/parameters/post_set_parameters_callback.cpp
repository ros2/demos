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
 * ros2 service call /post_set_parameters_callback/set_parameters rcl_interfaces/srv/SetParameters
                    "{parameters: [{name: "param1", value: {type: 3, double_value: 1.0}}]}"
 */

#include <string>
#include <memory>

#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp{
class PostSetParameterCallback: public rclcpp::Node{
 public:
  DEMO_NODES_CPP_PUBLIC
  explicit PostSetParameterCallback(const rclcpp::NodeOptions & options)
      : Node("post_set_parameters_callback", options){
    this->declare_parameter<double>("param1", 1.0);
    this->declare_parameter<double>("param2", 2.0);

    // tracks "param2" value
    internal_tracked_class_parameter_ = this->get_parameter("param2").as_double();

    // TODO: remove this after some testing
    auto validationCallback =  [this](std::vector<rclcpp::Parameter> parameters){
      rcl_interfaces::msg::SetParametersResult result;
      for(const auto&param:parameters){
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

    // setting another parameter from the callback is possible
    // we expect the callback to be called again for param2
    auto postSetParameterCallback= [this](std::vector<rclcpp::Parameter> parameters){
      for(const auto&param:parameters){
        if(param.get_name() == "param1"){
          RCLCPP_INFO(this->get_logger(),
                      "param1 set successfully, try changing param2");
          auto result = this->set_parameter(rclcpp::Parameter("param2", 4.0));
          if(result.successful){
            RCLCPP_INFO(this->get_logger(),
                        "param2 changed successfully from postSetParameterCallback");
          }
        }

        // the class member can be changed after successful change in param2 value
        if(param.get_name() == "param2"){
          internal_tracked_class_parameter_ = param.get_value<double>();
        }
      }
    };

    validation_callback_handle_ = this->add_on_set_parameters_callback(validationCallback);
    post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(postSetParameterCallback);
  }

 private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validation_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_parameters_callback_handle_;
  double internal_tracked_class_parameter_;
};

} // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::PostSetParameterCallback)