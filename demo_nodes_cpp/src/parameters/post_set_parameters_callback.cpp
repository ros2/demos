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

    validation_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PostSetParameterCallback::validationCallback, this, std::placeholders::_1));
    post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(
        std::bind(&PostSetParameterCallback::postSetParameterCallback, this, std::placeholders::_1));
  }

 private:
  rcl_interfaces::msg::SetParametersResult validationCallback(
      const std::vector<rclcpp::Parameter> &parameters){
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
  }

  void postSetParameterCallback(const std::vector<rclcpp::Parameter> &parameters){
    rcl_interfaces::msg::SetParametersResult result;
    for(const auto&param:parameters){
      if(param.get_name() == "param1"){
        this->set_parameter(rclcpp::Parameter("param2", 4.0));
      }
      if(param.get_name() == "param2"){
      }
    }
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validation_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_parameters_callback_handle_;
};

} // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::PostSetParameterCallback)