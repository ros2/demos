// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <cinttypes>
#include <memory>
#include <vector>

#include "rcl/service_introspection.h"

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "demo_nodes_cpp/visibility_control.h"

// This demo program shows how to configure service introspection on the fly,
// by hooking it up to a parameter.  This program consists of a service
// node (IntrospectionServiceNode) that listens on the '/add_two_ints' service
// for clients.  When one connects and sends a request, it adds the two integers
// and returns the result.
//
// The above is a fairly common ROS 2 service, but what this program is trying
// to demonstrate is introspection capabilities.  The IntrospectionServiceNode
// has a string parameter called 'service_configure_introspection'.  If this is
// set to 'disabled' (the default), then no introspection happens.  If this is set
// to 'metadata' (see details on how to set the parameters below), then
// essential metadata (timestamps, sequence numbers, etc) is sent to a hidden
// topic called /add_two_ints/_service_event.
//
// To see this in action, run the following:
//
// ros2 launch demo_nodes_cpp introspect_services_launch.py
//   Since the default for introspection is 'disabled', this is no different than
//   a normal client and server.  No additional topics will be made, and
//   no introspection data will be sent.  However, changing the introspection
//   configuration dynamically is fully supported.  This can be seen by
//   running 'ros2 param set /introspection_service service_configure_introspection metadata'
//   which will configure the service to start sending service introspection
//   metadata to /add_two_ints/_service_event.
//
// Once the parameter is set, introspection data can be seen by running:
//   ros2 topic echo /add_two_ints/_service_event

namespace demo_nodes_cpp
{

class IntrospectionServiceNode : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit IntrospectionServiceNode(const rclcpp::NodeOptions & options)
  : Node("introspection_service", options)
  {
    auto handle_add_two_ints = [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(
          this->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64,
          request->a, request->b);
        response->sum = request->a + request->b;
      };
    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", handle_add_two_ints);

    auto on_set_parameter_callback =
      [](std::vector<rclcpp::Parameter> parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const rclcpp::Parameter & param : parameters) {
          if (param.get_name() != "service_configure_introspection") {
            continue;
          }

          if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            result.successful = false;
            result.reason = "must be a string";
            break;
          }

          if (param.as_string() != "disabled" && param.as_string() != "metadata" &&
            param.as_string() != "contents")
          {
            result.successful = false;
            result.reason = "must be one of 'disabled', 'metadata', or 'contents'";
            break;
          }
        }

        return result;
      };

    auto post_set_parameter_callback =
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        for (const rclcpp::Parameter & param : parameters) {
          if (param.get_name() != "service_configure_introspection") {
            continue;
          }

          rcl_service_introspection_state_t introspection_state = RCL_SERVICE_INTROSPECTION_OFF;

          if (param.as_string() == "disabled") {
            introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
          } else if (param.as_string() == "metadata") {
            introspection_state = RCL_SERVICE_INTROSPECTION_METADATA;
          } else if (param.as_string() == "contents") {
            introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
          }

          this->srv_->configure_introspection(
            this->get_clock(), rclcpp::SystemDefaultsQoS(), introspection_state);
          break;
        }
      };

    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
      on_set_parameter_callback);
    post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(
      post_set_parameter_callback);

    this->declare_parameter("service_configure_introspection", "disabled");
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_parameters_callback_handle_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::IntrospectionServiceNode)
