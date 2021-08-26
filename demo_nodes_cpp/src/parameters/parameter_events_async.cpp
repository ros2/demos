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
#include <cstring>
#include <memory>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;
using SetParametersResult =
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
namespace demo_nodes_cpp
{
class ParameterEventsAsyncNode : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ParameterEventsAsyncNode(const rclcpp::NodeOptions & options)
  : Node("parameter_events", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Typically a parameter client is created for a remote node by passing the name of the remote
    // node in the constructor; in this example we create a parameter client for this node itself.
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

    auto on_parameter_event_callback =
      [this](rcl_interfaces::msg::ParameterEvent::UniquePtr event) -> void
      {
        // ignore qos overrides
        event->new_parameters.erase(
          std::remove_if(
            event->new_parameters.begin(),
            event->new_parameters.end(),
            [](const auto & item) {
              const char * param_override_prefix = "qos_overrides.";
              return std::strncmp(
                item.name.c_str(), param_override_prefix, sizeof(param_override_prefix) - 1) == 0u;
            }),
          event->new_parameters.end());
        if (
          !event->new_parameters.size() && !event->changed_parameters.size() &&
          !event->deleted_parameters.size())
        {
          return;
        }
        // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
        std::stringstream ss;
        ss << "\nParameter event:\n new parameters:";
        for (auto & new_parameter : event->new_parameters) {
          ss << "\n  " << new_parameter.name;
        }
        ss << "\n changed parameters:";
        for (auto & changed_parameter : event->changed_parameters) {
          ss << "\n  " << changed_parameter.name;
        }
        ss << "\n deleted parameters:";
        for (auto & deleted_parameter : event->deleted_parameters) {
          ss << "\n  " << deleted_parameter.name;
        }
        ss << "\n";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      };

    // Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);

    // Even though this is in the same node, we still have to wait for the
    // service to be available before declaring parameters (otherwise there is
    // a chance we'll miss some events in the callback).
    while (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "interrupted while waiting for the service. exiting.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Declare parameters that may be set on this node
    this->declare_parameter("foo", 0);
    this->declare_parameter("bar", "");
    this->declare_parameter("baz", 0.);
    this->declare_parameter("foobar", false);

    // Queue a `set_parameters` request as soon as `spin` is called on this node.
    // TODO(dhood): consider adding a "call soon" notion to Node to not require a timer for this.
    timer_ = create_wall_timer(
      200ms,
      [this]() {
        this->queue_first_set_parameter_request();
      });
  }

private:
  // Set several different types of parameters.
  DEMO_NODES_CPP_LOCAL
  void queue_first_set_parameter_request()
  {
    timer_->cancel();  // Prevent another request from being queued by the timer.
    while (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "interrupted while waiting for the service. exiting.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto response_received_callback = [this](SetParametersResult future) {
        // Check to see if they were set.
        for (auto & result : future.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
          }
        }
        this->queue_second_set_parameter_request();
      };

    parameters_client_->set_parameters(
      {
        rclcpp::Parameter("foo", 2),
        rclcpp::Parameter("bar", "hello"),
        rclcpp::Parameter("baz", 1.45),
        rclcpp::Parameter("foobar", true),
      }, response_received_callback);
  }

  // Change the value of some of them.
  DEMO_NODES_CPP_LOCAL
  void queue_second_set_parameter_request()
  {
    auto response_received_callback = [this](SetParametersResult future) {
        // Check to see if they were set.
        for (auto & result : future.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
          }
        }
        // TODO(wjwwood): Create and use delete_parameter

        // Give time for all of the ParameterEvent callbacks to be received.
        timer_ = create_wall_timer(
          100ms,
          []() {
            rclcpp::shutdown();
          });
      };
    parameters_client_->set_parameters(
      {
        rclcpp::Parameter("foo", 3),
        rclcpp::Parameter("bar", "world"),
      }, response_received_callback);
  }

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ParameterEventsAsyncNode)
