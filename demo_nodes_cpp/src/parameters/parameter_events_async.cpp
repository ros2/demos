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
#include <iostream>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using SetParametersResult =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

class ParameterEventsAsyncNode : public rclcpp::Node
{
public:
  ParameterEventsAsyncNode()
  : Node("parameter_events")
  {
    // Typically a parameter client is created for a remote node by passing the name of the remote
    // node in the constructor; in this example we create a parameter client for this node itself.
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

    auto on_parameter_event_callback =
      [](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
      {
        // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
        std::cout << "Parameter event:" << std::endl << " new parameters:" << std::endl;
        for (auto & new_parameter : event->new_parameters) {
          std::cout << "  " << new_parameter.name << std::endl;
        }
        std::cout << " changed parameters:" << std::endl;
        for (auto & changed_parameter : event->changed_parameters) {
          std::cout << "  " << changed_parameter.name << std::endl;
        }
        std::cout << " deleted parameters:" << std::endl;
        for (auto & deleted_parameter : event->deleted_parameters) {
          std::cout << "  " << deleted_parameter.name << std::endl;
        }
      };

    // Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);

    // Queue a `set_parameters` request as soon as `spin` is called on this node.
    // TODO(dhood): consider adding a "call soon" notion to Node to not require a timer for this.
    timer_ = create_wall_timer(0s,
        [this]() {
          this->queue_first_set_parameter_request();
        });
  }

  // Set several different types of parameters.
  void queue_first_set_parameter_request()
  {
    timer_->cancel();  // Prevent another request from being queued by the timer.
    while (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        printf("interrupted while waiting for the service. exiting.\n");
        rclcpp::shutdown();
        return;
      }
      printf("service not available, waiting again...\n");
    }
    auto response_received_callback = [this](SetParametersResult future) {
        // Check to see if they were set.
        for (auto & result : future.get()) {
          if (!result.successful) {
            std::cerr << "failed to set parameter: " << result.reason << std::endl;
          }
        }
        this->queue_second_set_parameter_request();
      };

    parameters_client_->set_parameters({
      rclcpp::parameter::ParameterVariant("foo", 2),
      rclcpp::parameter::ParameterVariant("bar", "hello"),
      rclcpp::parameter::ParameterVariant("baz", 1.45),
      rclcpp::parameter::ParameterVariant("foobar", true),
    }, response_received_callback);
  }

  // Change the value of some of them.
  void queue_second_set_parameter_request()
  {
    auto response_received_callback = [this](SetParametersResult future) {
        // Check to see if they were set.
        for (auto & result : future.get()) {
          if (!result.successful) {
            std::cerr << "failed to set parameter: " << result.reason << std::endl;
          }
        }
        // TODO(wjwwood): Create and use delete_parameter

        // Give time for all of the ParameterEvent callbacks to be received.
        timer_ = create_wall_timer(100ms,
            []() {
              rclcpp::shutdown();
            });
      };
    parameters_client_->set_parameters({
      rclcpp::parameter::ParameterVariant("foo", 3),
      rclcpp::parameter::ParameterVariant("bar", "world"),
    }, response_received_callback);
  }

private:
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ParameterEventsAsyncNode>();
  // TODO(esteve): Make the parameter service automatically start with the node.
  auto parameter_service = std::make_shared<rclcpp::ParameterService>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
