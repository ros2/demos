// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "composition/client_component.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace composition
{

Client::Client(const rclcpp::NodeOptions & options)
: Node("Client", options)
{
  client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  // Note(dhood): The timer period must be greater than the duration of the timer callback.
  // Otherwise, the timer can starve a single-threaded executor.
  // See https://github.com/ros2/rclcpp/issues/392 for updates.
  timer_ = create_wall_timer(2s, [this]() {return this->on_timer();});
}

void Client::on_timer()
{
  if (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
    return;
  }

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  // In order to wait for a response to arrive, we need to spin().
  // However, this function is already being called from within another spin().
  // Unfortunately, the current version of spin() is not recursive and so we
  // cannot call spin() from within another spin().
  // Therefore, we cannot wait for a response to the request we just made here
  // within this callback, because it was executed by some other spin function.
  // The workaround for this is to give the async_send_request() method another
  // argument which is a callback that gets executed once the future is ready.
  // We then return from this callback so that the existing spin() function can
  // continue and our callback will get called once the response is received.
  using ServiceResponseFuture =
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      RCLCPP_INFO(this->get_logger(), "Got result: [%" PRId64 "]", future.get()->sum);
    };
  auto future_result = client_->async_send_request(request, response_received_callback);
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Client)
