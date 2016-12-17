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

#include <iostream>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace composition
{

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

Client::Client()
: Node("Client")
{
  client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  timer_ = create_wall_timer(1_s, std::bind(&Client::on_timer, this));
}

bool Client::on_timer()
{
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      fprintf(stderr,
        "add_two_ints_client was interrupted while waiting for the service. Exiting.\n");
      return false;
    }
    fprintf(stderr, "service not available, waiting again...\n");
  }

  // We currently cannot call spin in a spin.
  // That results in the fact that we cannot trigger and wait for
  // a service call within the timer callback.
  // The workaround for this is to give the async request another
  // callback which gets executed once the future is ready.
  // decltype(client_)::element_type::SharedFuture
  // = rclcpp::client::Client<example_interfaces::srv::AddTwoInts>::SharedFuture
  auto return_callback = [](decltype(client_)::element_type::SharedFuture future) {
      printf("Got result: [%s]\n",
        std::to_string(future.get()->sum).c_str());
    };
  auto future_result = client_->async_send_request(request, return_callback);

  return true;
}

}  // namespace composition

#include "class_loader/class_loader_register_macro.h"

CLASS_LOADER_REGISTER_CLASS(composition::Client, rclcpp::Node)
