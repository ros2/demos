// Copyright 2014 Open Source Robotics Foundation, Inc.
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
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

// TODO(wjwwood): make this into a method of rclcpp::Client.
example_interfaces::srv::AddTwoInts::Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client,
  example_interfaces::srv::AddTwoInts::Request::SharedPtr request)
{
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return result.get();
  } else {
    return NULL;
  }
}

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_client");

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  // TODO(wjwwood): make it like `client->send_request(node, request)->sum`
  // TODO(wjwwood): consider error condition
  auto result = send_request(node, client, request);
  if (result) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Result of add_two_ints: " << result->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.");
  }

  rclcpp::shutdown();
  return 0;
}
