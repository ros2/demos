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
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "test_msgs/srv/nested.hpp"

using namespace std::chrono_literals;

using srv_type = rcl_interfaces::srv::SetParameters;
using srv_type_request = rcl_interfaces::srv::SetParameters_Request;

void
fill_request(std::shared_ptr<srv_type::Request> request)
{
  (void) request;
  rcl_interfaces::msg::Parameter p;
  p.name = "my_parameter_name";
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = 2;
  pv.integer_value = 13;
  p.value = pv;
  request->parameters.push_back(p);
  //test_msgs::msg::Primitives p;
  //p.int32_value = 32;
  //p.float32_value = 32.32f;
  //p.float64_value = 64.64f;
  //int idx = 1;
  //for (auto i = 0; i < idx+1; ++i) {
  //  request->primitives.push_back(p);
  //}
}

void
print_response(const std::shared_ptr<srv_type::Response> response)
{
  (void) response;
  std::cout << "Incoming response" << std::endl;
  std::cout << "Primitive value " << response->results[0].reason << std::endl;
}

void print_usage()
{
  printf("Usage for add_two_ints_client app:\n");
  printf("add_two_ints_client [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-s service_name : Specify the service name for this client. Defaults to add_two_ints.\n");
}

// TODO(wjwwood): make this into a method of rclcpp::Client.
example_interfaces::srv::AddTwoInts_Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client,
  example_interfaces::srv::AddTwoInts_Request::SharedPtr request)
{
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
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

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  auto topic = std::string("add_two_ints");
  if (rcutils_cli_option_exist(argv, argv + argc, "-s")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-s"));
  }
  auto client = node->create_client<srv_type>(topic);

  auto request = std::make_shared<srv_type::Request>();
  fill_request(request);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.")
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...")
  }

  // TODO(wjwwood): make it like `client->send_request(node, request)->sum`
  // TODO(wjwwood): consider error condition
  auto result = send_request(node, client, request);
  if (result) {
    RCLCPP_INFO(node->get_logger(), "Result of add_two_ints: %zd", result->sum)
  } else {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.")
  }

  rclcpp::shutdown();
  return 0;
}
