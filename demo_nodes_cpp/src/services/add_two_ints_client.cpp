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

#include <cinttypes>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for add_two_ints_client app:\n");
  printf("add_two_ints_client [-s service_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-s service_name : Specify the service name for this client. Defaults to add_two_ints.\n");
}

class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(const std::string & service_name)
  : Node("add_two_ints_client")
  {
    client_ = create_client<example_interfaces::srv::AddTwoInts>(service_name);
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = -2;
    request->b = -3;

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        printf("add_two_ints_client was interrupted while waiting for the service. Exiting.\n");
        //return 0;
      }
      printf("service not available, waiting again...\n");
    }

    // TODO(wjwwood): make it like `client->send_request(node, request)->sum`
    // TODO(wjwwood): consider error condition
    auto result = send_request(request);
    if (result) {
      printf("Result of add_two_ints: %" PRId64 "\n", result->sum);
    } else {
      printf("add_two_ints_client was interrupted. Exiting.\n");
    }
  }

  // TODO(wjwwood): make this into a method of rclcpp::Client.
  example_interfaces::srv::AddTwoInts_Response::SharedPtr send_request(
    example_interfaces::srv::AddTwoInts_Request::SharedPtr request)
  {
    auto result = client_->async_send_request(request);
    // Wait for the result.
      printf("spinning");
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      return result.get();
      printf("Result of add_two_ints:\n");
    } else {
      printf("no result\n");
      return NULL;
    }
  }

private:
  rclcpp::client::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  rclcpp::init(argc, argv);

  auto service_name = std::string("add_two_ints");
  if (rcutils_cli_option_exist(argv, argv + argc, "-s")) {
    service_name = std::string(rcutils_cli_get_option(argv, argv + argc, "-s"));
  }
  auto node = std::make_shared<ClientNode>(service_name);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
