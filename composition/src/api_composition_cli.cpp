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

#include <chrono>
#include <memory>
#include <string>

#include "composition/srv/load_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for api_composition_cli:\n");
  printf("api_composition_cli package_name plugin_name [--delay delay_ms] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("--delay delay_ms: Delay in ms before attempting request. Defaults to 0.\n");
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (argc < 3 || rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "--delay")) {
    std::chrono::milliseconds delay = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, "--delay")));
    std::this_thread::sleep_for(delay);
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("api_composition_cli");
  auto client = node->create_client<composition::srv::LoadNode>("load_node");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto request = std::make_shared<composition::srv::LoadNode::Request>();
  request->package_name = argv[1];
  request->plugin_name = argv[2];

  RCLCPP_INFO(node->get_logger(), "Sending request...");
  auto result = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Waiting for response...");
  while (true) {
    auto ret = rclcpp::spin_until_future_complete(node, result, 1s);
    if (ret == rclcpp::executor::FutureReturnCode::SUCCESS) {
      break;
    }
    if (ret != rclcpp::executor::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.");
      if (!rclcpp::ok()) {
        return 0;
      }
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Response not available, waiting again...");
  }
  RCLCPP_INFO(
    node->get_logger(), "Result of load_node: success = %s",
    result.get()->success ? "true" : "false");

  rclcpp::shutdown();

  return result.get()->success ? 0 : 1;
}
