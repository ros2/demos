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

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  if (argc != 3) {
    fprintf(stderr, "Requires exactly two arguments to be passed: package name and plugin name\n");
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("api_composition_cli");
  auto client = node->create_client<composition::srv::LoadNode>("load_node");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      printf("api_composition_cli was interrupted while waiting for the service. Exiting.\n");
      return 0;
    }
    printf("service not available, waiting again...\n");
  }

  auto request = std::make_shared<composition::srv::LoadNode::Request>();
  request->package_name = argv[1];
  request->plugin_name = argv[2];

  printf("Sending request...\n");
  auto result = client->async_send_request(request);
  printf("Waiting for response...\n");
  if (rclcpp::spin_until_future_complete(node, result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    fprintf(stderr, "api_composition_cli was interrupted. Exiting.\n");
    if (!rclcpp::ok()) {
      return 0;
    }
    return 1;
  }
  printf("Result of load_node: success = %s\n", result.get()->success ? "true" : "false");

  rclcpp::shutdown();

  return result.get()->success ? 0 : 1;
}
