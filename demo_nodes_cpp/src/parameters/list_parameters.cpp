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

#include <memory>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("list_parameters");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.")
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...")
  }

  RCLCPP_INFO(node->get_logger(), "Setting parameters...")
  // Set several differnet types of parameters.
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::Parameter("foo", 2),
    rclcpp::Parameter("bar", "hello"),
    rclcpp::Parameter("baz", 1.45),
    rclcpp::Parameter("foo.first", 8),
    rclcpp::Parameter("foo.second", 42),
    rclcpp::Parameter("foobar", true),
  });

  RCLCPP_INFO(node->get_logger(), "Listing parameters...")
  // List the details of a few parameters up to a namespace depth of 10.
  auto parameters_and_prefixes = parameters_client->list_parameters({"foo", "bar"}, 10);

  std::stringstream ss;
  ss << "\nParameter names:";
  for (auto & name : parameters_and_prefixes.names) {
    ss << "\n " << name;
  }
  ss << "\nParameter prefixes:";
  for (auto & prefix : parameters_and_prefixes.prefixes) {
    ss << "\n " << prefix;
  }
  RCLCPP_INFO(node->get_logger(), ss.str().c_str())

  rclcpp::shutdown();

  return 0;
}
