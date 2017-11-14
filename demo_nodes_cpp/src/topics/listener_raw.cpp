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

#include <iostream>
#include <memory>
#include <string>

#include "rcl/types.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"
#include "test_msgs/msg/nested.hpp"

void print_usage()
{
  printf("Usage for listener app:\n");
  printf("listener [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to subscribe. Defaults to chatter.\n");
}

void chatterCallback_raw(const std::shared_ptr<rcl_message_raw_t> msg)
{
  std::cout << "I heard: [" << msg->buffer_length << "]" << std::endl;
  for (size_t i = 0; i < msg->buffer_length; ++i) {
    fprintf(stderr, "%02x ", msg->buffer[i]);
  }
  fprintf(stderr, "\n");
}

void chatterCallback(const std::shared_ptr<test_msgs::msg::Nested> msg)
{
  std::cout << "I heard: [" << msg->primitive_values.int32_value << "]" << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("listener");

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  auto topic = std::string("chatter");
  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
  }
  auto sub = node->create_subscription<test_msgs::msg::Nested>(
    topic, chatterCallback_raw);

  rclcpp::spin(node);

  return 0;
}
