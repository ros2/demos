// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rcutils/snprintf.h"

#include "std_msgs/msg/string.hpp"

void print_usage()
{
  printf("Usage for talker app:\n");
  printf("talker [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("talker");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  auto topic = std::string("chatter");
  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
  }
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>(topic, custom_qos_profile);

  rclcpp::WallRate loop_rate(2);

  int i = 1;

  rcl_message_raw_t raw_msg;
  raw_msg.buffer_length = 23;
  raw_msg.buffer = new char[raw_msg.buffer_length];
  rcutils_snprintf(raw_msg.buffer, raw_msg.buffer_length, "%c%c%c%c%c%c%c%c%s %d",
    0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, "Hello World:", (++i));
  //snprintf(raw_msg.buffer, raw_msg.buffer_length, "%s",
  //  "000100000f00000048656c6c6f20576f726c643a203100");
  //auto msg = std::make_shared<test_msgs::msg::Nested>();

  for (unsigned int i = 0; i < raw_msg.buffer_length; ++i) {
    fprintf(stderr, "%02x ", raw_msg.buffer[i]);
  }
  fprintf(stderr, "\n");

  while (rclcpp::ok()) {
    printf("Publishing: '%s'\n", raw_msg.buffer);
    chatter_pub->publish(&raw_msg);
    //msg->primitive_values.int32_value = i;
    //printf("Publishing: '%d'\n", i);
    //chatter_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
