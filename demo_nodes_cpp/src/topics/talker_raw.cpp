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
#include "test_msgs/msg/nested.hpp"
#include "test_msgs/msg/dynamic_array_primitives.hpp"

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

  auto node = rclcpp::node::Node::make_shared("talker");

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
  auto chatter_pub = node->create_publisher<test_msgs::msg::DynamicArrayPrimitives>(topic, custom_qos_profile);

  rclcpp::WallRate loop_rate(2);

  rcl_message_raw_t raw_msg;
  raw_msg.buffer_length = 68;
  raw_msg.buffer = new char[raw_msg.buffer_length];
  snprintf(raw_msg.buffer, raw_msg.buffer_length, "%s",
    "00000000000000000000000000000000000000000001000d00000000000000000000");
  auto msg = std::make_shared<test_msgs::msg::Nested>();

  //int i = 1;

  while (rclcpp::ok()) {
    //rcutils_snprintf(raw_msg.buffer, raw_msg.buffer_length, "%c%c%c%c%c%c%c%c%s %d",
    //  0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, "hello world", (++i));
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
