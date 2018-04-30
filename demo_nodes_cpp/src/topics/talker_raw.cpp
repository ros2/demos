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

#include "rmw/raw_message.h"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for talker app:\n");
  printf("talker [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
}

class RawTalker : public rclcpp::Node
{
public:
  explicit RawTalker(const std::string & topic_name)
  : Node("raw_talker")
  {
    raw_msg_ = rmw_get_zero_initialized_raw_message();
    auto allocator = rcutils_get_default_allocator();
    rmw_initialize_raw_message(
      &raw_msg_,
      0u,
      &allocator);

    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        auto string_msg = std::make_shared<std_msgs::msg::String>();
        string_msg->data = "Hello World:" + std::to_string(count_++);
        auto string_ts =
          rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::String>();
        rmw_raw_message_resize(&raw_msg_, 8u + string_msg->data.size());
        auto ret = rmw_serialize(string_msg.get(), string_ts, &raw_msg_);
        if (ret != RMW_RET_OK) {
          fprintf(stderr, "failed to deserialize raw message\n");
        }

        // This is the manual CDR serialization of a string message with the content of
        // Hello World: <count_> equivalent to talker example.
        // rcutils_snprintf(raw_msg_.buffer, raw_msg_.buffer_length, "%c%c%c%c%c%c%c%c%s %zu",
        //   0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, "Hello World:", count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s: %zu'", "Hello World", count_)

        printf("ROS message:\n");
        printf("%s\n", string_msg->data.c_str());
        printf("Raw message:\n");
        for (unsigned int i = 0; i < raw_msg_.buffer_length; ++i) {
          printf("%02x ", raw_msg_.buffer[i]);
        }
        printf("\n");

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(&raw_msg_);
      };

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, custom_qos_profile);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

  ~RawTalker()
  {
    rmw_raw_message_fini(&raw_msg_);
  }

private:
  size_t count_ = 1;
  rcl_message_raw_t raw_msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("chatter");
  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
  }

  // Create a node.
  auto node = std::make_shared<RawTalker>(topic);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
