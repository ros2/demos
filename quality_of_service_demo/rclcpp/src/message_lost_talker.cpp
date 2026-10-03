// Copyright 2020 Open Source Robotics Foundation, Inc.
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
#include <utility>
#include <vector>

#include "rcutils/cmdline_parser.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "quality_of_service_demo/visibility_control.h"

using namespace std::chrono_literals;

namespace quality_of_service_demo
{
void print_usage()
{
  std::cout <<
    "Usage: message_lost_talker [-h] [-s SIZE]\n\n"
    "optional arguments:\n"
    "\t-h:                           Print this help message.\n"
    "\t-s <message_size>:            Message size in KiB, default to 8192 KiB" <<
    std::endl;
}

class MessageLostTalker : public rclcpp::Node
{
public:
  QUALITY_OF_SERVICE_DEMO_PUBLIC
  explicit MessageLostTalker(const rclcpp::NodeOptions & options)
  : Node("message_lost_talker", options),
    message_size_(8u * 1024u * 1024u)  // 8MB
  {
    const std::vector<std::string> & args = this->get_node_options().arguments();
    if (args.size()) {
      // Argument usage
      if (args.cend() != std::find(args.cbegin(), args.cend(), "-h")) {
        print_usage();
        // TODO(ivanpauno): Update the rclcpp_components template to be able to handle
        // exceptions. Raise one here, so stack unwinding happens gracefully.
        std::exit(0);
      }
      // Argument: message size
      auto opt_it = std::find(args.cbegin(), args.cend(), "-s");
      if (opt_it != args.cend()) {
        ++opt_it;
        if (opt_it == args.cend()) {
          print_usage();
          std::cout << "\n-s must be followed by a possitive integer" << std::endl;
          // TODO(ivanpauno): Update the rclcpp_components template to be able to handle
          // exceptions. Raise one here, so stack unwinding happens gracefully.
          std::exit(0);
        }
        std::istringstream input_stream(*opt_it);
        input_stream >> message_size_;
        if (!input_stream) {
          print_usage();
          std::cout << "\n-s must be followed by a possitive integer, got: '" <<
            *opt_it << "'" << std::endl;
          // TODO(ivanpauno): Update the rclcpp_components template to be able to handle
          // exceptions. Raise one here, so stack unwinding happens gracefully.
          std::exit(0);
        }
        message_size_ *= 1024uL;
      }
    }
    msg_.data = std::vector<uint8_t>(message_size_, 0u);
    // Timer callback
    auto publish_message =
      [this]() -> void
      {
        rclcpp::Time now = this->get_clock()->now();
        msg_.header.stamp = now;
        RCLCPP_INFO(
          this->get_logger(),
          "Publishing an image, sent at [%f]",
          now.seconds());

        pub_->publish(msg_);
      };
    // Create a publisher
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("message_lost_chatter", 1);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(3s, publish_message);
  }

private:
  size_t message_size_;
  sensor_msgs::msg::Image msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace quality_of_service_demo

RCLCPP_COMPONENTS_REGISTER_NODE(quality_of_service_demo::MessageLostTalker)
