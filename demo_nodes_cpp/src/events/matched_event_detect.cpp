// Copyright 2023 Sony Group Corporation.
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

#include "rcl/event.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{

// This demo program shows detected matched event.
// Matched event occur while connection between publisher and subscription is done.
// Run this demo at one console by below command
// $ ros2 run demo_nodes_cpp matched_event_detect
// It will create 2 topics.
// - 'pub_matched_event_detect' for detecting matched event of publisher
// - 'sub_matched_event_detect' for detecting matched event of subscription
//
// For checking matched event of publisher
// On another console, run below command
// $ ros2 topic echo /pub_matched_event_detect
// You can run above command many times on different consoles to check the number of connected
// subscription by the output of demo program.
//
// For checking matched event of subscription
// On another console, run below command
// ros2 topic pub -r 1 /sub_matched_event_detect std_msgs/String "{data: '123'}"
// You can run above command many times on different consoles to check the number of connected
// publisher by the output of demo program.

class MatchedEventDetectNode : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC explicit MatchedEventDetectNode(const rclcpp::NodeOptions & options)
  : Node("matched_event_detection_node", options)
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.event_callbacks.matched_callback =
      [this](rmw_matched_status_t & s) {
        if (connect_subscription_) {
          if (s.current_count == 0) {
            RCLCPP_INFO(this->get_logger(), "Last subscription is disconnected.");
            connect_subscription_ = false;
          } else {
            RCLCPP_INFO(
              this->get_logger(),
              "Current number of connected subscription is %lu", s.current_count);
          }
        } else {
          if (s.current_count != 0) {
            RCLCPP_INFO(this->get_logger(), "First subscription is connected.");
            connect_subscription_ = true;
          }
        }
      };

    pub_ = create_publisher<std_msgs::msg::String>(
      "pub_matched_event_detect", 10, pub_options);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.event_callbacks.matched_callback =
      [this](rmw_matched_status_t & s) {
        if (connect_publisher_) {
          if (s.current_count == 0) {
            RCLCPP_INFO(this->get_logger(), "Last publisher is disconnected.");
            connect_publisher_ = false;
          } else {
            RCLCPP_INFO(
              this->get_logger(),
              "Current number of connected publisher is %lu", s.current_count);
          }
        } else {
          if (s.current_count != 0) {
            RCLCPP_INFO(this->get_logger(), "First publisher is connected.");
            connect_publisher_ = true;
          }
        }
      };
    sub_ = create_subscription<std_msgs::msg::String>(
      "sub_matched_event_detect",
      10,
      [](std_msgs::msg::String::ConstSharedPtr) {},
      sub_options);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  bool connect_subscription_{false};
  bool connect_publisher_{false};
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::MatchedEventDetectNode)
