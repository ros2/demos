// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <string>

#include "quality_of_service_demo/common_nodes.hpp"

using namespace std::chrono_literals;

Talker::Talker(
  const std::string & topic_name,
  rclcpp::PublisherOptions<> pub_options,
  size_t max_count,
  std::chrono::milliseconds assert_node_period,
  std::chrono::milliseconds assert_topic_period)
: Node("talker"),
  max_count_(max_count)
{
  RCLCPP_INFO(get_logger(), "Talker starting up");
  publisher_ = create_publisher<std_msgs::msg::String>(topic_name, nullptr, pub_options);
  publish_timer_ = create_wall_timer(
    500ms,
    [this]() -> void {
      std_msgs::msg::String msg;
      msg.data = "Talker says " + std::to_string(count_++);
      publisher_->publish(msg);
      if (max_count_ > 0 && count_ > max_count_) {
        publish_timer_->cancel();
      }
    });
  // If enabled, create timer to assert liveliness at the node level
  if (assert_node_period != 0ms) {
    assert_node_timer_ = create_wall_timer(
      assert_node_period,
      [this]() -> void {
        this->assert_liveliness();
      });
  }
  // If enabled, create timer to assert liveliness on the topic
  if (assert_topic_period != 0ms) {
    assert_topic_timer_ = create_wall_timer(
      assert_topic_period,
      [this]() -> void {
        publisher_->assert_liveliness();
      });
  }
}

void Talker::pause_for(std::chrono::milliseconds pause_length)
{
  if (pause_timer_) {
    // Already paused - just ignore
    return;
  }
  publish_timer_->cancel();
  pause_timer_ = create_wall_timer(
    pause_length,
    [this]() {
      publish_timer_->reset();
      pause_timer_ = nullptr;
    });
}


Listener::Listener(
  const std::string & topic_name,
  rclcpp::SubscriptionOptions<> sub_options,
  bool defer_subscribe)
: Node("listener"),
  sub_options_(sub_options),
  topic_(topic_name)
{
  RCLCPP_INFO(get_logger(), "Listener starting up");
  if (!defer_subscribe) {
    start_listening();
  }
}

void Listener::start_listening()
{
  subscription_ = create_subscription<std_msgs::msg::String>(
    topic_,
    [this](const typename std_msgs::msg::String::SharedPtr msg) -> void
    {
      RCLCPP_INFO(get_logger(), "Listener heard: [%s]", msg->data.c_str());
    },
    nullptr,
    sub_options_);
}
