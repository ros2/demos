// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "topic_statistics_demo/string_talker_listener_nodes.hpp"

using namespace std::chrono_literals;

StringTalker::StringTalker(
  const std::string & topic_name,
  std::chrono::milliseconds publish_period)
: Node("string_talker"),
  topic_name_(topic_name),
  publish_period_(publish_period) {}

void StringTalker::initialize()
{
  RCLCPP_INFO(get_logger(), "Talker starting up");

  publisher_ = this->create_publisher<std_msgs::msg::String>(
    topic_name_,
    10 /* QoS history_depth */);
  publish_timer_ = create_wall_timer(
    publish_period_,
    [this]() -> void {
      publish();
    });
}

void StringTalker::publish()
{
  std_msgs::msg::String msg;
  msg.data = "Talker says " + std::to_string(publish_count_);
  RCLCPP_DEBUG(get_logger(), "Publishing: '%s'", msg.data.c_str());
  publisher_->publish(msg);

  ++publish_count_;
}


StringListener::StringListener(
  const std::string & topic_name,
  const rclcpp::SubscriptionOptions & subscription_options)
: Node("string_listener"),
  subscription_options_(subscription_options),
  topic_name_(topic_name) {}

void StringListener::initialize()
{
  RCLCPP_INFO(get_logger(), "Listener starting up");
  start_listening();
}

void StringListener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<std_msgs::msg::String>(
      topic_name_,
      10,  /**QoS history_depth */
      [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_DEBUG(get_logger(), "Listener heard: [%s]", msg->data.c_str());
      },
      subscription_options_);
  }
}
