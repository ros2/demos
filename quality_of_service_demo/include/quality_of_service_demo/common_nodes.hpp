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

#ifndef QUALITY_OF_SERVICE_DEMO__COMMON_NODES_HPP_
#define QUALITY_OF_SERVICE_DEMO__COMMON_NODES_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static const uint32_t MILLION = 1000L * 1000L;

class Talker : public rclcpp::Node
{
public:
  Talker(
    const std::string & topic_name,
    rclcpp::PublisherOptions pub_options,
    size_t max_count = 0,
    std::chrono::milliseconds assert_node_period = std::chrono::milliseconds(0),
    std::chrono::milliseconds assert_topic_period = std::chrono::milliseconds(0));

  void pause_for(std::chrono::milliseconds pause_length);

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr pause_timer_;
  rclcpp::TimerBase::SharedPtr assert_node_timer_;
  rclcpp::TimerBase::SharedPtr assert_topic_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_ = 0;
  size_t max_count_ = 0;
};

class Listener : public rclcpp::Node
{
public:
  Listener(
    const std::string & topic_name,
    rclcpp::SubscriptionOptions sub_options,
    bool defer_subscribe = false);
  void start_listening();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ = nullptr;
  rclcpp::SubscriptionOptions sub_options_;
  std::string topic_;
};

#endif  // QUALITY_OF_SERVICE_DEMO__COMMON_NODES_HPP_
