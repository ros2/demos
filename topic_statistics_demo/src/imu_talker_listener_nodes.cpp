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
#include <random>
#include <string>

#include "topic_statistics_demo/imu_talker_listener_nodes.hpp"

using namespace std::chrono_literals;

ImuTalker::ImuTalker(
  const std::string & topic_name,
  std::chrono::milliseconds publish_period)
: Node("imu_talker"),
  topic_name_(topic_name),
  publish_period_(publish_period),
  random_generator_(random_number_seed_()),
  random_distribution_(0.0, 1.0) {}

void ImuTalker::initialize()
{
  RCLCPP_INFO(get_logger(), "Talker starting up");

  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    topic_name_,
    10 /* QoS history_depth */);
  publish_timer_ = create_wall_timer(
    publish_period_,
    [this]() -> void {
      publish();
    });
}

void ImuTalker::publish()
{
  sensor_msgs::msg::Imu msg;
  // Timestamp the message to a random time before now,
  // to demonstrate message age metric calculation.

  if (std::floor(random_distribution_(random_generator_) * 2)) {
    RCLCPP_DEBUG(get_logger(), "Adding fixed offset to message timestamp");
    msg.header.stamp =
      this->now() - rclcpp::Duration{0, static_cast<uint32_t>(this->now().nanoseconds() * 0.975)};
  } else {
    msg.header.stamp = this->now();
  }

  RCLCPP_DEBUG(get_logger(), "Publishing header: %u", msg.header.stamp.nanosec);
  publisher_->publish(msg);
}


ImuListener::ImuListener(
  const std::string & topic_name,
  const rclcpp::SubscriptionOptions & subscription_options)
: Node("imu_listener"),
  subscription_options_(subscription_options),
  topic_name_(topic_name) {}

void ImuListener::initialize()
{
  RCLCPP_INFO(get_logger(), "Listener starting up");
  start_listening();
}

void ImuListener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_name_,
      10,  /* QoS history_depth */
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) -> void
      {
        RCLCPP_DEBUG(get_logger(), "Listener heard: %u", msg->header.stamp.nanosec);
      },
      subscription_options_);
  }
}
