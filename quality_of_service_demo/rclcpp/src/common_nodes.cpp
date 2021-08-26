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

#include "./utils.hpp"

using namespace std::chrono_literals;

Talker::Talker(
  const rclcpp::QoS & qos_profile,
  const std::string & topic_name,
  size_t publish_count,
  std::chrono::milliseconds publish_period,
  std::chrono::milliseconds assert_topic_period)
: Node("talker"),
  qos_profile_(qos_profile),
  topic_name_(topic_name),
  stop_at_count_(publish_count),
  publish_period_(publish_period),
  assert_topic_period_(assert_topic_period) {}

void
Talker::initialize()
{
  RCLCPP_INFO(get_logger(), "Talker starting up");

  publisher_ = create_publisher<std_msgs::msg::String>(
    topic_name_,
    qos_profile_,
    publisher_options_);
  publish_timer_ = create_wall_timer(
    publish_period_,
    [this]() -> void {
      publish();
    });

  // If enabled, create timer to assert liveliness on the topic
  if (assert_topic_period_ != 0ms) {
    assert_topic_timer_ = create_wall_timer(
      assert_topic_period_,
      [this]() -> bool {
        return assert_publisher_liveliness();
      });
  }
}

void
Talker::publish()
{
  std_msgs::msg::String msg;
  msg.data = "Talker says " + std::to_string(publish_count_);
  RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
  publisher_->publish(msg);

  ++publish_count_;
  if (stop_at_count_ > 0 && publish_count_ >= stop_at_count_) {
    publish_timer_->cancel();
  }
}

size_t
Talker::get_published_count() const
{
  return publish_count_;
}

bool
Talker::assert_publisher_liveliness() const
{
  std::cout << "asserting publisher liveliness" << std::endl;
  return publisher_->assert_liveliness();
}

void
Talker::pause_publish_for(std::chrono::milliseconds pause_length)
{
  if (pause_timer_) {
    // Already paused - ignoring.
    return;
  }
  publish_timer_->cancel();
  pause_timer_ = create_wall_timer(
    pause_length,
    [this]() {
      // Publishing immediately on pause expiration and resuming regular interval.
      publish();
      publish_timer_->reset();
      pause_timer_ = nullptr;
    });
}

void
Talker::toggle_publish()
{
  if (publish_timer_->is_canceled()) {
    std::cout << "start sending messages" << std::endl;
    publish_timer_->cancel();
    publish_timer_->reset();
  } else {
    std::cout << "stop sending messages" << std::endl;
    publish_timer_->cancel();
  }
}

void
Talker::stop_publish_and_assert_liveliness()
{
  publish_timer_->cancel();

  if (assert_topic_timer_) {
    assert_topic_timer_->cancel();
    assert_topic_timer_.reset();
  }
}

void
Talker::print_qos() const
{
  ::print_qos(publisher_->get_actual_qos());
}

Listener::Listener(
  const rclcpp::QoS & qos_profile,
  const std::string & topic_name,
  bool defer_subscribe)
: Node("listener"),
  qos_profile_(qos_profile),
  topic_name_(topic_name),
  defer_subscribe_(defer_subscribe) {}

void
Listener::initialize()
{
  RCLCPP_INFO(get_logger(), "Listener starting up");
  if (!defer_subscribe_) {
    start_listening();
  }
}

void
Listener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<std_msgs::msg::String>(
      topic_name_,
      qos_profile_,
      [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(get_logger(), "Listener heard: [%s]", msg->data.c_str());
      },
      subscription_options_);
  }
}

void
Listener::print_qos() const
{
  ::print_qos(subscription_->get_actual_qos());
}
