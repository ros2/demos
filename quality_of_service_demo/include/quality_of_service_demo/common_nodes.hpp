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
  /**
    * \param[in] topic_name Topic to publish to.
    * \param[in] qos_profile QoS profile for Publisher.
    * \param[in] publisher_options Additional options for Publisher.
    * \param[in] publish_count (Optional) Number of messages to publish before stopping.
    *   0 (default) means publish forever.
    * \param[in] assert_node_period (Optional) How often to manually assert Node liveliness.
    *   0 (default) means never.
    * \param[in] assert_topic_period (Optional) How often to manually assert Publisher liveliness.
    *   0 (default) means never.
    **/
  Talker(
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile,
    const rclcpp::PublisherOptions & publisher_options,
    size_t publish_count = 0,
    std::chrono::milliseconds assert_node_period = std::chrono::milliseconds(0),
    std::chrono::milliseconds assert_topic_period = std::chrono::milliseconds(0));

  /// Stop publishing for a while.
  /**
    * Stops the publisher for the specified amount of time.
    * A message will be published immediately on the expiration of pause_duration.
    * The regular publishing interval will resume at that point.
    * If publishing is already paused, this call will be ignored.
    * The remaining pause duration will not be affected.
    * \param[in] pause_duration Amount of time to pause for.
    **/
  void pause_for(std::chrono::milliseconds pause_duration);

  /// Publish a single message.
  /**
    * Counts towards the total message count that will be published.
    */
  void publish();

  /// Cancel publishing, and any manual liveliness assertions this node was configured to do.
  void stop();

private:
  rclcpp::TimerBase::SharedPtr assert_node_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr assert_topic_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr publish_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr pause_timer_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ = nullptr;
  size_t publish_count_ = 0;
  const size_t stop_at_count_ = 0;
};

class Listener : public rclcpp::Node
{
public:
  /// Standard Constructor.
  /**
    * \param[in] topic_name Topic to subscribe to.
    * \param[in] qos_profile QoS profile for Subscription.
    * \param[in] sub_options Additional options for Subscription.
    * \param[in] defer_subscribe (Optional) don't create Subscription until user calls
    *   start_listening().
    */
  Listener(
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile,
    const rclcpp::SubscriptionOptions & subscription_options,
    bool defer_subscribe = false);

  /// Instantiates Subscription.
  /**
    * Does nothing if it has already been called.
    */
  void start_listening();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ = nullptr;
  rclcpp::QoS qos_profile_;
  rclcpp::SubscriptionOptions subscription_options_;
  const std::string topic_name_;
};

#endif  // QUALITY_OF_SERVICE_DEMO__COMMON_NODES_HPP_
