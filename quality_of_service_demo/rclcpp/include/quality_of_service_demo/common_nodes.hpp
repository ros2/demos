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

constexpr char DEFAULT_TOPIC_NAME[] = "qos_chatter";

/// Node class to publish data at a specified rate, optionally pausing to test liveliness
class Talker final : public rclcpp::Node
{
public:
  /// Standard Constructor
  /**
    * \param[in] qos_profile QoS profile for Publisher.
    * \param[in] topic_name Topic to publish to.
    * \param[in] publish_count (Optional) Number of messages to publish before stopping.
    *   0 (default) means publish forever.
    * \param[in] publish_period (Optional) How often to publish
    * \param[in] assert_topic_period (Optional) How often to manually assert Publisher liveliness.
    *   0 (default) means never.
    **/
  explicit Talker(
    const rclcpp::QoS & qos_profile,
    const std::string & topic_name = DEFAULT_TOPIC_NAME,
    size_t publish_count = 0,
    std::chrono::milliseconds publish_period = std::chrono::milliseconds(500),
    std::chrono::milliseconds assert_topic_period = std::chrono::milliseconds(0));

  /// Initialize the publisher.
  void initialize();

  /// Publish a single message.
  /**
    * Counts towards the total message count that will be published.
    */
  void publish();

  /// Get the number of messages that have been published by this publisher.
  size_t get_published_count() const;

  /// Assert the liveliness of the publisher.
  bool assert_publisher_liveliness() const;

  /// Start/Stop publishing for a while.
  void toggle_publish();

  /// Stop publishing for a while.
  /**
    * Stops the publisher for the specified amount of time.
    * A message will be published immediately on the expiration of pause_duration.
    * The regular publishing interval will resume at that point.
    * If publishing is already paused, this call will be ignored.
    * The remaining pause duration will not be affected.
    * \param[in] pause_duration Amount of time to pause for.
    **/
  void pause_publish_for(std::chrono::milliseconds pause_duration);

  /// Cancel publishing, and any manual liveliness assertions this node was configured to do.
  void stop_publish_and_assert_liveliness();

  /// Get the publisher's settings options.
  rclcpp::PublisherOptions & get_options() {return publisher_options_;}

  /// Print the QoS settings of the publisher.
  void print_qos() const;

private:
  rclcpp::QoS qos_profile_;
  rclcpp::PublisherOptions publisher_options_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ = nullptr;

  const std::string topic_name_;
  size_t publish_count_ = 0;
  const size_t stop_at_count_ = 0;

  std::chrono::milliseconds publish_period_ = std::chrono::milliseconds(500);
  std::chrono::milliseconds assert_topic_period_ = std::chrono::milliseconds(0);
  rclcpp::TimerBase::SharedPtr publish_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr pause_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr assert_topic_timer_ = nullptr;
};

/// Node class to listen to listen to incoming data from the Talker
class Listener final : public rclcpp::Node
{
public:
  /// Standard Constructor.
  /**
    * \param[in] qos_profile QoS profile for Subscription.
    * \param[in] topic_name Topic to subscribe to.
    * \param[in] defer_subscribe (Optional) don't create Subscription until user calls
    *   start_listening().
    */
  explicit Listener(
    const rclcpp::QoS & qos_profile,
    const std::string & topic_name = DEFAULT_TOPIC_NAME,
    bool defer_subscribe = false);

  /// Initialize the publisher.
  void initialize();

  /// Instantiates Subscription.
  /**
    * Does nothing if it has already been called.
    */
  void start_listening();

  /// Get the subscription's settings options.
  rclcpp::SubscriptionOptions & get_options() {return subscription_options_;}

  /// Print the QoS settings of the subscriber.
  void print_qos() const;

private:
  rclcpp::QoS qos_profile_;
  rclcpp::SubscriptionOptions subscription_options_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ = nullptr;

  const std::string topic_name_;
  const bool defer_subscribe_ = false;
};

#endif  // QUALITY_OF_SERVICE_DEMO__COMMON_NODES_HPP_
