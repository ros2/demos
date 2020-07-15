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

#ifndef TOPIC_STATISTICS_DEMO__IMU_TALKER_LISTENER_NODES_HPP_
#define TOPIC_STATISTICS_DEMO__IMU_TALKER_LISTENER_NODES_HPP_

#include <chrono>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace imu_msgs
{
constexpr char DEFAULT_TOPIC_NAME[] = "imu_chatter";
}

class ImuTalker : public rclcpp::Node
{
public:
  /// Standard constructor.
  /**
    * \param[in] topic_name Topic to publish chatter messages to.
    * \param[in] publish_period Frequency of publishing chatter messages.
    **/
  ImuTalker(
    const std::string & topic_name = imu_msgs::DEFAULT_TOPIC_NAME,
    std::chrono::milliseconds publish_period = std::chrono::milliseconds(1000));

  /// Initialize the publisher.
  void initialize();

  /// Publish a single message.
  void publish();

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_ = nullptr;

  const std::string topic_name_;
  std::chrono::milliseconds publish_period_ = std::chrono::milliseconds(1000);
  rclcpp::TimerBase::SharedPtr publish_timer_ = nullptr;

  std::random_device random_number_seed_;
  std::mt19937 random_generator_;
  std::uniform_real_distribution<> random_distribution_;
};

class ImuListener : public rclcpp::Node
{
public:
  /// Standard Constructor.
  /**
    * \param[in] topic_name Topic to subscribe to.
    * \param[in] subscription_options SubscriptionOptions to use for the subscription.
    */
  ImuListener(
    const std::string & topic_name = imu_msgs::DEFAULT_TOPIC_NAME,
    const rclcpp::SubscriptionOptions & subscription_options = rclcpp::SubscriptionOptions());

  /// Initialize the listener node.
  void initialize();

  /// Instantiate a Subscription to the chatter topic.
  void start_listening();

private:
  rclcpp::SubscriptionOptions subscription_options_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_ = nullptr;

  const std::string topic_name_;
};

#endif  // TOPIC_STATISTICS_DEMO__IMU_TALKER_LISTENER_NODES_HPP_
