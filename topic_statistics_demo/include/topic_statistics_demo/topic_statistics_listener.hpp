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

#ifndef TOPIC_STATISTICS_DEMO__TOPIC_STATISTICS_LISTENER_HPP_
#define TOPIC_STATISTICS_DEMO__TOPIC_STATISTICS_LISTENER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "statistics_msgs/msg/metrics_message.hpp"

namespace topic_stats_demo
{
constexpr char STATISTICS_TOPIC_NAME[] = "statistics";
}

class TopicStatisticsListener : public rclcpp::Node
{
public:
  /// Standard Constructor.
  /**
    * \param[in] topic_name Topic to which statistics are published.
    */
  explicit TopicStatisticsListener(
    const std::string & topic_name = topic_stats_demo::STATISTICS_TOPIC_NAME);

  /// Initialize the listener node.
  void initialize();

  /// Return string representation of a MetricsMessage.
  /**
    * \param[in] results Statistics heard form the subscribed topic.
    * \return String representation of the input statistics.
    */
  std::string MetricsMessageToString(const statistics_msgs::msg::MetricsMessage & results);

  /// Instantiate a Subscription to the statistics topic.
  void start_listening();

private:
  rclcpp::SubscriptionOptions subscription_options_;
  rclcpp::Subscription<statistics_msgs::msg::MetricsMessage>::SharedPtr subscription_ = nullptr;

  const std::string topic_name_;
};

#endif  // TOPIC_STATISTICS_DEMO__TOPIC_STATISTICS_LISTENER_HPP_
