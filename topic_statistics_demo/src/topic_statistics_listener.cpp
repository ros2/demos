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

#include <sstream>
#include <string>

#include "topic_statistics_demo/topic_statistics_listener.hpp"

using statistics_msgs::msg::MetricsMessage;
const char * STATISTIC_TYPES[] = {"unknown", "avg", "min", "max", "std_dev", "sample_count"};

TopicStatisticsListener::TopicStatisticsListener(const std::string & topic_name)
: Node("statistics_listener"),
  topic_name_(topic_name) {}

void TopicStatisticsListener::initialize()
{
  RCLCPP_INFO(get_logger(), "TopicStatisticsListener starting up");
  start_listening();
}

void TopicStatisticsListener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<statistics_msgs::msg::MetricsMessage>(
      topic_name_,
      10,  /* QoS history_depth */
      [this](statistics_msgs::msg::MetricsMessage::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(get_logger(), "Statistics heard:\n%s", MetricsMessageToString(*msg).c_str());
      },
      subscription_options_);
  }
}

std::string TopicStatisticsListener::MetricsMessageToString(const MetricsMessage & results)
{
  std::stringstream ss;
  ss << "Metric name: " << results.metrics_source <<
    " source: " << results.measurement_source_name <<
    " unit: " << results.unit;
  ss << "\nWindow start: " << results.window_start.nanosec << " end: " <<
    results.window_stop.nanosec;

  for (const auto & statistic : results.statistics) {
    ss << "\n" <<
      STATISTIC_TYPES[statistic.data_type] <<
      ": " <<
      std::to_string(statistic.data);
  }

  return ss.str();
}
