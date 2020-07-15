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
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "topic_statistics_demo/imu_talker_listener_nodes.hpp"
#include "topic_statistics_demo/string_talker_listener_nodes.hpp"
#include "topic_statistics_demo/topic_statistics_listener.hpp"

using namespace std::chrono_literals;

static const char * OPTION_PUBLISH_TOPIC = "--publish-topic";
static const char * DEFAULT_PUBLISH_TOPIC = "/statistics";
static const char * OPTION_PUBLISH_PERIOD = "--publish-period";
static const size_t DEFAULT_PUBLISH_PERIOD = 5000;

void print_usage()
{
  printf("\nUsage for display_topic_statistics:\n");
  printf(
    "display_topic_statistics "
    " message_type "
    "[%s statistics publish topic] "
    "[%s statistics publish period] "
    "[-h]\n",
    OPTION_PUBLISH_TOPIC,
    OPTION_PUBLISH_PERIOD);
  printf("\nrequired arguments:\n");
  printf(
    "message_type: "
    "Type of topic to display statistics for. Can be one of {string, imu}.\n");
  printf("\noptional arguments:\n");
  printf("-h : Print this help message.\n");
  printf(
    "%s : "
    "Topic to which topic statistics get published. "
    "Defaults to %s.\n",
    OPTION_PUBLISH_TOPIC, DEFAULT_PUBLISH_TOPIC);
  printf(
    "%s : "
    "Publish period (in positive integer milliseconds) for publication of statistics. "
    "Defaults to %zu (%zus).\n",
    OPTION_PUBLISH_PERIOD, DEFAULT_PUBLISH_PERIOD, DEFAULT_PUBLISH_PERIOD / 1000);
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Argument count and usage
  if (argc < 2 || rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Configuration variables
  std::string topic_type(argv[1]);
  std::string publish_topic(DEFAULT_PUBLISH_TOPIC);
  std::chrono::milliseconds publish_period(DEFAULT_PUBLISH_PERIOD);
  std::string test_topic("topic_statistics_chatter");

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_TOPIC)) {
    publish_topic = rcutils_cli_get_option(argv, argv + argc, OPTION_PUBLISH_TOPIC);
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_PERIOD)) {
    publish_period = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_PUBLISH_PERIOD)));
  }

  // Initialization and configuration
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  // Enable topic statistics publication through subscriptions
  auto options = rclcpp::SubscriptionOptions();
  options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  options.topic_stats_options.publish_topic = publish_topic;
  options.topic_stats_options.publish_period = publish_period;

  if (topic_type == "imu") {
    // Start the talker and listener nodes to pass IMU messages
    auto talker = std::make_shared<ImuTalker>(test_topic);
    talker->initialize();
    auto listener = std::make_shared<ImuListener>(test_topic, options);
    listener->initialize();

    // Start the listener node to listen to statistics
    auto statistics_listener = std::make_shared<TopicStatisticsListener>(publish_topic);
    statistics_listener->initialize();

    // Execution
    executor.add_node(talker);
    executor.add_node(listener);
    executor.add_node(statistics_listener);
    executor.spin();
  } else if (topic_type == "string") {
    // Start the talker and listener nodes to pass IMU messages
    auto talker = std::make_shared<StringTalker>(test_topic);
    talker->initialize();
    auto listener = std::make_shared<StringListener>(test_topic, options);
    listener->initialize();

    // Start the listener node to listen to statistics
    auto statistics_listener = std::make_shared<TopicStatisticsListener>(publish_topic);
    statistics_listener->initialize();

    // Execution
    executor.add_node(talker);
    executor.add_node(listener);
    executor.add_node(statistics_listener);
    executor.spin();
  } else {
    printf("Invalid message type provided: %s", argv[1]);
    print_usage();
    return 0;
  }

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
