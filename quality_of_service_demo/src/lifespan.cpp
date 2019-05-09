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
#include <memory>
#include <string>

#include "rcutils/cmdline_parser.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "quality_of_service_demo/common_nodes.hpp"

static const char * OPTION_HISTORY = "--history";
static const uint32_t DEFAULT_HISTORY = 10;
static const char * OPTION_PUBLISH_COUNT = "--publish-count";
static const uint32_t DEFAULT_PUBLISH_COUNT = 5;
static const char * OPTION_SUBSCRIBE_AFTER = "--subscribe-after";
static const uint32_t DEFAULT_SUBSCRIBE_AFTER = 5000;

void print_usage()
{
  printf("Usage for lifespan:\n");
  printf("lifespan lifespan_duration [--history history_depth] [-p publish_n_messages] "
    "[-s subscribe_after_duration] [-h]\n");
  printf("required arguments:\n");
  printf("lifespan duration: Duration (in ms) of the Lifespan QoS setting.\n");
  printf("options:\n");
  printf("-h : Print this help message.\n");
  printf("%s : The depth of the Publisher's history queue - the maximum number of messages "
    "it will store. Defaults to %lu\n", OPTION_HISTORY, (unsigned long)DEFAULT_HISTORY);
  printf("%s publish_n_messages : How many messages to publish before stopping. Defaults to %lu\n",
    OPTION_PUBLISH_COUNT, (unsigned long)DEFAULT_PUBLISH_COUNT);
  printf("%s subscribe_after_duration : The Subscriber will be created this long (in ms) after "
    "application startup. Defaults to %lu\n", OPTION_SUBSCRIBE_AFTER, (unsigned long)DEFAULT_SUBSCRIBE_AFTER);
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Argument parsing
  if (argc < 2 || rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Required arguments
  std::chrono::milliseconds lifespan_duration(std::stoul(argv[1]));

  // Optional argument default values
  size_t history = DEFAULT_HISTORY;
  size_t publish_count = DEFAULT_PUBLISH_COUNT;
  std::chrono::milliseconds subscribe_after_duration(DEFAULT_SUBSCRIBE_AFTER);

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_HISTORY)) {
    history = std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_HISTORY));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_COUNT)) {
    publish_count = std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_PUBLISH_COUNT));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_SUBSCRIBE_AFTER)) {
    subscribe_after_duration = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_SUBSCRIBE_AFTER)));
  }

  // Configuration and Initialization
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::string topic("qos_lifespan_chatter");

  rclcpp::QoS qos_profile(history);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos_profile.lifespan(lifespan_duration);

  rclcpp::SubscriptionOptions sub_options;
  rclcpp::PublisherOptions pub_options;

  auto listener = std::make_shared<Listener>(topic, qos_profile, sub_options, true);
  auto talker = std::make_shared<Talker>(topic, qos_profile, pub_options, publish_count);

  exec.add_node(talker);
  exec.add_node(listener);
  auto timer = listener->create_wall_timer(
    subscribe_after_duration,
    [listener]() -> void {
      listener->start_listening();
    });

  // Execution
  exec.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
