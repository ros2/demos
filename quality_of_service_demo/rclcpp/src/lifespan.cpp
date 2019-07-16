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

#include "rcutils/cmdline_parser.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "quality_of_service_demo/common_nodes.hpp"

static const char * OPTION_HISTORY = "--history";
static const size_t DEFAULT_HISTORY = 10;
static const char * OPTION_PUBLISH_COUNT = "--publish-count";
static const size_t DEFAULT_PUBLISH_COUNT = 5;
static const char * OPTION_SUBSCRIBE_AFTER = "--subscribe-after";
static const size_t DEFAULT_SUBSCRIBE_AFTER = 5000;

void print_usage()
{
  printf("Usage for lifespan:\n");
  printf(
    "lifespan "
    "lifespan_duration "
    "[%s history_depth] "
    "[%s publish_count] "
    "[%s subscribe_after] "
    "[-h]\n",
    OPTION_HISTORY,
    OPTION_PUBLISH_COUNT,
    OPTION_SUBSCRIBE_AFTER);
  printf("required arguments:\n");
  printf(
    "lifespan duration: "
    "Duration in positive integer milliseconds of the Lifespan QoS setting.\n");
  printf("optional arguments:\n");
  printf("-h : Print this help message.\n");
  printf(
    "%s history : "
    "The depth of the Publisher's history queue - "
    "the maximum number of messages it will store for late-joining subscriptions. "
    "Defaults to %zu\n",
    OPTION_HISTORY, DEFAULT_HISTORY);
  printf(
    "%s publish_n_messages : "
    "How many messages to publish before stopping. "
    "Defaults to %zu\n",
    OPTION_PUBLISH_COUNT, DEFAULT_PUBLISH_COUNT);
  printf(
    "%s subscribe_after_duration : "
    "The Subscriber will be created this long in positive integer milliseconds after "
    "application startup. Defaults to %zu\n",
    OPTION_SUBSCRIBE_AFTER, DEFAULT_SUBSCRIBE_AFTER);
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
  qos_profile
  // Guaranteed delivery is needed to send messages to late-joining subscriptions.
  .reliable()
  // Store messages on the publisher so that they can be affected by Lifespan.
  .transient_local()
  .lifespan(lifespan_duration);

  auto talker = std::make_shared<Talker>(qos_profile, topic, publish_count);
  auto listener = std::make_shared<Listener>(qos_profile, topic, true);

  talker->initialize();
  listener->initialize();

  auto timer = listener->create_wall_timer(
    subscribe_after_duration,
    [listener]() -> void {
      listener->start_listening();
    });

  // Execution
  exec.add_node(talker);
  exec.add_node(listener);
  exec.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
