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

using namespace std::chrono_literals;

static const char * OPTION_POLICY = "--policy";
static const char * DEFAULT_POLICY = "AUTOMATIC";
static const char * OPTION_TOPIC_ASSERT_PERIOD = "--topic-assert-period";
static const char * OPTION_KILL_PUBLISHER_AFTER = "--kill-publisher-after";
static const size_t DEFAULT_KILL_PUBLISHER_AFTER = 3000;

void print_usage()
{
  printf("Usage for liveliness:\n");
  printf(
    "liveliness "
    "lease_duration "
    "[%s liveliness_policy] "
    "[%s topic_assert_liveliness_period] "
    "[-h]\n",
    OPTION_POLICY,
    OPTION_TOPIC_ASSERT_PERIOD);
  printf("required arguments:\n");
  printf(
    "lease_duration: "
    "Duration in positive integer milliseconds after which an inactive Publisher is considered "
    "not-alive. 0 means never.\n");
  printf("optional arguments:\n");
  printf("-h : Print this help message.\n");
  printf(
    "%s liveliness_policy : "
    "You may specify AUTOMATIC, or MANUAL_BY_TOPIC. "
    "Defaults to %s\n",
    OPTION_POLICY, DEFAULT_POLICY);
  printf(
    "%s topic_assert_period : "
    "How often the Publisher will assert the liveliness of its Publisher "
    ", in positive integer milliseconds. "
    "Defaults to 0 (never)\n",
    OPTION_TOPIC_ASSERT_PERIOD);
  printf(
    "%s kill_publisher_after : "
    "Kill the publisher after this amount of time (in uint milliseconds). "
    "In AUTOMATIC - destroy the whole node. "
    "In MANUAL_BY_TOPIC, stop topic liveliness assertion. "
    "Defaults to %zu\n",
    OPTION_KILL_PUBLISHER_AFTER, DEFAULT_KILL_PUBLISHER_AFTER);
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
  std::chrono::milliseconds liveliness_lease_duration(std::stoul(argv[1]));
  std::chrono::milliseconds topic_assert_period(0);
  std::chrono::milliseconds kill_publisher_after(DEFAULT_KILL_PUBLISHER_AFTER);
  const char * policy_name = DEFAULT_POLICY;
  rmw_qos_liveliness_policy_t liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  std::string topic("qos_liveliness_chatter");

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_TOPIC_ASSERT_PERIOD)) {
    topic_assert_period = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_TOPIC_ASSERT_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_KILL_PUBLISHER_AFTER)) {
    kill_publisher_after = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_KILL_PUBLISHER_AFTER)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_POLICY)) {
    policy_name = rcutils_cli_get_option(argv, argv + argc, OPTION_POLICY);
  }

  if (strcmp(policy_name, "AUTOMATIC") == 0) {
    liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  } else if (strcmp(policy_name, "MANUAL_BY_TOPIC") == 0) {
    liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  } else {
    printf("Unknown liveliness policy: %s\n", policy_name);
    print_usage();
    return 1;
  }

  // Initialization and configuration
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp::QoS qos_profile(10);
  qos_profile
  .liveliness(liveliness_policy_kind)
  .liveliness_lease_duration(liveliness_lease_duration);

  auto talker = std::make_shared<Talker>(
    qos_profile, topic, 0, 500ms, topic_assert_period);

  auto listener = std::make_shared<Listener>(qos_profile, topic);
  listener->get_options().event_callbacks.liveliness_callback =
    [](rclcpp::QOSLivelinessChangedInfo & event)
    {
      printf("Liveliness changed event: \n");
      printf("  alive_count: %d\n", event.alive_count);
      printf("  not_alive_count: %d\n", event.not_alive_count);
      printf("  alive_count_change: %d\n", event.alive_count_change);
      printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
    };

  talker->initialize();
  listener->initialize();

  auto kill_talker_timer = listener->create_wall_timer(
    kill_publisher_after, [&talker, &executor, liveliness_policy_kind]() {
      if (!talker) {
        // *INDENT-OFF* (uncrustify 0.72 erroneously wants to remove this)
        return;
        // *INDENT-ON*
      }
      switch (liveliness_policy_kind) {
        case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
          executor.remove_node(talker);
          talker.reset();
          break;
        case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
          talker->stop_publish_and_assert_liveliness();
          break;
        default:
          break;
      }
    });

  // Execution
  executor.add_node(talker);
  executor.add_node(listener);
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
