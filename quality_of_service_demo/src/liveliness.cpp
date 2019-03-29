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

static const size_t default_kill_publisher_after_ms = 3000;

void print_usage()
{
  printf("Usage for liveliness:\n");
  printf("liveliness lease_duration [-p liveliness_policy] [-n node_assert_liveliness_period] "
    "[-t topic_assert_liveliness_period] [-h]\n");
  printf("required arguments:\n");
  printf("lease_duration: Duration (in ms) before an inactive Publisher is considered not-alive. "
    "0 means infinity.\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-p liveliness_policy : You may specify AUTOMATIC, MANUAL_BY_NODE, or MANUAL_BY_TOPIC. "
    "Defaults to AUTOMATIC\n");
  printf("-n node_assert_period : How often the Publisher will assert the liveliness of its "
    "Node (in ms). Defaults to 0 (never)\n");
  printf("-t topic_assert_period : How often the Publisher will assert the liveliness of its "
    "Publisher (in ms). Defaults to 0 (never)\n");
  printf("-k kill_publisher_after : Kill the publisher after this amount of time (in ms). In "
    "AUTOMATIC - destroy the whole node. In MANUAL_BY_NODE, stop node liveliness assertion. In "
    "MANUAL_BY_TOPIC, stop topic liveliness assertion. Defaults to %zu\n",
    default_kill_publisher_after_ms);
}

int main(int argc, char * argv[])
{
  // Argument parsing
  if (argc < 2 || rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }
  // Required arguments
  size_t liveliness_lease_duration_ms = std::stoul(argv[1]);

  // Optional argument default values
  std::chrono::milliseconds node_assert_period(0);
  std::chrono::milliseconds topic_assert_period(0);
  std::chrono::milliseconds kill_publisher_after(default_kill_publisher_after_ms);
  // TODO(emersonknapp) once new types are available
  // rmw_qos_liveliness_policy_t liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, "-n")) {
    node_assert_period = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, "-n")));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    topic_assert_period = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, "-t")));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "-k")) {
    kill_publisher_after = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, "-k")));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "-p")) {
    char * policy_str = rcutils_cli_get_option(argv, argv + argc, "-p");
    /*
    TODO(emersonknapp) once new types are available
    if (strcmp(policy_str, "AUTOMATIC") == 0) {
      liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    } else if (strcmp(policy_str, "MANUAL_BY_NODE") == 0) {
      liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE;
    } else if (strcmp(policy_str, "MANUAL_BY_TOPIC") == 0) {
      liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    } else {
    */
      printf("Unknown liveliness policy: %s\n", policy_str);
      print_usage();
      return 1;
    // }
  }

  // Configuration and Initialization
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::string topic("qos_liveliness_chatter");

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  /*
  TODO(emersonknapp) once new types are available
  qos_profile.liveliness = liveliness_policy_kind;
  qos_profile.liveliness_lease_duration.sec = liveliness_lease_duration_ms / 1000;
  qos_profile.liveliness_lease_duration.nsec = (liveliness_lease_duration_ms % 1000) * 1000000;
  */

  rclcpp::SubscriptionOptions<> sub_options;
  sub_options.qos_profile = qos_profile;
  /*
  TODO(emersonknapp) once callbacks are available
  sub_options.event_callbacks.liveliness_callback =
    [](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      printf("Liveliness changed event: \n");
      printf("  alive_count: %d\n", event.alive_count);
      printf("  not_alive_count: %d\n", event.not_alive_count);
      printf("  alive_count_change: %d\n", event.alive_count_change);
      printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
    });
  */

  rclcpp::PublisherOptions<> pub_options;
  pub_options.qos_profile = qos_profile;

  auto listener = std::make_shared<Listener>(topic, sub_options);
  auto talker = std::make_shared<Talker>(
    topic,
    pub_options,
    0,
    node_assert_period,
    topic_assert_period);

  // Execution
  exec.add_node(listener);
  exec.add_node(talker);

  auto timer = listener->create_wall_timer(
    kill_publisher_after,
    [talker]() -> void {
      if (talker->assert_node_timer_) {
        talker->assert_node_timer_->cancel();
      }
      if (talker->assert_topic_timer_) {
        talker->assert_topic_timer_->cancel();
      }
    });
    /*
    TODO(emersonknapp) once new types are available
    [&exec, talker, liveliness_policy_kind]() -> void {
      switch (liveliness_policy_kind) {
        case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
          exec.remove_node(talker);
          break;
        case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE:
          if (talker->assert_node_timer_) {
            talker->assert_node_timer_->cancel();
          }
          break;
        case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
          if (talker->assert_topic_timer_) {
            talker->assert_topic_timer_->cancel();
          }
          break;
        default:
          break;
      }
    });
    */

  exec.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
