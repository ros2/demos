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

static const size_t default_history = 10;
static const size_t default_publish_n_messages = 5;
static const size_t default_subscribe_after_duration_ms = 5000;

void print_usage()
{
  printf("Usage for lifespan:\n");
  printf("lifespan lifespan_duration [--history history_depth] [-p publish_n_messages] "
    "[-s subscribe_after_duration] [-h]\n");
  printf("required arguments:\n");
  printf("lifespan duration: Duration (in ms) of the Lifespan QoS setting.\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("--history : The depth of the Publisher's history queue - the maximum number of messages "
    "it will store. Defaults to %zu\n", default_history);
  printf("-p publish_n_messages : How many messages to publish before stopping. Defaults to %zu\n",
    default_publish_n_messages);
  printf("-s subscribe_after_duration : The Subscriber will be created this long (in ms) after "
    "application startup. Defaults to %zu\n", default_subscribe_after_duration_ms);
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
  size_t lifespan_duration_ms = std::stoul(argv[1]);

  // Optional argument default values
  size_t history = default_history;
  size_t publish_n_messages = default_publish_n_messages;
  std::chrono::milliseconds subscribe_after_duration(default_subscribe_after_duration_ms);

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, "--history")) {
    history = std::stoul(rcutils_cli_get_option(argv, argv + argc, "--history"));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "-p")) {
    publish_n_messages = std::stoul(rcutils_cli_get_option(argv, argv + argc, "-p"));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "-s")) {
    subscribe_after_duration = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, "-s")));
  }

  // Configuration and Initialization
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::string topic("qos_lifespan_chatter");

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.depth = history;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  /*
  TODO(emersonknapp) once new types are available
  qos_profile.lifespan.sec = lifespan_duration_ms / 1000;
  qos_profile.lifespan.nsec = (lifespan_duration_ms % 1000) * 1000000;
  */

  rclcpp::SubscriptionOptions<> sub_options;
  sub_options.qos_profile = qos_profile;
  rclcpp::PublisherOptions<> pub_options;
  pub_options.qos_profile = qos_profile;

  auto listener = std::make_shared<Listener>(topic, sub_options, true);
  auto talker = std::make_shared<Talker>(topic, pub_options, publish_n_messages);

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
