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

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "quality_of_service_demo/common_nodes.hpp"

static const char * OPTION_PUBLISH_FOR = "--publish-for";
static const size_t DEFAULT_PUBLISH_FOR = 5000;
static const char * OPTION_PAUSE_FOR = "--pause-for";
static const size_t DEFAULT_PAUSE_FOR = 1000;

void print_usage()
{
  printf("Usage for deadline:\n");
  printf(
    "deadline "
    "deadline_duration "
    "[%s publish_for] "
    "[%s pause_for] "
    "[-h]\n",
    OPTION_PUBLISH_FOR,
    OPTION_PAUSE_FOR);
  printf("required arguments:\n");
  printf(
    "deadline_duration: "
    "Duration in positive integer milliseconds of the Deadline QoS setting.\n");
  printf("optional arguments:\n");
  printf("-h : Print this help message.\n");
  printf(
    "%s publish_for : "
    "Duration to publish (in positive integer milliseconds) until pausing the talker. "
    "Defaults to %zu.\n",
    OPTION_PUBLISH_FOR, DEFAULT_PUBLISH_FOR);
  printf(
    "%s pause_for : "
    "Duration to pause (in positive integer milliseconds) before starting to publish again. "
    "Defaults to %zu.\n",
    OPTION_PAUSE_FOR, DEFAULT_PAUSE_FOR);
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
  std::chrono::milliseconds deadline_duration(std::stoul(argv[1]));
  std::chrono::milliseconds period_pause_talker(DEFAULT_PUBLISH_FOR);
  std::chrono::milliseconds duration_pause_talker(DEFAULT_PAUSE_FOR);
  std::string topic("qos_deadline_chatter");

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_FOR)) {
    period_pause_talker = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_PUBLISH_FOR)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PAUSE_FOR)) {
    duration_pause_talker = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_PAUSE_FOR)));
  }

  // Initialization and configuration
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp::QoS qos_profile(10);
  qos_profile.deadline(deadline_duration);

  auto talker = std::make_shared<Talker>(qos_profile, topic);
  talker->get_options().event_callbacks.deadline_callback =
    [node = talker.get()](rclcpp::QOSDeadlineOfferedInfo & event) -> void
    {
      RCLCPP_INFO(
        node->get_logger(), "Offered deadline missed - total %d delta %d",
        event.total_count, event.total_count_change);
    };

  auto listener = std::make_shared<Listener>(qos_profile, topic);
  listener->get_options().event_callbacks.deadline_callback =
    [node = listener.get()](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_INFO(
        node->get_logger(), "Requested deadline missed - total %d delta %d",
        event.total_count, event.total_count_change);
    };

  talker->initialize();
  listener->initialize();

  auto pause_timer = talker->create_wall_timer(
    period_pause_talker,
    [talker, duration_pause_talker]() -> void {
      talker->pause_publish_for(duration_pause_talker);
    });

  // Execution
  executor.add_node(talker);
  executor.add_node(listener);
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
