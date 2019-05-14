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
  printf("deadline deadline_duration [-p period_pause_talker] [-d duration_pause_talker] [-h]\n");
  printf("required arguments:\n");
  printf("deadline_duration: Duration (in uint milliseconds) of the Deadline QoS setting.\n");
  printf("options:\n");
  printf("-h : Print this help message.\n");
  printf("%s duration_publish_for : How long to publish (in uint milliseconds) until pausing "
    "the talker. Defaults to %zu.\n",
    OPTION_PUBLISH_FOR, DEFAULT_PUBLISH_FOR);
  printf("%s duration_pause_for : How long to pause the talker (in uint milliseconds) before "
    "beginning to publish again. Defaults to %zu.\n",
    OPTION_PAUSE_FOR, DEFAULT_PAUSE_FOR);
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
  std::chrono::milliseconds deadline_duration(std::stoul(argv[1]));

  // Optional argument default values
  std::chrono::milliseconds period_pause_talker(DEFAULT_PUBLISH_FOR);
  std::chrono::milliseconds duration_pause_talker(DEFAULT_PAUSE_FOR);

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
  rclcpp::executors::SingleThreadedExecutor exec;

  std::string topic("qos_deadline_chatter");

  rclcpp::QoS qos_profile(10);
  qos_profile.deadline(deadline_duration);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.deadline_callback =
    [](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCUTILS_LOG_INFO("Requested deadline missed - total %d delta %d",
        event.total_count, event.total_count_change);
    };

  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.deadline_callback =
    [](rclcpp::QOSDeadlineOfferedInfo & event) -> void
    {
      RCUTILS_LOG_INFO("Offered deadline missed - total %d delta %d",
        event.total_count, event.total_count_change);
    };

  auto talker = std::make_shared<Talker>(topic, qos_profile, pub_options);
  auto listener = std::make_shared<Listener>(topic, qos_profile, sub_options);

  auto pause_timer = talker->create_wall_timer(
    period_pause_talker,
    [talker, duration_pause_talker]() -> void {
      talker->pause_for(duration_pause_talker);
    });

  // Execution
  exec.add_node(talker);
  exec.add_node(listener);
  exec.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
