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

static const unsigned default_period_pause_talker = 5000;
static const unsigned default_duration_pause_talker = 1000;

void print_usage()
{
  printf("Usage for deadline:\n");
  printf("deadline deadline_duration [-p period_pause_talker] [-d duration_pause_talker] [-h]\n");
  printf("required arguments:\n");
  printf("deadline_duration: Duration (in ms) of the Deadline QoS setting.\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-p period_pause_talker : How often to pause the talker (in ms). Defaults to %u.\n",
    default_period_pause_talker);
  printf("-d duration_pause_talker : How long to pause the talker (in ms). Defaults to %u.\n",
    default_duration_pause_talker);
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
  size_t deadline_duration_ms = std::stoul(argv[1]);

  // Optional argument default values
  std::chrono::milliseconds period_pause_talker(default_period_pause_talker);
  std::chrono::milliseconds duration_pause_talker(default_duration_pause_talker);

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, "-p")) {
    period_pause_talker = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, "-p")));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, "-d")) {
    duration_pause_talker = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, "-d")));
  }

  // Initialization and configuration
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::string topic("qos_deadline_chatter");

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  /*
  TODO(emersonknapp) once the new types have been added
  qos_profile.deadline.sec = deadline_duration_ms / 1000;
  qos_profile.deadline.nsec = (deadline_duration_ms % 1000) * 1000000;
  */

  rclcpp::SubscriptionOptions<> sub_options;
  sub_options.qos_profile = qos_profile;
  /*
  TODO(emersonknapp) once the callbacks have been added
  sub_options.event_callbacks.deadline_callback =
    [](rclcpp::DeadlineRequestedInfo & event) -> void
    {
      RCUTILS_LOG_INFO("Requested deadline missed - total %d delta %d",
        event.total_count, event.total_count_change);
    });
  */

  rclcpp::PublisherOptions<> pub_options;
  pub_options.qos_profile = qos_profile;
  /*
  TODO(emersonknapp) once the callbacks have been added
  pub_options.event_callbacks.deadline_callback =
    [](rclcpp::DeadlineOfferedInfo & event) -> void
    {
      RCUTILS_LOG_INFO("Offered deadline missed - total %d delta %d",
        event.total_count, event.total_count_change);
    });
  */


  auto talker = std::make_shared<Talker>(topic, pub_options);
  auto listener = std::make_shared<Listener>(topic, sub_options);

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
