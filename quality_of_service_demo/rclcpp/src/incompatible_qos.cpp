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

void print_usage()
{
  std::cout << "Usage:\n";
  std::cout << "incompatible_qos [-h] <incompatible_qos_policy_name>\n\n";
  std::cout << "required arguments:\n";
  std::cout << "incompatible_qos_policy_name: The QoS Policy that should be incompatible between\n"
    "                              the publisher and subscription (durability, deadline,\n"
    "                              liveliness_policy, liveliness_lease_duration,\n"
    "                              or reliability).\n\n";
  std::cout << "optional arguments:\n";
  std::cout << "-h:                           Print this help message.\n";
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
  std::string qos_policy_name(argv[1]);
  rclcpp::QoS qos_profile_publisher(10);
  rclcpp::QoS qos_profile_subscription(10);

  if (qos_policy_name == "durability") {
    std::cout << "Durability incompatibility selected.\n"
      "Incompatibility condition: publisher durability kind < "
      "subscription durability kind.\n"
      "Setting publisher durability to: VOLATILE\n"
      "Setting subscription durability to: TRANSIENT_LOCAL\n";
    qos_profile_publisher.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_profile_subscription.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  } else if (qos_policy_name == "deadline") {
    std::cout << "Deadline incompatibility selected.\n"
      "Incompatibility condition: publisher deadline > subscription deadline\n"
      "Setting publisher deadline to: 2 seconds\n"
      "Setting subscription deadline to: 1 second\n";
    qos_profile_publisher.deadline(std::chrono::milliseconds(2000));
    qos_profile_subscription.deadline(std::chrono::milliseconds(1000));
  } else if (qos_policy_name == "liveliness_policy") {
    std::cout << "Liveliness Policy incompatibility selected.\n"
      "Incompatibility condition: publisher liveliness policy < "
      "subscripition liveliness policy.\n"
      "Setting publisher liveliness policy to: AUTOMATIC\n"
      "Setting subscription liveliness policy to: MANUAL_BY_TOPIC\n";
    qos_profile_publisher.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    qos_profile_subscription.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  } else if (qos_policy_name == "liveliness_lease_duration") {
    std::cout << "Liveliness lease duration incompatibility selected.\n"
      "Incompatibility condition: publisher liveliness lease duration > "
      "subscription liveliness lease duration\n"
      "Setting publisher liveliness lease duration to: 2 seconds\n"
      "Setting subscription liveliness lease duration to: 1 seconds\n";
    qos_profile_publisher.liveliness_lease_duration(std::chrono::milliseconds(2000));
    qos_profile_subscription.liveliness_lease_duration(std::chrono::milliseconds(1000));
  } else if (qos_policy_name == "reliability") {
    std::cout << "Reliability incompatibility selected.\n"
      "Incompatibility condition: publisher reliability < subscripition reliability.\n"
      "Setting publisher reliability to: BEST_EFFORT\n"
      "Setting subscription reliability to: RELIABLE\n";
    qos_profile_publisher.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile_subscription.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  } else {
    std::cout << qos_policy_name << " is not a valid qos policy name\n";
    print_usage();
    return 0;
  }
  std::cout << "\n";

  // Initialization and configuration
  rclcpp::init(argc, argv);
  const std::string topic("qos_incompatible_qos_chatter");
  constexpr size_t num_msgs = 5;

  auto talker = std::make_shared<Talker>(qos_profile_publisher, topic, num_msgs);
  talker->get_options().event_callbacks.incompatible_qos_callback =
    [node = talker.get()](rclcpp::QOSOfferedIncompatibleQoSInfo & event) -> void
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Offered incompatible qos - total %d delta %d last_policy_kind: %d",
        event.total_count, event.total_count_change, event.last_policy_kind);
    };

  auto listener = std::make_shared<Listener>(qos_profile_subscription, topic);
  listener->get_options().event_callbacks.incompatible_qos_callback =
    [node = listener.get()](rclcpp::QOSRequestedIncompatibleQoSInfo & event) -> void
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Requested incompatible qos - total %d delta %d last_policy_kind: %d",
        event.total_count, event.total_count_change, event.last_policy_kind);
    };

  try {
    talker->initialize();
    listener->initialize();
  } catch (const rclcpp::UnsupportedEventTypeException & exc) {
    std::cout << '\n' << exc.what() << "\n\n"
      "Please try this demo using a different RMW implementation\n";
    return -1;
  }

  // Execution
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(talker);
  executor.add_node(listener);

  while (talker->get_published_count() < num_msgs) {
    executor.spin_once();
  }

  // Cleanup
  rclcpp::shutdown();

  return 0;
}
