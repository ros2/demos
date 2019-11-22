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
  printf("Usage:\n");
  printf("incompatible_qos "
    "incompatible_qos_policy_name "
    "[-h]\n");
  printf("required arguments:\n");
  printf("incompatible_qos_policy_name: "
    "The QoS Policy which will have an incompatible value.\n"
	" durability, deadline, liveliness_policy, liveliness_lease_duration and reliability\n");
  printf("optional arguments:\n");
  printf("-h : Print this help message.\n");
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

  rclcpp::QoS qos_profile_publisher(10);
  rclcpp::QoS qos_profile_subscription(10);

  // Configuration variables
  std::string qos_policy_name = argv[1];
  if (qos_policy_name.compare("durability") == 0) {
	  printf("Durability incompatibility selected.\n"
			  "Incompatibility condition: publisher durability kind < "
			  "subscripition durability kind.\n"
			  "Setting publisher durability to: VOLATILE\n"
			  "Setting subscription durability to: TRANSIENT_LOCAL\n");
	  qos_profile_publisher.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	  qos_profile_subscription.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  } else if (qos_policy_name.compare("deadline") == 0) {
	  printf("Deadline incompatibility selected.\n"
			  "Incompatibility condition: publisher deadline > subscription deadline\n"
			  "Setting publisher deadline to: 2 seconds\n"
			  "Setting subscription deadline to: 1 second\n");
	  qos_profile_publisher.deadline(std::chrono::milliseconds(2000));
	  qos_profile_subscription.deadline(std::chrono::milliseconds(1000));
  } else if (qos_policy_name.compare("liveliness_policy") == 0) {
	  printf("Liveliness Policy incompatibility selected.\n"
			  "Incompatibility condition: publisher liveliness policy < "
			  "subscripition liveliness policy.\n"
			  "Setting publisher liveliness policy to: MANUAL_BY_NODE\n"
			  "Setting subscription liveliness policy to: MANUAL_BY_TOPIC\n");
	  qos_profile_publisher.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE);
	  qos_profile_subscription.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  } else if (qos_policy_name.compare("liveliness_lease_duration") == 0) {
	  printf("Liveliness lease duration incompatibility selected.\n"
			  "Incompatibility condition: publisher liveliness lease duration > "
			  "subscription liveliness lease duration\n"
			  "Setting publisher liveliness lease duration to: 2 seconds\n"
			  "Setting subscription liveliness lease duration to: 1 seconds\n");
	  qos_profile_publisher.liveliness_lease_duration(std::chrono::milliseconds(2000));
	  qos_profile_subscription.liveliness_lease_duration(std::chrono::milliseconds(1000));
  } else if (qos_policy_name.compare("reliability") == 0) {
	  printf("Reliability incompatibility selected.\n"
			  "Incompatibility condition: publisher reliability < subscripition reliability.\n"
			  "Setting publisher reliability to: BEST_EFFORT\n"
			  "Setting subscription reliability to: RELIABLE\n");
	  qos_profile_publisher.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
	  qos_profile_subscription.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  } else {
	  printf("%s is not a valid qos policy name\n", qos_policy_name.c_str());
	  print_usage();
	  return 0;
  }
  printf("\n\n");
  std::string topic("qos_incompatible_qos_chatter");

  // Initialization and configuration
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto talker = std::make_shared<Talker>(qos_profile_publisher, topic, 5);
  talker->get_options().event_callbacks.incompatible_qos_callback =
    [node = talker.get()](rclcpp::QOSOfferedIncompatibleQoSInfo & event) -> void
    {
      RCLCPP_INFO(node->get_logger(),
        "Offered incompatible qos - total %d delta %d last_policy_id: %d",
        event.total_count, event.total_count_change, event.last_policy_id);
    };

  auto listener = std::make_shared<Listener>(qos_profile_subscription, topic);
  listener->get_options().event_callbacks.incompatible_qos_callback =
    [node = listener.get()](rclcpp::QOSRequestedIncompatibleQoSInfo & event) -> void
    {
      RCLCPP_INFO(node->get_logger(),
        "Requested incompatible qos - total %d delta %d last_policy_id: %d",
        event.total_count, event.total_count_change, event.last_policy_id);
    };


  talker->initialize();
  listener->initialize();

  // Execution
  executor.add_node(talker);
  executor.add_node(listener);
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
