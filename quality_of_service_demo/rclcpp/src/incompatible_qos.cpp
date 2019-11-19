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

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  std::string topic("qos_deadline_chatter");

  // Initialization and configuration
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp::QoS qos_profile_publisher(10);
  qos_profile_publisher.deadline(std::chrono::milliseconds(2000));

  rclcpp::QoS qos_profile_subscription(10);
  qos_profile_subscription.deadline(std::chrono::milliseconds(1000));

  auto talker = std::make_shared<Talker>(qos_profile_publisher, topic);
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

  auto pause_timer = talker->create_wall_timer(
    std::chrono::milliseconds(1000),
    [talker]() -> void {
      talker->pause_publish_for(std::chrono::milliseconds(5000));
    });

  // Execution
  executor.add_node(talker);
  executor.add_node(listener);
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
