// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rttest/utils.h"

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"
#include "pendulum_msgs/msg/rttest_results.hpp"

// Non real-time safe node for logging (file IO, console output)

int main(int argc, char * argv[])
{
  setbuf(stdout, NULL);
  rclcpp::init(argc, argv);

  auto logger_node = rclcpp::Node::make_shared("pendulum_logger");
  std::string filename = "pendulum_logger_results.csv";
  std::ofstream fstream;
  {
    fstream.open(filename, std::ios_base::out);
    fstream << "iteration timestamp latency minor_pagefaults minor_pagefaults" << std::endl;
    fstream.close();
  }

  fstream.open(filename, std::ios_base::app);
  size_t i = 0;
  auto logging_callback =
    [&filename, &i](const pendulum_msgs::msg::RttestResults::SharedPtr msg) {
      printf("Commanded motor angle: %f\n", msg->command.position);
      printf("Actual motor angle: %f\n", msg->state.position);

      printf("Current latency: %lu ns\n", msg->cur_latency);
      printf("Mean latency: %f ns\n", msg->mean_latency);
      printf("Min latency: %lu ns\n", msg->min_latency);
      printf("Max latency: %lu ns\n", msg->max_latency);

      printf("Minor pagefaults during execution: %lu\n", msg->minor_pagefaults);
      printf("Major pagefaults during execution: %lu\n\n", msg->major_pagefaults);

      std::ofstream fstream;
      struct timespec timestamp;
      timestamp.tv_sec = msg->stamp.sec;
      timestamp.tv_nsec = msg->stamp.nanosec;
      fstream << i << " " << timespec_to_long(&timestamp) <<
        " " << msg->cur_latency << " " <<
        msg->minor_pagefaults << " " <<
        msg->major_pagefaults << std::endl;
      ++i;
    };

  // The quality of service profile is tuned for real-time performance.
  // More QoS settings may be exposed by the rmw interface in the future to fulfill real-time
  // requirements.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
  // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
  // are sent, to aid with recovery in the event of dropped messages.
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile.depth = 100;

  auto subscription = logger_node->create_subscription<pendulum_msgs::msg::RttestResults>(
    "pendulum_statistics", logging_callback, qos_profile);

  printf("Logger node initialized.\n");
  rclcpp::spin(logger_node);
  fstream.close();

  rclcpp::shutdown();

  return 0;
}
