// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("dummy_joint_states");

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states");

  rclcpp::WallRate loop_rate(50);

  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->name.push_back("single_rrbot_joint1");
  msg->name.push_back("single_rrbot_joint2");
  msg->position.push_back(0.0);
  msg->position.push_back(0.0);

  auto counter = 0.0;
  auto joint_value = 0.0;
  while (rclcpp::ok()) {
    counter += 0.1;
    joint_value = std::sin(counter);

    for (size_t i = 0; i < msg->name.size(); ++i) {
      msg->position[i] = joint_value;
    }

    // TODO(karsten1987): use rclcpp version of Time::now()
    uint32_t now_sec = 0;
    uint32_t now_nanosec = 0;
    {
      rcl_time_point_value_t now = 0;
      rcl_ret_t ret = rcl_system_time_now(&now);
      if (ret != RCL_RET_OK) {
        fprintf(stderr, "Could not get current time: %s\n", rcl_get_error_string_safe());
        exit(-1);
      }
      now_sec = RCL_NS_TO_S(now);
      now_nanosec = now % (1000 * 1000 * 1000);
    }
    msg->header.stamp.sec = now_sec;
    msg->header.stamp.nanosec = now_nanosec;

    joint_state_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
