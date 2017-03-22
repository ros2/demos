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
#include "rcl/rcl.h"

#include "nav_msgs/msg/occupancy_grid.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("dummy_map_server");

  rmw_qos_profile_t latched_qos = rmw_qos_profile_default;
  latched_qos.depth = 1;
  latched_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", latched_qos);

  rclcpp::WallRate loop_rate(1);

  auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  msg->header.frame_id = "world";

  msg->info.resolution = 0.1f;
  msg->info.width = 100;
  msg->info.height = 100;
  msg->info.origin.position.x = -(msg->info.width * msg->info.resolution) / 2;
  msg->info.origin.position.y = -(msg->info.width * msg->info.resolution) / 2;
  msg->info.origin.position.z = 0;
  msg->info.origin.orientation.x = 0;
  msg->info.origin.orientation.y = 0;
  msg->info.origin.orientation.z = 0;
  msg->info.origin.orientation.w = 1;

  for (size_t i = 0; i < msg->info.width * msg->info.height; ++i) {
    msg->data.push_back(-1);
  }

  int lhs = 0;
  int center = 1;
  int rhs = 2;
  while (rclcpp::ok()) {
    msg->data[(lhs) % (msg->info.width * msg->info.height)] = -1;
    msg->data[(center) % (msg->info.width * msg->info.height)] = -1;
    msg->data[(rhs) % (msg->info.width * msg->info.height)] = -1;
    msg->data[(++lhs) % (msg->info.width * msg->info.height)] = 127;
    msg->data[(++center) % (msg->info.width * msg->info.height)] = 128;
    msg->data[(++rhs) % (msg->info.width * msg->info.height)] = 127;

    msg->header.stamp = rclcpp::Time::now();

    map_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
