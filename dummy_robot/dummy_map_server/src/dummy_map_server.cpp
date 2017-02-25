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

#include "nav_msgs/msg/occupancy_grid.hpp"

#define DEG2RAD M_PI/180.0

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("dummy_map_server");

  auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map");

  rclcpp::WallRate loop_rate(1);

  auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  msg->header.frame_id = "base_link";

  msg->info.resolution = 0.1;
  msg->info.width = 100;
  msg->info.height = 100;
  msg->info.origin.position.x = -(msg->info.width * msg->info.resolution) / 2;
  msg->info.origin.position.y = -(msg->info.width * msg->info.resolution) / 2;
  msg->info.origin.position.z = 0;
  msg->info.origin.orientation.x = 0;
  msg->info.origin.orientation.y = 0;
  msg->info.origin.orientation.z = 0;
  msg->info.origin.orientation.w = 1;

  for (size_t i = 0; i < msg->info.width * msg->info.height; ++i)
  {
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


    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    if (now <= std::chrono::nanoseconds(0)) {
      msg->header.stamp.sec = msg->header.stamp.nanosec = 0;
    } else {
      msg->header.stamp.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
      msg->header.stamp.nanosec = now.count() % 1000000000;
    }

    map_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
