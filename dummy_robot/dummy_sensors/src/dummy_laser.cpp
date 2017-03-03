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

#include "sensor_msgs/msg/laser_scan.hpp"

#define DEG2RAD M_PI / 180.0

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("dummy_laser");

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan");

  rclcpp::WallRate loop_rate(30);

  auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  msg->header.frame_id = "dummy_laser_link";

  double angle_resolution = 2500;
  double start_angle = -450000;
  double stop_angle = 2250000;
  double scan_frequency = 2500;

  double angle_range = stop_angle - start_angle;
  double num_values = angle_range / angle_resolution;
  if (static_cast<int>(angle_range) % static_cast<int>(angle_resolution) == 0) {
    // Include endpoint
    ++num_values;
  }
  msg->ranges.resize(num_values);
  msg->intensities.resize(num_values);

  msg->time_increment = (angle_resolution / 10000.0) / 360.0 / (scan_frequency / 100.0);
  msg->angle_increment = angle_resolution / 10000.0 * DEG2RAD;
  msg->angle_min = start_angle / 10000.0 * DEG2RAD - M_PI / 2;
  msg->angle_max = stop_angle / 10000.0 * DEG2RAD - M_PI / 2;
  msg->scan_time = 100.0 / scan_frequency;
  msg->range_min = 0.0;
  msg->range_max = 10.0;

  printf("angle inc:\t%f\n", msg->angle_increment);
  printf("scan size:\t%zu\n", msg->ranges.size());
  printf("scan time increment: \t%f\n", msg->time_increment);

  auto counter = 0.0;
  auto amplitude = 1;
  auto distance = 0.0;
  while (rclcpp::ok()) {
    counter += 0.1;
    distance = std::abs(amplitude * std::sin(counter));

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      msg->ranges[i] = distance;
    }

    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    if (now <= std::chrono::nanoseconds(0)) {
      msg->header.stamp.sec = msg->header.stamp.nanosec = 0;
    } else {
      msg->header.stamp.sec =
        static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
      msg->header.stamp.nanosec = now.count() % 1000000000;
    }

    laser_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
