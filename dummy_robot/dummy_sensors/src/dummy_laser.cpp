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

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <math.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#define DEG2RAD M_PI / 180.0

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("dummy_laser");

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  rclcpp::WallRate loop_rate(30);

  sensor_msgs::msg::LaserScan msg;
  msg.header.frame_id = "single_rrbot_hokuyo_link";

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
  msg.ranges.resize(static_cast<int>(num_values));

  msg.time_increment =
    static_cast<float>((angle_resolution / 10000.0) / 360.0 / (scan_frequency / 100.0));
  msg.angle_increment = static_cast<float>(angle_resolution / 10000.0 * DEG2RAD);
  msg.angle_min = static_cast<float>(start_angle / 10000.0 * DEG2RAD - M_PI / 2);
  msg.angle_max = static_cast<float>(stop_angle / 10000.0 * DEG2RAD - M_PI / 2);
  msg.scan_time = static_cast<float>(100.0 / scan_frequency);
  msg.range_min = 0.0f;
  msg.range_max = 10.0f;

  RCLCPP_INFO(node->get_logger(), "angle inc:\t%f", msg.angle_increment);
  RCLCPP_INFO(node->get_logger(), "scan size:\t%zu", msg.ranges.size());
  RCLCPP_INFO(node->get_logger(), "scan time increment: \t%f", msg.time_increment);

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  auto counter = 0.0;
  auto amplitude = 1;
  auto distance = 0.0f;
  while (rclcpp::ok()) {
    counter += 0.1;
    distance = static_cast<float>(std::abs(amplitude * std::sin(counter)));

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      msg.ranges[i] = distance;
    }

    msg.header.stamp = clock->now();

    laser_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
