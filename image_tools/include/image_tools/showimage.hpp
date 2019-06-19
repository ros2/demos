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

#ifndef IMAGE_TOOLS__SHOWIMAGE_HPP_
#define IMAGE_TOOLS__SHOWIMAGE_HPP_

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "image_tools/visibility_control.h"


namespace image_tools
{

class ShowImage : public rclcpp::Node
{
public:
  IMAGE_TOOLS_PUBLIC
  explicit ShowImage(const rclcpp::NodeOptions & options);

  IMAGE_TOOLS_PUBLIC
  void
  show_image(
    const sensor_msgs::msg::Image::SharedPtr msg,
    bool show_camera, rclcpp::Logger logger);

  IMAGE_TOOLS_PUBLIC
  int
  encoding2mat_type(const std::string & encoding);

  IMAGE_TOOLS_PUBLIC
  bool setup(std::vector<std::string> args);

  /// Execute main functions with image subscriber and callback
  IMAGE_TOOLS_PUBLIC
  void execute();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  bool show_camera_ = true;
  std::string topic_ = "image";
};

}  // namespace image_tools

#endif  // IMAGE_TOOLS__SHOWIMAGE_HPP_
