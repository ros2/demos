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

#ifndef IMAGE_TOOLS__CAM2IMAGE_HPP_
#define IMAGE_TOOLS__CAM2IMAGE_HPP_

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <memory>
#include <utility>
#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "../src/burger.hpp"

#include "image_tools/options.hpp"
#include "image_tools/visibility_control.h"


namespace image_tools
{

class Cam2Image : public rclcpp::Node
{
public:
  IMAGE_TOOLS_PUBLIC
  explicit Cam2Image(rclcpp::NodeOptions options);

  IMAGE_TOOLS_PUBLIC
  explicit Cam2Image(rclcpp::NodeOptions options, int argc, char ** argv);

  IMAGE_TOOLS_PUBLIC
  std::string
  mat_type2encoding(int mat_type);

  IMAGE_TOOLS_PUBLIC
  void convert_frame_to_message(
    const cv::Mat & frame,
    size_t frame_id, sensor_msgs::msg::Image & msg);

  IMAGE_TOOLS_PUBLIC
  void execute();

  IMAGE_TOOLS_PUBLIC
  bool setup(int argc, char ** argv);

protected:
  void on_timer();

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool show_camera_ = false;
  size_t depth_ = rmw_qos_profile_default.depth;
  double freq_ = 30.0;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  size_t width_ = 200;
  size_t height_ = 120;
  bool burger_mode_ = false;
  std::string topic_ = "image";
};

}  // namespace image_tools

#endif  // IMAGE_TOOLS__CAM2IMAGE_HPP_
