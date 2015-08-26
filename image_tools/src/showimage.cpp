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

#include <iostream>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <image_tools/options.hpp>

int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

void show_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
  std::stringstream ss;
  ss << "Received image #" << msg->header.frame_id << std::endl;
  std::cout << ss.str();

  cv::Mat frame(
    msg->height, msg->width, encoding2mat_type(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  // NOTE(esteve): Use C version of cvShowImage to avoid this on Windows:
  // http://stackoverflow.com/questions/20854682/opencv-multiple-unwanted-window-with-garbage-name
  CvMat cvframe = frame;
  cvShowImage("showimage", &cvframe);
  cv::waitKey(1);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  size_t depth = 10;
  rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABLE;
  rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_KEEP_ALL_HISTORY;

  if (!parse_command_options(
    argc, argv, &depth, &reliability_policy, &history_policy)) {
    return 0;
  }

  cvNamedWindow("showimage", CV_WINDOW_AUTOSIZE);

  auto node = rclcpp::node::Node::make_shared("showimage");

  rmw_qos_profile_t custom_qos_profile;
  custom_qos_profile.history = history_policy;
  custom_qos_profile.depth = depth;
  custom_qos_profile.reliability = reliability_policy;

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    "image", custom_qos_profile, show_image);

  rclcpp::spin(node);

  return 0;
}
