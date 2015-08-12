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

#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include <image_tools/options.hpp>

std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void convert_frame_to_message(
  const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
{
  // copy cv information into ros message
  msg->height = frame.rows;
  msg->width = frame.cols;
  msg->encoding = mat_type2encoding(frame.type());
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size);
  msg->header.frame_id = std::to_string(frame_id);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  bool show_camera = false;
  size_t depth = 10;
  rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABLE;
  rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_KEEP_ALL_HISTORY;
  size_t width = 320;
  size_t height = 240;

  if (!parse_command_options(
    argc, argv, &depth, &reliability_policy, &history_policy, &show_camera,
    &width, &height)) {
    return 0;
  }

  auto node = rclcpp::node::Node::make_shared("cam2image");

  rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;
  custom_camera_qos_profile.depth = depth;
  custom_camera_qos_profile.reliability = reliability_policy;
  custom_camera_qos_profile.history = history_policy;

  auto pub = node->create_publisher<sensor_msgs::msg::Image>(
    "image", custom_camera_qos_profile);

  bool is_flipped = false;
  auto callback =
    [&is_flipped](const std_msgs::msg::Bool::SharedPtr msg) -> void
    {
      is_flipped = msg->data;
      printf("Set flip mode to: %s\n", is_flipped ? "on" : "off");
    };

  rmw_qos_profile_t custom_flip_qos_profile = rmw_qos_profile_default;
  custom_flip_qos_profile.depth = 10;

  auto sub = node->create_subscription<std_msgs::msg::Bool>(
    "flip_image", custom_flip_qos_profile, callback);

  rclcpp::WallRate loop_rate(30);

  cv::VideoCapture cap;
  cap.open(0);
  if (!cap.isOpened()){
    fprintf(stderr, "Could not open video stream\n");
    return 1;
  }

  cv::Mat frame;
  cv::Mat flipped_frame;

  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->is_bigendian = false;

  size_t i = 1;

  while (rclcpp::ok()) {
    cap >> frame;
    // check if the frame was grabbed correctly
    if (!frame.empty()) {
      if (!is_flipped) {
        convert_frame_to_message(frame, i, msg);
      } else {
        cv::flip(frame, flipped_frame, 1);
        convert_frame_to_message(flipped_frame, i, msg);
      }
      if (show_camera) {
        cv::imshow("cam2image", frame);
        cv::waitKey(1);
      }
      std::cout << "Publishing image #" << i << std::endl;
      pub->publish(msg);
      ++i;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
