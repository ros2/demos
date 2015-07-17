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

#include <sensor_interfaces/msg/image.hpp>


std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "rgb8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("cam2image");

  auto pub = node->create_publisher<sensor_interfaces::msg::Image>("image", 10);

  rclcpp::WallRate loop_rate(30);

  cv::VideoCapture cap;
  cap.open(0);
  if (!cap.isOpened()){
    fprintf(stderr, "Could not open video stream\n");
    return 1;
  }

  cv::Mat frame;

  auto msg = std::make_shared<sensor_interfaces::msg::Image>();
  msg->is_bigendian = false;

  auto i = 1;

  while (rclcpp::ok()) {
    cap >> frame;
    // check if the frame was grabbed correctly
    if (!frame.empty()) {
      // copy cv information into ros message
      msg->height = frame.rows;
      msg->width = frame.cols;
      msg->encoding = mat_type2encoding(frame.type());
      msg->step = frame.step;
      size_t size = frame.step * frame.rows;
      msg->data.resize(size);
      memcpy(&msg->data[0], frame.data, size);

      std::cout << "Publishing image #" << i << std::endl;
      pub->publish(msg);
      ++i;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
