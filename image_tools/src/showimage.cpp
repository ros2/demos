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

#include <sensor_interfaces/msg/image.hpp>


int
encoding2mat_type(const std::string & encoding)
{
  if(encoding == "mono8") {
    return CV_8UC1;
  } else if(encoding == "rgb8") {
    return CV_8UC3;
  } else if(encoding == "mono16") {
    return CV_16SC1;
  } else if(encoding == "rgba8") {
    return CV_8UC4;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

void show_image(const sensor_interfaces::msg::Image::ConstSharedPtr & msg) {
  std::stringstream ss;
  ss << "Received image #" << msg->header.frame_id << std::endl;
  std::cout << ss.str();

  unsigned char *data = new unsigned char[msg->data.size()];

  std::copy(msg->data.begin(), msg->data.end(), data);

  cv::Mat frame(
    msg->height, msg->width, encoding2mat_type(msg->encoding),
    data, msg->step);

  cv::imshow("view", frame);
  cv::waitKey(1);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("showimage");

  auto sub = node->create_subscription<sensor_interfaces::msg::Image>("image", 10, show_image);

  rclcpp::spin(node);

  return 0;
}
