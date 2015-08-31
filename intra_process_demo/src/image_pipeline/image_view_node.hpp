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

#ifndef INTRA_PROCESS_COMMS_EXAMPLE_IMAGE_VIEW_NODE_HPP_
#define INTRA_PROCESS_COMMS_EXAMPLE_IMAGE_VIEW_NODE_HPP_

#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "common.hpp"

// Node which receives sensor_msgs/Image messages and renders them using OpenCV.
class ImageViewNode : public rclcpp::Node
{
public:
  ImageViewNode(const std::string & input, const std::string & node_name = "image_view_node")
  : Node(node_name, true)
  {
    // Create a subscription on the input topic.
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(input, rmw_qos_profile_sensor_data,
      [node_name](const sensor_msgs::msg::Image::SharedPtr msg) {
        // Create a cv::Mat from the image message (without copying).
        cv::Mat cv_mat(
          msg->width, msg->height,
          encoding2mat_type(msg->encoding),
          msg->data.data());
        // Annotate with the pid and pointer address.
        std::stringstream ss;
        ss << "pid: " << GETPID() << ", ptr: " << msg.get();
        cv::putText(cv_mat, ss.str(), cvPoint(30, 90),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(0, 255, 0), 1, CV_AA);
        // Show the image.
        cv::imshow(node_name, cv_mat);
        char key = cv::waitKey(1);  // Look for key presses.
        if (key == 27 /* ESC */ || key == 'q') {
          rclcpp::shutdown();
        }
        if (key == ' ') {  // If <space> then pause until another <space>.
          key = '\0';
          while (key != ' ') {
            key = cv::waitKey(1);
          }
        }
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

#endif  /* INTRA_PROCESS_COMMS_EXAMPLE_IMAGE_VIEW_NODE_HPP_ */
