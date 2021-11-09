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

#ifndef IMAGE_PIPELINE__IMAGE_VIEW_NODE_HPP_
#define IMAGE_PIPELINE__IMAGE_VIEW_NODE_HPP_

#include <sstream>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "common.hpp"

/// Node which receives sensor_msgs/Image messages and renders them using OpenCV.
class ImageViewNode final : public rclcpp::Node
{
public:
  /// \brief Construct a new ImageViewNode for visualizing image data
  /// \param input The topic name to subscribe to
  /// \param node_name The node name to use
  /// \param watermark Whether to add a watermark to the image before displaying
  explicit ImageViewNode(
    const std::string & input, const std::string & node_name = "image_view_node",
    bool watermark = true)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic.
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input,
      rclcpp::SensorDataQoS(),
      [node_name, watermark](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Create a cv::Mat from the image message (without copying).
        cv::Mat cv_mat(
          msg->height, msg->width,
          encoding2mat_type(msg->encoding),
          const_cast<unsigned char *>(msg->data.data()));
        if (watermark) {
          // Annotate with the pid and pointer address.
          std::stringstream ss;
          ss << "pid: " << GETPID() << ", ptr: " << msg.get();
          draw_on_image(cv_mat, ss.str(), 60);
        }
        // Show the image.
        cv::Mat c_mat = cv_mat;
        cv::imshow(node_name.c_str(), c_mat);
        char key = cv::waitKey(1);    // Look for key presses.
        if (key == 27 /* ESC */ || key == 'q') {
          rclcpp::shutdown();
        }
        if (key == ' ') {    // If <space> then pause until another <space>.
          key = '\0';
          while (key != ' ') {
            key = cv::waitKey(1);
            if (key == 27 /* ESC */ || key == 'q') {
              rclcpp::shutdown();
            }
            if (!rclcpp::ok()) {
              break;
            }
          }
        }
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

#endif  // IMAGE_PIPELINE__IMAGE_VIEW_NODE_HPP_
