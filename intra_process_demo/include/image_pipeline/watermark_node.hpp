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

#ifndef IMAGE_PIPELINE__WATERMARK_NODE_HPP_
#define IMAGE_PIPELINE__WATERMARK_NODE_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "common.hpp"

/// Node that receives an image, adds some text as a watermark, and publishes it again.
class WatermarkNode final : public rclcpp::Node
{
public:
  /// \brief Construct a WatermarkNode that accepts an image, adds a watermark, and republishes it
  /// \param input The name of the topic to subscribe to
  /// \param output The topic to publish watermarked images to
  /// \param text The text to add to the image
  /// \param node_name The node name to use
  explicit WatermarkNode(
    const std::string & input, const std::string & output, const std::string & text,
    const std::string & node_name = "watermark_node")
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    rclcpp::SensorDataQoS qos;
    // Create a publisher on the input topic.
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, qos);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a subscription on the output topic.
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input,
      qos,
      [captured_pub, text](sensor_msgs::msg::Image::UniquePtr msg) {
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        // Create a cv::Mat from the image message (without copying).
        cv::Mat cv_mat(
          msg->height, msg->width,
          encoding2mat_type(msg->encoding),
          msg->data.data());
        // Annotate the image with the pid, pointer address, and the watermark text.
        std::stringstream ss;
        ss << "pid: " << GETPID() << ", ptr: " << msg.get() << " " << text;
        draw_on_image(cv_mat, ss.str(), 40);
        pub_ptr->publish(std::move(msg));  // Publish it along.
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

#endif  // IMAGE_PIPELINE__WATERMARK_NODE_HPP_
