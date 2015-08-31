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

#ifndef INTRA_PROCESS_COMMS_EXAMPLE_WATERMARK_NODE_HPP_
#define INTRA_PROCESS_COMMS_EXAMPLE_WATERMARK_NODE_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "common.hpp"

class WatermarkNode : public rclcpp::Node
{
public:
  WatermarkNode(
    const std::string & input, const std::string & output, const std::string & text,
    const std::string & node_name = "watermark_node")
  : Node(node_name, true)
  {
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.history = RMW_QOS_POLICY_KEEP_LAST_HISTORY;
    qos.depth = 1;

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, qos);

    sub_ = this->create_subscription_with_unique_ptr_callback<sensor_msgs::msg::Image>(
      input, qos,
      [this, text](sensor_msgs::msg::Image::UniquePtr & msg) {
        cv::Mat cv_mat(
          msg->width, msg->height,
          encoding2mat_type(msg->encoding),
          msg->data.data());
        std::stringstream ss;
        ss << msg.get() << " " << text;
        cv::putText(cv_mat, ss.str(), cvPoint(30, 60),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(0, 255, 0), 1, CV_AA);
        this->pub_->publish(msg);
      });

  }

private:
  rclcpp::Publisher::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

#endif  /* INTRA_PROCESS_COMMS_EXAMPLE_WATERMARK_NODE_HPP_ */
