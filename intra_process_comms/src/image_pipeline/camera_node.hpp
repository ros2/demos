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

#ifndef INTRA_PROCESS_COMMS_EXAMPLE_CAMERA_NODE_HPP_
#define INTRA_PROCESS_COMMS_EXAMPLE_CAMERA_NODE_HPP_

#include <chrono>
#include <sstream>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "common.hpp"

class CameraNode : public rclcpp::Node
{
public:
  CameraNode(const std::string & output, const std::string & node_name = "camera_node",
    int device = 0, int width = 320, int height = 240)
  : Node(node_name, true), canceled_(false)
  {
    // Initialize OpenCV
    cap_.open(device);
    cap_.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap_.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if (!cap_.isOpened()) {
      throw std::runtime_error("Could not open video stream!");
    }
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.history = RMW_QOS_POLICY_KEEP_LAST_HISTORY;
    qos.depth = 1;

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, qos);

    thread_ = std::thread(std::bind(&CameraNode::loop, this));
  }

  virtual ~CameraNode()
  {
    canceled_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void loop()
  {
    while (rclcpp::ok() && !canceled_.load()) {
      cap_ >> frame_;
      if (frame_.empty()) {
        continue;
      }
      sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
      std::stringstream ss;
      ss << msg.get();
      cv::putText(frame_, ss.str(), cvPoint(30, 30),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(0, 255, 0), 1, CV_AA);
      set_now(msg->header.stamp);
      msg->header.frame_id = "camera_frame";
      msg->height = frame_.cols;
      msg->width = frame_.rows;
      msg->encoding = mat_type2encoding(frame_.type());
      msg->is_bigendian = false;
      msg->step = frame_.step;
      msg->data.assign(frame_.datastart, frame_.dataend);
      pub_->publish(msg);
    }
  }

private:
  rclcpp::Publisher::SharedPtr pub_;
  std::thread thread_;
  std::atomic<bool> canceled_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

#endif  /* INTRA_PROCESS_COMMS_EXAMPLE_CAMERA_NODE_HPP_ */
