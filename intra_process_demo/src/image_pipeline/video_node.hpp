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

#ifndef IMAGE_PIPELINE__VIDEO_NODE_HPP_
#define IMAGE_PIPELINE__VIDEO_NODE_HPP_

#include <chrono>
#include <sstream>
#include <string>
#include <thread>

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "common.hpp"

class VideoNode : public rclcpp::Node
{
public:
  virtual ~VideoNode()
  {
    // Make sure to join the thread on shutdown.
    canceled_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void loop()
  {
    // While running...
    while (rclcpp::ok() && !canceled_.load()) {
      // Capture a frame from OpenCV.
      cap_ >> frame_;
      if (frame_.empty()) {
        continue;
      }
      // Create a new unique_ptr to an Image message for storage.
      sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
      std::stringstream ss;
      // Put this process's id and the msg's pointer address on the image.
      ss << "pid: " << GETPID() << ", ptr: " << msg.get();
      draw_on_image(frame_, ss.str(), 20);
      // Pack the OpenCV image into the ROS image.
      set_now(msg->header.stamp);
      msg->header.frame_id = "camera_frame";
      msg->height = frame_.cols;
      msg->width = frame_.rows;
      msg->encoding = mat_type2encoding(frame_.type());
      msg->is_bigendian = false;
      msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
      msg->data.assign(frame_.datastart, frame_.dataend);
      pub_->publish(msg);  // Publish.
    }
  }

protected:
  explicit VideoNode(const std::string & output, const std::string & node_name = "camera_node")
  : rclcpp::Node(node_name, true), canceled_(false)
  {}

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::thread thread_;
  std::atomic<bool> canceled_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

#endif  // IMAGE_PIPELINE__VIDEO_NODE_HPP_
