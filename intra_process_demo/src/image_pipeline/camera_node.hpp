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

// Node which captures images from a camera using OpenCV and publishes them.
// Images are annotated with this process's id as well as the message's ptr.
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
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, rmw_qos_profile_sensor_data);
    // Create the camera reading loop.
    thread_ = std::thread(std::bind(&CameraNode::loop, this));
  }

  virtual ~CameraNode()
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

private:
  rclcpp::Publisher::SharedPtr pub_;
  std::thread thread_;
  std::atomic<bool> canceled_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

#endif  /* INTRA_PROCESS_COMMS_EXAMPLE_CAMERA_NODE_HPP_ */
