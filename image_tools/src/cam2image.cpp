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

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "image_tools/cam2image.hpp"


namespace image_tools
{
/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string
Cam2Image::mat_type2encoding(int mat_type)
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

/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
void Cam2Image::convert_frame_to_message(
  const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image & msg)
{
  // copy cv information into ros message
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = std::to_string(frame_id);
}

/// Constructor initializes default demo parameters  
Cam2Image::Cam2Image(rclcpp::NodeOptions options) : Node("cam2image", options){
  // Initialize default demo parameters
  show_camera = false;
  depth = rmw_qos_profile_default.depth;
  freq = 20.0;
  reliability_policy = rmw_qos_profile_default.reliability;
  history_policy = rmw_qos_profile_default.history;
  width = 200;
  height = 120;
  burger_mode = true;
  topic = "image";

}

/// Read in and parse command line arguments.
/**
 * \param[in] argc 
 * \param[in] argv 
 * \return A bool whether command line options were valid or not 
 */
bool Cam2Image::setup(int argc, char ** argv){
  if (!parse_command_options(
        argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &freq, &width,
        &height, &burger_mode, &topic))
  {
      return false;
  }

  return true;

}


/// Execute main functions of component 
/**
 * publish camera feed or burger image to "image" topic 
 */
void 
Cam2Image::execute()
{
  rclcpp::Logger node_logger = this->get_logger();

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      history_policy,
      depth));
  
  qos.reliability(reliability_policy);
  RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());
  pub_ = create_publisher<sensor_msgs::msg::Image>(topic, qos);

  // is_flipped will cause the incoming camera image message to flip about the y-axis.
  bool is_flipped = false;

  auto callback = [&is_flipped, &node_logger](const std_msgs::msg::Bool::SharedPtr msg) -> void
  {
    is_flipped = msg->data;
    RCLCPP_INFO(node_logger, "Set flip mode to: %s", is_flipped ? "on" : "off");
  };

  sub_ = create_subscription<std_msgs::msg::Bool>(
    "flip_image", rclcpp::SensorDataQoS(), callback);

  // Set a loop rate for our main event loop.
  rclcpp::WallRate loop_rate(freq);

  cv::VideoCapture cap;
  burger::Burger burger_cap;
  if (!burger_mode) {
    // Initialize OpenCV video capture stream.
    // Always open device 0.
    cap.open(0);

    // Set the width and height based on command line arguments.
    cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if (!cap.isOpened()) {
      RCLCPP_ERROR(node_logger, "Could not open video stream");
      // return 1;
    }
  }

  // Initialize OpenCV image matrices.
  cv::Mat frame;
  cv::Mat flipped_frame;

  size_t i = 1;
  while (rclcpp::ok()) {
    // Initialize a shared pointer to an Image message.
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;
    // Get the frame from the video capture.
    if (burger_mode) {
      frame = burger_cap.render_burger(width, height);
    } else {
      cap >> frame;
    }
    // Check if the frame was grabbed correctly
    if (!frame.empty()) {
      // Convert to a ROS image
      if (!is_flipped) {
        convert_frame_to_message(frame, i, *msg);
      } else {
        // Flip the frame if needed
        cv::flip(frame, flipped_frame, 1);
        convert_frame_to_message(flipped_frame, i, *msg);
      }
      if (show_camera) {
        cv::Mat cvframe = frame;
        // Show the image in a window called "cam2image".
        cv::imshow("cam2image", cvframe);
        // Draw the image to the screen and wait 1 millisecond.
        cv::waitKey(1);
      }
      // Publish the image message and increment the frame_id.
      RCLCPP_INFO(node_logger, "Publishing image #%zd", i);
      pub_->publish(std::move(msg));
      ++i;
    }
    // Do some work in rclcpp and wait for more to come in.
    // rclcpp::spin_some(this);
    loop_rate.sleep();
  }
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(image_tools::Cam2Image)
