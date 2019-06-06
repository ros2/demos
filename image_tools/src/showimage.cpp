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
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "image_tools/options.hpp"

#include "image_tools/showimage.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"



namespace image_tools
{
/// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
/**
 * \param[in] encoding A string representing the encoding type.
 * \return The OpenCV encoding type.
 */
int
ShowImage::encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

/// Convert the ROS Image message to an OpenCV matrix and display it to the user.
// \param[in] msg The image message to show.
void ShowImage::show_image(
  const sensor_msgs::msg::Image::SharedPtr msg, bool show_camera, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
  std::cerr << "Received image #" << msg->header.frame_id.c_str() << std::endl;

  if (show_camera) {
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(
      msg->height, msg->width, encoding2mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step);

    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    cv::Mat cvframe = frame;

    // Show the image in a window called "showimage".
    cv::imshow("showimage", cvframe);
    // Draw the screen and wait for 1 millisecond.
    cv::waitKey(1);
  }
}

/// Default constructor for no command line args
/**
 */
ShowImage::ShowImage(rclcpp::NodeOptions options)
: Node("showimage", options)
{
  execute();
}

/// Constructor for setup with command line args
ShowImage::ShowImage(rclcpp::NodeOptions options, int argc, char ** argv)
: Node("showimage", options){
  if(setup(argc, argv)){
    execute();
  }else{
    rclcpp::shutdown();
  }
}

/// Execute main functions with image subscriber and callback
/**
 */
void ShowImage::execute(){
  if (show_camera_) {
    // Initialize an OpenCV named window called "showimage".
      // RCLCPP_INFO(this->get_logger(), "SHOWING THE CAMERA");

    cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
    cv::waitKey(1);
  }

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      history_policy_,
      depth_
  ));
  qos.reliability(reliability_policy_);
  auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      show_image(msg, show_camera_, this->get_logger());
    };

  std::cerr << "Subscribing to topic '" << topic_ << "'" << std::endl;
  RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", topic_.c_str());
  sub_ = create_subscription<sensor_msgs::msg::Image>(topic_, qos, callback);
}


/// Read in and parse command line arguments.
/**
 * \param[in] argc 
 * \param[in] argv 
 * \return A bool whether command line options were valid or not 
 */
bool ShowImage::setup(int argc, char ** argv){
  if (!parse_command_options(
      argc, argv,  &depth_, &reliability_policy_, &history_policy_, &show_camera_, nullptr, nullptr,
      nullptr, nullptr, &topic_))
  {
    return false;
  }

  return true;
}


}  // namespace image_tools

#include "rclcpp_components/register_node_macro.hpp"


RCLCPP_COMPONENTS_REGISTER_NODE(image_tools::ShowImage)
