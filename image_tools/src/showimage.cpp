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

/// Constructor that initializes the default demo parameters
/**
 */
ShowImage::ShowImage(rclcpp::NodeOptions options)
: Node("showimage", options)
{
  // Initialize default demo parameters
  depth = rmw_qos_profile_default.depth;
  reliability_policy = rmw_qos_profile_default.reliability;
  history_policy = rmw_qos_profile_default.history;
  show_camera = true;
  topic = "image";
}

/// Execute main functions with image subscriber and callback 
/**
 */
void ShowImage::execute(){
if (show_camera) {
    // Initialize an OpenCV named window called "showimage".
    cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
    cv::waitKey(10);
  }

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      history_policy,
      depth
  ));
  qos.reliability(reliability_policy);
  auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      show_image(msg, show_camera, this->get_logger());
    };

  std::cerr << "Subscribing to topic '" << topic << "'" << std::endl;
  RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", topic.c_str());
  sub_ = create_subscription<sensor_msgs::msg::Image>(topic, qos, callback);

}


/// Read in and parse command line arguments.
/**
 * \param[in] argc 
 * \param[in] argv 
 * \return A bool whether command line options were valid or not 
 */
bool ShowImage::setup(int argc, char ** argv){
  if (!parse_command_options(
      argc, argv,  &depth, &reliability_policy, &history_policy, &show_camera, nullptr, nullptr,
      nullptr, nullptr, &topic))
  {

    return false;
  }

  return true;
}


}

#include "rclcpp_components/register_node_macro.hpp"


RCLCPP_COMPONENTS_REGISTER_NODE(image_tools::ShowImage)
