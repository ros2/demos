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

#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "image_tools/options.hpp"

/// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
/**
 * \param[in] encoding A string representing the encoding type.
 * \return The OpenCV encoding type.
 */
int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

/// Convert the ROS Image message to an OpenCV matrix and display it to the user.
// \param[in] msg The image message to show.
void show_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::stringstream ss;
  ss << "Received image #" << msg->header.frame_id << std::endl;
  std::cout << ss.str();

  // Convert to an OpenCV matrix by assigning the data.
  cv::Mat frame(
    msg->height, msg->width, encoding2mat_type(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  // NOTE(esteve): Use C version of cvShowImage to avoid this on Windows:
  // http://stackoverflow.com/questions/20854682/opencv-multiple-unwanted-window-with-garbage-name
  // Show the image in a window called "showimage".
  CvMat cvframe = frame;
  cvShowImage("showimage", &cvframe);
  // Draw the screen and wait for 1 millisecond.
  cv::waitKey(1);
}

int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default demo parameters
  size_t depth = 10;
  rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABLE;
  rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_KEEP_ALL_HISTORY;

  // Configure demo parameters with command line options.
  if (!parse_command_options(argc, argv, &depth, &reliability_policy, &history_policy)) {
    return 0;
  }

  // Initialize an OpenCV named window called "showimage".
  cvNamedWindow("showimage", CV_WINDOW_AUTOSIZE);

  // Initialize a ROS node.
  auto node = rclcpp::node::Node::make_shared("showimage");

  // Set quality of service profile based on command line options.
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  // The history policy determines how messages are saved until the message is taken by the reader.
  // KEEP_ALL saves all messages until they are taken.
  // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
  // parameter.
  custom_qos_profile.history = history_policy;

  // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
  custom_qos_profile.depth = depth;

  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  custom_qos_profile.reliability = reliability_policy;

  // Initialize a subscriber that will receive the ROS Image message to be displayed.
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    "image", show_image, custom_qos_profile);

  rclcpp::spin(node);

  return 0;
}
