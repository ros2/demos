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

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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
void show_image(
  const sensor_msgs::msg::Image::SharedPtr msg, bool show_camera, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
  std::cerr << "Received image #" << msg->header.frame_id.c_str() << std::endl;

  if (show_camera) {
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(
      msg->height, msg->width, encoding2mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step);

    CvMat cvframe;
    if (msg->encoding == "rgb8") {
      cv::Mat frame2;
      cv::cvtColor(frame, frame2, cv::COLOR_RGB2BGR);
      cvframe = frame2;
    } else {
      cvframe = frame;
    }

    // NOTE(esteve): Use C version of cvShowImage to avoid this on Windows:
    // http://stackoverflow.com/q/20854682
    // Show the image in a window called "showimage".
    cvShowImage("showimage", &cvframe);
    // Draw the screen and wait for 1 millisecond.
    cv::waitKey(1);
  }
}

int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  std::cerr << "Right after init" << std::endl;

  // Initialize default demo parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  bool show_camera = true;
  std::string topic("image");

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Configure demo parameters with command line options.
  if (!parse_command_options(
      argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, nullptr, nullptr,
      nullptr, nullptr, &topic))
  {
    return 0;
  }

  if (show_camera) {
    std::cerr << "Creating window" << std::endl;
    // Initialize an OpenCV named window called "showimage".
    cvNamedWindow("showimage", CV_WINDOW_AUTOSIZE);
    cv::waitKey(1);
    std::cerr << "After creating window" << std::endl;
  }

  // Initialize a ROS node.
  auto node = rclcpp::Node::make_shared("showimage");

  std::cerr << "AFter creating node" << std::endl;

  // Set quality of service profile based on command line options.
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
  custom_qos_profile.depth = depth;

  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  custom_qos_profile.reliability = reliability_policy;

  // The history policy determines how messages are saved until the message is taken by the reader.
  // KEEP_ALL saves all messages until they are taken.
  // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
  // parameter.
  custom_qos_profile.history = history_policy;

  std::cerr << "Right before defining callback" << std::endl;
  auto callback = [show_camera, &node](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      show_image(msg, show_camera, node->get_logger());
    };

  std::cerr << "Subscribing to topic '" << topic << "'" << std::endl;
  RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic.c_str());
  // Initialize a subscriber that will receive the ROS Image message to be displayed.
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    topic, callback, custom_qos_profile);

  std::cerr << "Spinning" << std::endl;
  rclcpp::spin(node);

  std::cerr << "Shutdown" << std::endl;
  rclcpp::shutdown();

  return 0;
}
