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
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_tools/options.hpp"
#include "image_tools/visibility_control.h"
namespace image_tools
{
class ShowImage : public rclcpp::Node
{
public:
  IMAGE_TOOLS_PUBLIC
  explicit ShowImage(const rclcpp::NodeOptions & options)
  : Node("showimage", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    std::vector<std::string> args = options.arguments();
    if (setup(args)) {
      execute();
    } else {
      rclcpp::shutdown();
    }
  }

  /// Read in and parse command line arguments.
  /**
   * \param[in] argc
   * \param[in] argv
   * \return A bool whether command line options were valid or not
   */
  IMAGE_TOOLS_PUBLIC
  bool setup(std::vector<std::string> args)
  {
    return parse_command_options(
      args, &depth_, &reliability_policy_, &history_policy_, &show_camera_, nullptr, nullptr,
      nullptr, nullptr, &topic_);
  }

  IMAGE_TOOLS_PUBLIC
  void execute()
  {
    if (show_camera_) {
      // Initialize an OpenCV named window called "showimage".
      cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
      cv::waitKey(1);
    }
    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        history_policy_,
        // Depth represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        depth_
    ));
    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos.reliability(reliability_policy_);
    auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg)
      {
        show_image(msg, show_camera_, this->get_logger());
      };

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", topic_.c_str());
    sub_ = create_subscription<sensor_msgs::msg::Image>(topic_, qos, callback);
  }

  /// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
  /**
   * \param[in] encoding A string representing the encoding type.
   * \return The OpenCV encoding type.
   */
  IMAGE_TOOLS_PUBLIC
  int encoding2mat_type(const std::string & encoding)
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
  IMAGE_TOOLS_PUBLIC
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

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  bool show_camera_ = true;
  std::string topic_ = "image";
};

}  // namespace image_tools

RCLCPP_COMPONENTS_REGISTER_NODE(image_tools::ShowImage)
