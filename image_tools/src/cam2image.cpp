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

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"

#include "image_tools/cv_mat_sensor_msgs_image_type_adapter.hpp"
#include "image_tools/visibility_control.h"

#include "./burger.hpp"
#include "./policy_maps.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  image_tools::ROSCvMatContainer,
  sensor_msgs::msg::Image);

namespace image_tools
{
class Cam2Image : public rclcpp::Node
{
public:
  IMAGE_TOOLS_PUBLIC
  explicit Cam2Image(const rclcpp::NodeOptions & options)
  : Node("cam2image", options),
    is_flipped_(false),
    publish_number_(1u)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Do not execute if a --help option was provided
    if (help(options.arguments())) {
      // TODO(jacobperron): Replace with a mechanism for a node to "unload" itself
      // from a container.
      exit(0);
    }
    parse_parameters();
    initialize();
  }

private:
  IMAGE_TOOLS_LOCAL
  void initialize()
  {
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
    pub_ = create_publisher<image_tools::ROSCvMatContainer>("image", qos);

    // Subscribe to a message that will toggle flipping or not flipping, and manage the state in a
    // callback
    auto callback = [this](std_msgs::msg::Bool::ConstSharedPtr msg) -> void
      {
        this->is_flipped_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Set flip mode to: %s", this->is_flipped_ ? "on" : "off");
      };
    // Set the QoS profile for the subscription to the flip message.
    sub_ = create_subscription<std_msgs::msg::Bool>(
      "flip_image", rclcpp::SensorDataQoS(), callback);

    if (!burger_mode_) {
      // Initialize OpenCV video capture stream.
      cap.open(device_id_);

      // Set the width and height based on command line arguments.
      cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
      if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        throw std::runtime_error("Could not open video stream");
      }
    }

    // Start main timer loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
      std::bind(&Cam2Image::timerCallback, this));
  }

  /// Publish camera, or burger, image.
  IMAGE_TOOLS_LOCAL
  void timerCallback()
  {
    cv::Mat frame;

    // Initialize a shared pointer to an Image message.
    // Get the frame from the video capture.
    if (burger_mode_) {
      frame = burger_cap.render_burger(width_, height_);
    } else {
      cap >> frame;
    }

    // If no frame was grabbed, return early
    if (frame.empty()) {
      return;
    }

    // Conditionally flip the image
    if (is_flipped_) {
      cv::flip(frame, frame, 1);
    }

    // Conditionally show image
    if (show_camera_) {
      // Show the image in a window called "cam2image".
      cv::imshow("cam2image", frame);
      // Draw the image to the screen and wait 1 millisecond.
      cv::waitKey(1);
    }

    std_msgs::msg::Header header;
    header.frame_id = frame_id_;
    header.stamp = this->now();
    image_tools::ROSCvMatContainer container(frame, header);

    // Publish the image message and increment the publish_number_.
    RCLCPP_INFO(get_logger(), "Publishing image #%zd", publish_number_++);
    pub_->publish(std::move(container));
  }

  IMAGE_TOOLS_LOCAL
  bool help(const std::vector<std::string> & args)
  {
    if (std::find(args.begin(), args.end(), "--help") != args.end() ||
      std::find(args.begin(), args.end(), "-h") != args.end())
    {
      std::stringstream ss;
      ss << "Usage: cam2image [-h] [--ros-args [-p param:=value] ...]" << std::endl;
      ss << "Publish images from a camera stream." << std::endl;
      ss << "Example: ros2 run image_tools cam2image --ros-args -p reliability:=best_effort";
      ss << std::endl << std::endl;
      ss << "Options:" << std::endl;
      ss << "  -h, --help\tDisplay this help message and exit";
      ss << std::endl << std::endl;
      ss << "Parameters:" << std::endl;
      ss << "  reliability\tReliability QoS setting. Either 'reliable' (default) or 'best_effort'";
      ss << std::endl;
      ss << "  history\tHistory QoS setting. Either 'keep_last' (default) or 'keep_all'.";
      ss << std::endl;
      ss << "\t\tIf 'keep_last', then up to N samples are stored where N is the depth";
      ss << std::endl;
      ss << "  depth\t\tDepth of the publisher queue. Only honored if history QoS is 'keep_last'.";
      ss << " Default value is 10";
      ss << std::endl;
      ss << "  frequency\tPublish frequency in Hz. Default value is 30";
      ss << std::endl;
      ss << "  burger_mode\tProduce images of burgers rather than connecting to a camera";
      ss << std::endl;
      ss << "  show_camera\tShow camera stream. Either 'true' or 'false' (default)";
      ss << std::endl;
      ss << "  device_id\tDevice ID of the camera. 0 (default) selects the default camera device.";
      ss << std::endl;
      ss << "  width\t\tWidth component of the camera stream resolution. Default value is 320";
      ss << std::endl;
      ss << "  height\tHeight component of the camera stream resolution. Default value is 240";
      ss << std::endl;
      ss << "  frame_id\t\tID of the sensor frame. Default value is 'camera_frame'";
      ss << std::endl << std::endl;
      ss << "Note: try running v4l2-ctl --list-formats-ext to obtain a list of valid values.";
      ss << std::endl;
      std::cout << ss.str();
      return true;
    }
    return false;
  }

  IMAGE_TOOLS_LOCAL
  void parse_parameters()
  {
    // Parse 'reliability' parameter
    rcl_interfaces::msg::ParameterDescriptor reliability_desc;
    reliability_desc.description = "Reliability QoS setting for the image publisher";
    reliability_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_reliability_policy_map) {
      reliability_desc.additional_constraints += entry.first + " ";
    }
    const std::string reliability_param = this->declare_parameter(
      "reliability", "reliable", reliability_desc);
    auto reliability = name_to_reliability_policy_map.find(reliability_param);
    if (reliability == name_to_reliability_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS reliability setting '" << reliability_param << "'";
      throw std::runtime_error(oss.str());
    }
    reliability_policy_ = reliability->second;

    // Parse 'history' parameter
    rcl_interfaces::msg::ParameterDescriptor history_desc;
    history_desc.description = "History QoS setting for the image publisher";
    history_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_history_policy_map) {
      history_desc.additional_constraints += entry.first + " ";
    }
    const std::string history_param = this->declare_parameter(
      "history", name_to_history_policy_map.begin()->first, history_desc);
    auto history = name_to_history_policy_map.find(history_param);
    if (history == name_to_history_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS history setting '" << history_param << "'";
      throw std::runtime_error(oss.str());
    }
    history_policy_ = history->second;

    // Declare and get remaining parameters
    depth_ = this->declare_parameter("depth", 10);
    freq_ = this->declare_parameter("frequency", 30.0);
    show_camera_ = this->declare_parameter("show_camera", false);
    device_id_ = static_cast<int>(this->declare_parameter("device_id", 0));
    width_ = this->declare_parameter("width", 320);
    height_ = this->declare_parameter("height", 240);
    rcl_interfaces::msg::ParameterDescriptor burger_mode_desc;
    burger_mode_desc.description = "Produce images of burgers rather than connecting to a camera";
    burger_mode_ = this->declare_parameter("burger_mode", false, burger_mode_desc);
    frame_id_ = this->declare_parameter("frame_id", "camera_frame");
  }

  cv::VideoCapture cap;
  burger::Burger burger_cap;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::Publisher<image_tools::ROSCvMatContainer>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS parameters
  bool show_camera_;
  size_t depth_;
  double freq_;
  rmw_qos_reliability_policy_t reliability_policy_;
  rmw_qos_history_policy_t history_policy_;
  size_t width_;
  size_t height_;
  bool burger_mode_;
  std::string frame_id_;
  int device_id_;

  /// If true, will cause the incoming camera image message to flip about the y-axis.
  bool is_flipped_;
  /// The number of images published.
  size_t publish_number_;
};

}  // namespace image_tools

RCLCPP_COMPONENTS_REGISTER_NODE(image_tools::Cam2Image)
