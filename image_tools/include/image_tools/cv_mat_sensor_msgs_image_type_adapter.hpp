// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef IMAGE_TOOLS__CV_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_
#define IMAGE_TOOLS__CV_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_

#include <variant>

#include "opencv2/opencv.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_tools/visibility_control.h"

namespace image_tools
{

/// A potentially owning, potentially non-owning, container of a cv::Mat and ROS header.
/**
 * The two main use cases for this are publishing user controlled data, and
 * recieving data from the middleware that may have been a ROS message
 * originally or may have been an cv::Mat originally.
 *
 * In the first case, publishing user owned data, the user will want to provide
 * their own cv::Mat.
 * The cv::Mat may own the data or it may not, so in the latter case, it is up
 * to the user to ensure the data the cv::Mat points to remains valid as long
 * as the middleware needs it.
 *
 * In the second case, receiving data from the middleware, the middleware will
 * either give a new ROSCvMatContainer which owns a sensor_msgs::msg::Image or
 * it will give a ROSCvMatContainer that was previously published by the user
 * (in the case of intra-process communication).
 * If the container owns the sensor_msgs::msg::Image, then the cv::Mat will just
 * reference data field of this message, so the container needs to be kept.
 * If the container was published by the user it may or may not own the data
 * and the cv::Mat it contains may or may not own the data.
 *
 * For these reasons, it is advisable to use cv::Mat::clone() if you intend to
 * copy the cv::Mat and let this container go.
 *
 * For more details about the ownership behavior of cv::Mat see documentation
 * for these methods of cv::Mat:
 *
 *   - template<typename _Tp > cv::Mat::Mat(const std::vector<_Tp> &, bool)
 *   - Mat & cv::Mat::operator=(const Mat &)
 *   - void cv::Mat::addref()
 *   - void cv::Mat::release()
 *
 */
class ROSCvMatContainer
{
public:
  using SensorMsgsImageStorageType = std::variant<
    nullptr_t,
    std::unique_ptr<sensor_msgs::msg::Image>,
    std::shared_ptr<sensor_msgs::msg::Image>
  >;

  /// Store an owning pointer to a sensor_msg::msg::Image, and create a cv::Mat that references it.
  IMAGE_TOOLS_PUBLIC
  explicit ROSCvMatContainer(SensorMsgsImageStorageType storage);

  /// Shallow copy the given cv::Mat into this class, but do not own the data directly.
  IMAGE_TOOLS_PUBLIC
  ROSCvMatContainer(const cv::Mat & mat_frame, const std_msgs::msg::Header & header);

  /// Move the given cv::Mat into this class.
  IMAGE_TOOLS_PUBLIC
  explicit ROSCvMatContainer(cv::Mat && mat_frame, const std_msgs::msg::Header & header);

  /// Copy the sensor_msgs::msg::Image into this contain and create a cv::Mat that references it.
  IMAGE_TOOLS_PUBLIC
  explicit ROSCvMatContainer(const sensor_msgs::msg::Image & mat_frame);

  /// Return true if this class owns the data the cv_mat references.
  /**
   * Note that this does not check if the cv::Mat owns its own data, only if
   * this class owns a sensor_msgs::msg::Image that the cv::Mat references.
   */
  IMAGE_TOOLS_PUBLIC
  bool
  is_owning() const;

  /// Const access the cv::Mat in this class.
  IMAGE_TOOLS_PUBLIC
  const cv::Mat &
  cv_mat() const;

  /// Get a shallow copy of the cv::Mat that is in this class.
  /**
   * Note that if you want to let this container go out of scope you should
   * make a deep copy with cv::Mat::clone() beforehand.
   */
  IMAGE_TOOLS_PUBLIC
  cv::Mat
  cv_mat();

  /// Const access the ROS Header.
  IMAGE_TOOLS_PUBLIC
  const std_msgs::msg::Header &
  header() const;

  /// Access the ROS Header.
  IMAGE_TOOLS_PUBLIC
  std_msgs::msg::Header &
  header();

  /// Const access the variant for potentially owning the sensor_msgs::msg::Image.s
  IMAGE_TOOLS_PUBLIC
  const SensorMsgsImageStorageType &
  sensor_msgs_image_storage() const;

  /// Access the variant for potentially owning the sensor_msgs::msg::Image.
  IMAGE_TOOLS_PUBLIC
  SensorMsgsImageStorageType &
  sensor_msgs_image_storage();

private:
  SensorMsgsImageStorageType storage_;
  std_msgs::msg::Header header_;
  cv::Mat frame_;
};

}  // namespace image_tools

template<>
struct rclcpp::TypeAdapter<image_tools::ROSCvMatContainer, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = image_tools::ROSCvMatContainer;
  using ros_message_type = sensor_msgs::msg::Image;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.height = source.cv_mat().rows;
    destination.width = source.cv_mat().cols;
    switch (source.cv_mat().type()) {
      case CV_8UC1:
        destination.encoding = "mono8";
        break;
      case CV_8UC3:
        destination.encoding = "bgr8";
        break;
      case CV_16SC1:
        destination.encoding = "mono16";
        break;
      case CV_8UC4:
        destination.encoding = "rgba8";
        break;
      default:
        throw std::runtime_error("unsupported encoding type");
    }
    destination.step = static_cast<sensor_msgs::msg::Image::_step_type>(source.cv_mat().step);
    size_t size = source.cv_mat().step * source.cv_mat().rows;
    destination.data.resize(size);
    memcpy(&destination.data[0], source.cv_mat().data, size);
    destination.header = source.header();
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = image_tools::ROSCvMatContainer(source);
  }
};

#endif  // IMAGE_TOOLS__CV_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_
