#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include "image_tools/options.hpp"

#include "./burger.hpp"

std::string
mat_type2encoding(int mat_type)
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

void convert_frame_to_message(
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

class ClearbotCam2image : public rclcpp::Node
{
  std::string topic("image");
  size_t width = 640;
  size_t height = 480;
  double freq = 30.0;

  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      history_policy,
      depth));
  qos.reliability(reliability_policy);
  
  public:
  explicit ClearbotCam2image()
  : Node("clearbot_cam2image")
  {
    rclcpp::Logger node_logger = this->get_logger();
    image_publisher = this->create_publisher<sensor_msgs::msg::Image>(topic, qos);
    timer_ = this->create_wall_timer(33ms, std::bind(ClearbotCam2image::timer_callback, this));
    cv::VideoCapture cap;
    cv::Mat frame;
    cap.open(2, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if (!cap.isOpened()){
      RCLCPP_ERROR(node_logger, "Could not open video stream");
      return 1;
    }
    RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());
      
  }
  private:
    void timer_callback()
    {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->is_bigendian = false;
      cap >> frame;
      if (!frame.empty()) {
        
        convert_frame_to_message(frame, i, *msg);
        RCLCPP_INFO(node_logger, "Publishing image #%zd", i);
        pub->publish(std::move(msg));
        ++i;
      }
    }
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_subscription_;

}

int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClearbotCam2image>());
  rclcpp::shutdown();
  return 0;
}
