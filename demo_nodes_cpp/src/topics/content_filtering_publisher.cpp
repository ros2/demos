// Copyright 2022 Open Source Robotics Foundation, Inc.
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
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/float32.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{
// The simulated temperature data starts from -100.0 and ends at 150.0 with a step size of 10.0
constexpr std::array<float, 3> TEMPERATURE_SETTING {-100.0f, 150.0f, 10.0f};

// Create a ContentFilteringPublisher class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class ContentFilteringPublisher : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ContentFilteringPublisher(const rclcpp::NodeOptions & options)
  : Node("content_filtering_publisher", options)
  {
    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::Float32>();
        msg_->data = temperature_;
        temperature_ += TEMPERATURE_SETTING[2];
        if (temperature_ > TEMPERATURE_SETTING[1]) {
          temperature_ = TEMPERATURE_SETTING[0];
        }
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg_->data);
        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg_));
      };
    // Create a publisher with a custom Quality of Service profile.
    // Uniform initialization is suggested so it can be trivially changed to
    // rclcpp::KeepAll{} if the user wishes.
    // (rclcpp::KeepLast(7) -> rclcpp::KeepAll() fails to compile)
    rclcpp::QoS qos(rclcpp::KeepLast{7});
    pub_ = this->create_publisher<std_msgs::msg::Float32>("temperature", qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  float temperature_ = TEMPERATURE_SETTING[0];
  std::unique_ptr<std_msgs::msg::Float32> msg_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ContentFilteringPublisher)
