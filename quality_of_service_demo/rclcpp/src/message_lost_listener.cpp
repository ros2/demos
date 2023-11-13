// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "quality_of_service_demo/visibility_control.h"

namespace quality_of_service_demo
{
class MessageLostListener : public rclcpp::Node
{
public:
  QUALITY_OF_SERVICE_DEMO_PUBLIC
  explicit MessageLostListener(const rclcpp::NodeOptions & options)
  : Node("MessageLostListener", options)
  {
    // Callback that will be used when a message is received.
    auto callback =
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) -> void
      {
        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Time stamp_time(msg->header.stamp, RCL_ROS_TIME);
        auto diff = now - stamp_time;
        RCLCPP_INFO(
          this->get_logger(),
          "I heard an image. Message single trip latency: [%f]",
          diff.seconds());
      };
    rclcpp::SubscriptionOptions sub_opts;
    // Update the subscription options, so an event handler that will report the number of lost
    // messages is created together with the subscription.
    sub_opts.event_callbacks.message_lost_callback =
      [logger = this->get_logger()](rclcpp::QOSMessageLostInfo & info)
      {
        RCLCPP_INFO_STREAM(
          logger,
          "Some messages were lost:\n>\tNumber of new lost messages: " <<
            info.total_count_change << " \n>\tTotal number of messages lost: " <<
            info.total_count);
      };
    // Create the subscription. This will also create an event handler based on the above
    // subscription options.
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "message_lost_chatter", 1, callback, sub_opts);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace quality_of_service_demo

RCLCPP_COMPONENTS_REGISTER_NODE(quality_of_service_demo::MessageLostListener)
