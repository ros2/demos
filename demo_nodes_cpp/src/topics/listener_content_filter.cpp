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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class ListenerContentFilter : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ListenerContentFilter(const rclcpp::NodeOptions & options)
  : Node("content_filter_listener", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Create a callback function for when messages are received.
    auto callback =
      [this](const std_msgs::msg::String & msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
      };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.

    // Initialize a subscription with a content filter to receive messages that are
    // 'Hello World: 10'.
    rclcpp::SubscriptionOptions sub_options;
    sub_options.content_filter_options.filter_expression = "data = %0";
    sub_options.content_filter_options.expression_parameters = {"'Hello World: 10'"};

    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback, sub_options);

    if (!sub_->is_cft_enabled()) {
      RCLCPP_WARN(
        this->get_logger(), "Content filter is not enabled since it's not supported");
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ListenerContentFilter)
