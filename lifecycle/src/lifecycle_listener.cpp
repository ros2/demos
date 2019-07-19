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

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

/// LifecycleListener class as a simple listener node
/**
 * We subscribe to two topics
 * - lifecycle_chatter: The data topic from the talker
 * - lc_talker__transition_event: The topic publishing
 *   notifications about state changes of the node
 *   lc_talker
 */
namespace lifecycle
{
class LifecycleListener : public rclcpp::Node
{
public:
  explicit LifecycleListener(const rclcpp::NodeOptions & options)
  : Node("lc_listener", options)
  {
    // Data topic from the lc_talker node
    sub_data_ = this->create_subscription<std_msgs::msg::String>("lifecycle_chatter", 10,
        std::bind(&LifecycleListener::data_callback, this, std::placeholders::_1));

    // Notification event topic. All state changes
    // are published here as TransitionEvents with
    // a start and goal state indicating the transition
    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/lc_talker/transition_event",
      10,
      std::bind(&LifecycleListener::notification_callback, this, std::placeholders::_1));
  }

  void data_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "data_callback: %s", msg->data.c_str());
  }

  void notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "notify callback: Transition from state %s to %s",
      msg->start_state.label.c_str(), msg->goal_state.label.c_str());
  }

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;
  std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>>
  sub_notification_;
};

}  // namespace lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle::LifecycleListener)
