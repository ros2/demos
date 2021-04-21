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
#ifndef LIFECYCLE__LIFECYCLE_LISTENER_HPP_
#define LIFECYCLE__LIFECYCLE_LISTENER_HPP_

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

/// LifecycleListener class as a simple listener node
/**
 * We subscribe to two topics
 * - lifecycle_chatter: The data topic from the talker
 * - lc_talker__transition_event: The topic publishing
 *   notifications about state changes of the node
 *   lc_talker
 */
class LifecycleListener : public rclcpp::Node
{
public:
  explicit LifecycleListener(const std::string & node_name);

  void data_callback(const std_msgs::msg::String::SharedPtr msg);

  void notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;
  std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>>
  sub_notification_;
};

#endif  // LIFECYCLE__LIFECYCLE_LISTENER_HPP_
