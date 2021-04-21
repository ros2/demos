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
#include "lifecycle/lifecycle_listener.hpp"

#include <memory>
#include <string>

LifecycleListener::LifecycleListener(const std::string & node_name)
: Node(node_name)
{
  // Data topic from the lc_talker node
  sub_data_ = this->create_subscription<std_msgs::msg::String>(
    "lifecycle_chatter", 10,
    std::bind(&LifecycleListener::data_callback, this, std::placeholders::_1));

  // Notification event topic. All state changes
  // are published here as TransitionEvents with
  // a start and goal state indicating the transition
  sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    "/lc_talker/transition_event",
    10,
    std::bind(&LifecycleListener::notification_callback, this, std::placeholders::_1));
}

void LifecycleListener::data_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "data_callback: %s", msg->data.c_str());
}

void LifecycleListener::notification_callback(
  const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "notify callback: Transition from state %s to %s",
    msg->start_state.label.c_str(), msg->goal_state.label.c_str());
}

int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto lc_listener = std::make_shared<LifecycleListener>("lc_listener");
  rclcpp::spin(lc_listener);

  rclcpp::shutdown();

  return 0;
}
