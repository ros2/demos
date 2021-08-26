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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{
class ListenerBestEffort : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ListenerBestEffort(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
      };

    sub_ = create_subscription<std_msgs::msg::String>("chatter", rclcpp::SensorDataQoS(), callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ListenerBestEffort)
