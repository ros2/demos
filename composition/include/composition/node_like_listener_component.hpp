// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef COMPOSITION__NODE_LIKE_LISTENER_COMPONENT_HPP_
#define COMPOSITION__NODE_LIKE_LISTENER_COMPONENT_HPP_

#include "composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

class NodeLikeListener
{
public:
  COMPOSITION_PUBLIC
  explicit NodeLikeListener(const rclcpp::NodeOptions & options);

  COMPOSITION_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace composition

#endif  // COMPOSITION__NODE_LIKE_LISTENER_COMPONENT_HPP_
