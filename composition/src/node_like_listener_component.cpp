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

#include "composition/node_like_listener_component.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

// Create a Listener "component" that does not subclass the generic rclcpp::Node base class.
/**
 * Note that "components" don't have to derive from rclcpp::Node.
 * In the case that an object does not inherit from rclcpp::Node, then it must implement:
 * - Constructor that takes `const rclcpp::NodeOptions&`
 * - get_node_base_interface() which returns a NodeBaseInterface::SharedPtr
 *
 * This is an example of an object that implements the interface required to be a component.
 */
NodeLikeListener::NodeLikeListener(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("listener", options))
{
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.
  auto callback =
    [this](const typename std_msgs::msg::String::SharedPtr msg) -> void
    {
      RCLCPP_INFO(this->node_->get_logger(), "I heard: [%s]", msg->data.c_str());
      std::flush(std::cout);
    };

  // Create a subscription to the "chatter" topic which can be matched with one or more
  // compatible ROS publishers.
  // Note that not all publishers on the same topic with the same type will be compatible:
  // they must have compatible Quality of Service policies.
  sub_ = this->node_->create_subscription<std_msgs::msg::String>("chatter", 10, callback);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
NodeLikeListener::get_node_base_interface() const
{
  return this->node_->get_node_base_interface();
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::NodeLikeListener)
