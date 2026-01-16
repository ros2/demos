// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include "composition/lifecycle_talker_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

namespace composition
{

LifecycleTalker::LifecycleTalker(const rclcpp::NodeOptions & options)
: LifecycleNode("lifecycle_talker", options), count_(0)
{
}

/// Callback for walltimer in order to publish the message.
/**
  * Callback for walltimer. This function gets invoked by the timer
  * and executes the publishing.
  * For this demo, we ask the node for its current state. If the
  * lifecycle publisher is not activate, we still invoke publish, but
  * the communication is blocked so that no messages is actually transferred.
  */
void LifecycleTalker::publish()
{
  auto msg = std::make_unique<example_interfaces::msg::String>();
  msg->data = "Lifecycle Hello World: " + std::to_string(++count_);

  // Print the current state for demo purposes
  RCLCPP_INFO_EXPRESSION(this->get_logger(), pub_->is_activated(),
      "Lifecycle Publisher is active. Publishing: '%s'", msg->data.c_str());
  RCLCPP_INFO_EXPRESSION(this->get_logger(), !pub_->is_activated(),
      "Lifecycle Publisher is inactive. Not publishing: '%s'", msg->data.c_str());
  std::flush(std::cout);

  // We independently from the current state call publish on the lifecycle publisher.
  // Only if the publisher is in an active state, the message transfer is
  // enabled and the message actually published.
  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  pub_->publish(std::move(msg));
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleTalker::on_configure(const rclcpp_lifecycle::State &)
{
  pub_ = create_publisher<example_interfaces::msg::String>("lc_chatter", 10);
  timer_ = create_wall_timer(1s, [this](){return this->publish();});

  RCLCPP_INFO(get_logger(), "on_configure() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleTalker::on_cleanup(const rclcpp_lifecycle::State &)
{
  // In our cleanup phase, we release the shared pointers to the timer and publisher.
  // These entities are no longer available and our node is "clean".
  timer_.reset();
  pub_.reset();

  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleTalker::on_shutdown(const rclcpp_lifecycle::State & state)
{
  // In our cleanup phase, we release the shared pointers to the timer and publisher.
  // These entities are no longer available and our node is "clean".
  timer_.reset();
  pub_.reset();

  RCLCPP_INFO(get_logger(), "on_shutdown() is called from state %s.", state.label().c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::LifecycleTalker)
