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

#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32.hpp"

#include "intra_process_demo/two_node_pipeline/consumer_component.hpp"
#include "intra_process_demo/visibility_control.h"

namespace intra_process_demo
{
namespace two_node_pipeline
{

Consumer::Consumer(const rclcpp::NodeOptions & options)
: Consumer("consumer", "number", options) {}

Consumer::Consumer(
  const std::string & name,
  const std::string & input,
  const rclcpp::NodeOptions & options)
: Node(name, options)
{
  // Create a subscription on the input topic which prints on receipt of new messages.
  sub_ = this->create_subscription<std_msgs::msg::Int32>(
    input,
    10,
    [](std_msgs::msg::Int32::UniquePtr msg) {
      printf(
        " Received message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
    });
}

}  // namespace two_node_pipeline
}  // namespace intra_process_demo

RCLCPP_COMPONENTS_REGISTER_NODE(intra_process_demo::two_node_pipeline::Consumer)
