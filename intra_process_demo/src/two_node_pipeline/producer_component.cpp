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

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32.hpp"

#include "intra_process_demo/two_node_pipeline/producer_component.hpp"
#include "intra_process_demo/visibility_control.h"

using namespace std::chrono_literals;

namespace intra_process_demo
{
namespace two_node_pipeline
{

Producer::Producer(const rclcpp::NodeOptions & options)
: Producer("producer", "number", options) {}

Producer::Producer(
  const std::string & name,
  const std::string & output,
  const rclcpp::NodeOptions & options)
: Node(name, options)
{
  // Create a publisher on the output topic.
  pub_ = this->create_publisher<std_msgs::msg::Int32>(output, 10);
  std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
  // Create a timer which publishes on the output topic at ~1Hz.
  auto callback = [captured_pub]() -> void {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr) {
        return;
      }
      static int32_t count = 0;
      std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
      msg->data = count++;
      printf(
        "Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
      pub_ptr->publish(std::move(msg));
    };
  timer_ = this->create_wall_timer(1s, callback);
}

}  // namespace two_node_pipeline
}  // namespace intra_process_demo

RCLCPP_COMPONENTS_REGISTER_NODE(intra_process_demo::two_node_pipeline::Producer)
