// Copyright (c) 2020 Robert Bosch GmbH
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

#include "cbg_executor_demo/PongNode.hpp"

#include <cassert>
#include <chrono>

namespace cbg_executor_demo
{

PongNode::PongNode()
: rclcpp::Node("pong_node")
{
  using std::placeholders::_1;

  high_pong_publisher_ = create_publisher<std_msgs::msg::Int32>(
    "high_pong",
    rclcpp::SystemDefaultsQoS());

  high_ping_subscription_ = create_subscription<std_msgs::msg::Int32>(
    "high_ping",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &PongNode::high_ping_subscription_callback, this,
      _1));

  auto second_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  assert(second_callback_group == get_low_prio_callback_group());

  low_pong_publisher_ = create_publisher<std_msgs::msg::Int32>(
    "low_pong",
    rclcpp::SystemDefaultsQoS());

  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = second_callback_group;
  low_ping_subscription_ = create_subscription<std_msgs::msg::Int32>(
    "low_ping",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &PongNode::low_ping_subscription_callback, this,
      _1), options);
}


rclcpp::CallbackGroup::SharedPtr PongNode::get_high_prio_callback_group()
{
  return get_callback_groups()[0].lock();  // ... which is the default callback group.
}


rclcpp::CallbackGroup::SharedPtr PongNode::get_low_prio_callback_group()
{
  return get_callback_groups()[1].lock();  // ... which is the second callback group create in the ctor.
}


void PongNode::high_ping_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  using namespace std::chrono_literals;
  burn_cpu_cycles(100000us);
  high_pong_publisher_->publish(*msg);
}


void PongNode::low_ping_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  using namespace std::chrono_literals;
  burn_cpu_cycles(100000us);
  low_pong_publisher_->publish(*msg);
}


void PongNode::burn_cpu_cycles(std::chrono::microseconds duration)
{
  if (duration > std::chrono::microseconds::zero()) {
    clockid_t clockId;
    pthread_getcpuclockid(pthread_self(), &clockId);
    timespec startTimeP;
    clock_gettime(clockId, &startTimeP);
    auto endTime = duration +
      std::chrono::seconds{startTimeP.tv_sec} +
    std::chrono::nanoseconds{startTimeP.tv_nsec};
    int x = 0;
    bool doAgain = true;
    while (doAgain) {
      while (x != std::rand() && x % 1000 != 0) {
        x++;
      }
      timespec currentTimeP;
      clock_gettime(clockId, &currentTimeP);
      auto currentTime = std::chrono::seconds{currentTimeP.tv_sec} +
      std::chrono::nanoseconds{currentTimeP.tv_nsec};
      doAgain = (currentTime < endTime);
    }
  }
}

}  // namespace cbg_executor_demo
