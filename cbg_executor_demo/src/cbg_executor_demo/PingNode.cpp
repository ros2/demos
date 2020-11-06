// Copyright
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

#include "cbg_executor_demo/PingNode.hpp"

#include <cassert>
#include <chrono>
#include <functional>


namespace cbg_executor_demo
{

PingNode::PingNode()
: rclcpp::Node("ping_node")
{
  using namespace std::chrono_literals;
  using std::placeholders::_1;

  const std::chrono::microseconds high_ping_period = 100000us;

  high_ping_timer_ =
    create_wall_timer(high_ping_period, std::bind(&PingNode::high_ping_timer_callback, this));
  high_ping_publisher_ = create_publisher<std_msgs::msg::Int32>(
    "high_ping",
    rclcpp::SystemDefaultsQoS());
  high_pong_subscription_ = create_subscription<std_msgs::msg::Int32>(
    "high_pong", rclcpp::SystemDefaultsQoS(),
    std::bind(&PingNode::high_pong_subscription_callback, this, _1));

  auto second_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  assert(second_callback_group == get_low_prio_callback_group());

  const std::chrono::microseconds low_ping_period = 100000us;

  low_ping_timer_ =
    create_wall_timer(
    low_ping_period, std::bind(
      &PingNode::low_ping_timer_callback,
      this), get_low_prio_callback_group());
  low_ping_publisher_ = create_publisher<std_msgs::msg::Int32>(
    "low_ping",
    rclcpp::SystemDefaultsQoS());
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = second_callback_group;
  low_pong_subscription_ = create_subscription<std_msgs::msg::Int32>(
    "low_pong", rclcpp::SystemDefaultsQoS(),
    std::bind(&PingNode::low_pong_subscription_callback, this, _1), options);
}


rclcpp::CallbackGroup::SharedPtr PingNode::get_high_prio_callback_group()
{
  return get_callback_groups()[0].lock();  // ... which is the default callback group.
}


rclcpp::CallbackGroup::SharedPtr PingNode::get_low_prio_callback_group()
{
  return get_callback_groups()[1].lock();  // ... which is the second callback group create in the ctor.
}


void PingNode::high_ping_timer_callback()
{
  std_msgs::msg::Int32 msg;
  msg.data = high_timestamps_.size();
  high_timestamps_.push_back(std::make_pair(now(), rclcpp::Time()));
  high_ping_publisher_->publish(msg);
}


void PingNode::high_pong_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  high_timestamps_[msg->data].second = now();
}


void PingNode::low_ping_timer_callback()
{
  std_msgs::msg::Int32 msg;
  msg.data = low_timestamps_.size();
  low_timestamps_.push_back(std::make_pair(now(), rclcpp::Time()));
  low_ping_publisher_->publish(msg);
}


void PingNode::low_pong_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  low_timestamps_[msg->data].second = now();
}


// void PingNode::print_statistics()
// {
//   std::cout << "Sent " << ping_sent_count_ << " pings on " << topics_prefix_ << "_ping topic" <<
//     std::endl;
//   std::cout << "Received " << pong_received_count_ << " pongs on " << topics_prefix_ <<
//     "_pong topic" << std::endl;

//   std::chrono::system_clock::duration latencyMax = 0us;
//   std::chrono::system_clock::duration latencyMin = 100000000s;
//   std::chrono::system_clock::duration latencySum = 0us;

//   for (size_t i = 0; i < ping_sent_timestamps_.size(); ++i) {
//     if (pong_received_timestamps_[i] >= ping_sent_timestamps_[i]) {
//       std::chrono::system_clock::duration latency = pong_received_timestamps_[i] -
//         ping_sent_timestamps_[i];
//       latencyMax = std::max(latencyMax, latency);
//       latencyMin = std::min(latencyMin, latency);
//       latencySum += latency;
//     }
//   }
//   if (pong_received_count_ > 0) {
//     std::chrono::system_clock::duration latencyAvg = latencySum / pong_received_count_;

//     std::cout << "latency on " << topics_prefix_ << " path: min=" <<
//       std::chrono::duration_cast<std::chrono::microseconds>(latencyMin).count() <<
//       "us max=" << std::chrono::duration_cast<std::chrono::microseconds>(latencyMax).count() <<
//       "us avg=" << std::chrono::duration_cast<std::chrono::microseconds>(latencyAvg).count() <<
//       "us " << std::endl;
//   }
// }

} // namespace cbg_executor_demo
