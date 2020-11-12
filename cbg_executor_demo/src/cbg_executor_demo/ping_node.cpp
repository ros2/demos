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

#include "cbg_executor_demo/ping_node.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <functional>

#include <cbg_executor_demo/parameter_helper.hpp>


namespace cbg_executor_demo
{

PingNode::PingNode()
: rclcpp::Node("ping_node")
{
  using std::placeholders::_1;

  declare_parameter<double>("high_ping_period", 0.01);
  std::chrono::nanoseconds high_period = get_nanos_from_secs_parameter(this, "high_ping_period");

  high_ping_timer_ = create_wall_timer(high_period, std::bind(&PingNode::send_high_ping, this));
  high_ping_publisher_ = create_publisher<std_msgs::msg::Int32>(
    "high_ping", rclcpp::SystemDefaultsQoS());
  high_pong_subscription_ = create_subscription<std_msgs::msg::Int32>(
    "high_pong", rclcpp::SystemDefaultsQoS(),
    std::bind(&PingNode::high_pong_received, this, _1));

  auto second_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  assert(second_cb_group == get_low_prio_callback_group());

  declare_parameter<double>("low_ping_period", 0.01);
  std::chrono::nanoseconds low_period = get_nanos_from_secs_parameter(this, "low_ping_period");

  low_ping_timer_ = create_wall_timer(
    low_period, std::bind(&PingNode::send_low_ping, this), get_low_prio_callback_group());
  low_ping_publisher_ = create_publisher<std_msgs::msg::Int32>(
    "low_ping", rclcpp::SystemDefaultsQoS());
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = second_cb_group;
  low_pong_subscription_ = create_subscription<std_msgs::msg::Int32>(
    "low_pong", rclcpp::SystemDefaultsQoS(),
    std::bind(&PingNode::low_pong_received, this, _1), options);
}


rclcpp::CallbackGroup::SharedPtr PingNode::get_high_prio_callback_group()
{
  return get_callback_groups()[0].lock();  // ... the default callback group.
}


rclcpp::CallbackGroup::SharedPtr PingNode::get_low_prio_callback_group()
{
  return get_callback_groups()[1].lock();  // ... the second callback group created in the ctor.
}


void PingNode::send_high_ping()
{
  std_msgs::msg::Int32 msg;
  msg.data = high_timestamps_.size();
  high_timestamps_.push_back(std::make_pair(now(), rclcpp::Time()));
  high_ping_publisher_->publish(msg);
}


void PingNode::high_pong_received(const std_msgs::msg::Int32::SharedPtr msg)
{
  high_timestamps_[msg->data].second = now();
}


void PingNode::send_low_ping()
{
  std_msgs::msg::Int32 msg;
  msg.data = low_timestamps_.size();
  low_timestamps_.push_back(std::make_pair(now(), rclcpp::Time()));
  low_ping_publisher_->publish(msg);
}


void PingNode::low_pong_received(const std_msgs::msg::Int32::SharedPtr msg)
{
  low_timestamps_[msg->data].second = now();
}


std::vector<rclcpp::Duration> PingNode::calc_latencies(
  const std::vector<std::pair<rclcpp::Time, rclcpp::Time>>& timestamps)
{
   std::vector<rclcpp::Duration> latencies;
   for (const auto pair : timestamps) {
     if (pair.first.get_clock_type() == pair.second.get_clock_type()
         && pair.first <= pair.second) {
       latencies.push_back(pair.second - pair.first);
     }
   }
   return latencies;
}


rclcpp::Duration PingNode::calc_avg_latency(const std::vector<rclcpp::Duration>& latencies)
{
  rclcpp::Duration sum = std::accumulate(latencies.begin(), latencies.end(), 
    rclcpp::Duration(0 , 0));
  rclcpp::Duration avg(sum.nanoseconds() / latencies.size());
  return avg;
}


void PingNode::print_statistics()
{
  size_t high_ping_count = high_timestamps_.size();
  std::vector<rclcpp::Duration> high_latencies = calc_latencies(high_timestamps_);
  size_t high_pong_count = high_latencies.size();
  RCLCPP_INFO(get_logger(), "High prio path: Sent %d pings, received %d pongs.", high_ping_count, 
    high_pong_count);
  RCLCPP_INFO(get_logger(), "High prio path: Average latency is %3.1f ms.",
    calc_avg_latency(high_latencies).seconds() * 1000.0);

  size_t low_ping_count = low_timestamps_.size();
  std::vector<rclcpp::Duration> low_latencies = calc_latencies(low_timestamps_);
  size_t low_pong_count = low_latencies.size();
  RCLCPP_INFO(get_logger(), "Low prio path: Sent %d pings, received %d pongs", low_ping_count, 
    low_pong_count);
  RCLCPP_INFO(get_logger(), "Low prio path: Average latency is %3.1f ms.",
    calc_avg_latency(low_latencies).seconds() * 1000.0);

  // // RCLCPP_INFO(node->get_logger(), "Sending goal");

  // std::cout << "Sent " << ping_sent_count_ << " pings on " << topics_prefix_ << "_ping topic" <<
  //   std::endl;
  // std::cout << "Received " << pong_received_count_ << " pongs on " << topics_prefix_ <<
  //   "_pong topic" << std::endl;

  // std::chrono::system_clock::duration latencyMax = 0us;
  // std::chrono::system_clock::duration latencyMin = 100000000s;
  // std::chrono::system_clock::duration latencySum = 0us;

  // for (size_t i = 0; i < ping_sent_timestamps_.size(); ++i) {
  //   if (pong_received_timestamps_[i] >= ping_sent_timestamps_[i]) {
  //     std::chrono::system_clock::duration latency = pong_received_timestamps_[i] -
  //       ping_sent_timestamps_[i];
  //     latencyMax = std::max(latencyMax, latency);
  //     latencyMin = std::min(latencyMin, latency);
  //     latencySum += latency;
  //   }
  // }
  // if (pong_received_count_ > 0) {
  //   std::chrono::system_clock::duration latencyAvg = latencySum / pong_received_count_;

  //   std::cout << "latency on " << topics_prefix_ << " path: min=" <<
  //     std::chrono::duration_cast<std::chrono::microseconds>(latencyMin).count() <<
  //     "us max=" << std::chrono::duration_cast<std::chrono::microseconds>(latencyMax).count() <<
  //     "us avg=" << std::chrono::duration_cast<std::chrono::microseconds>(latencyAvg).count() <<
  //     "us " << std::endl;
  // }
}

}  // namespace cbg_executor_demo
