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

class PingSubnode
{
public:
  PingSubnode(
    rclcpp::Node * node, std::string prefix,
    rclcpp::CallbackGroup::SharedPtr callback_group)
  : node_(node), prefix_(prefix), callback_group_(callback_group)
  {
    using std::placeholders::_1;

    node_->declare_parameter<double>(prefix_ + "_ping_period", 0.01);
    std::chrono::nanoseconds ping_period = get_nanos_from_secs_parameter(
      node_,
      prefix_ + "_ping_period");

    ping_timer_ = node_->create_wall_timer(
      ping_period,
      std::bind(&PingSubnode::send_ping, this), callback_group_);
    ping_publisher_ = node->create_publisher<std_msgs::msg::Int32>(
      prefix + "_ping", rclcpp::SystemDefaultsQoS());
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
    options.callback_group = callback_group_;
    pong_subscription_ = node->create_subscription<std_msgs::msg::Int32>(
      prefix_ + "_pong",
      rclcpp::SystemDefaultsQoS(), std::bind(&PingSubnode::pong_received, this, _1), options);
  }

  void send_ping()
  {
    std_msgs::msg::Int32 msg;
    msg.data = ping_times_.size();
    ping_times_.push_back(node_->now());
    pong_times_.resize(ping_times_.size());
    ping_publisher_->publish(msg);
  }

  void pong_received(const std_msgs::msg::Int32::SharedPtr msg)
  {
    pong_times_[msg->data] = node_->now();
  }

  size_t count_sent_pings() const
  {
    return ping_times_.size();
  }

  size_t count_received_pongs() const
  {
    return std::count_if(
      pong_times_.cbegin(), pong_times_.cend(), [](rclcpp::Time t)
      {
        return t.nanoseconds() > 0;
      });
  }

  rclcpp::Duration calc_average_latency()
  {
    assert(count_received_pongs() > 0);
    rclcpp::Duration sum(0, 0);
    for (size_t i = 0; i < ping_times_.size(); ++i) {
      if (pong_times_[i].nanoseconds() > 0) {
        sum = sum + (pong_times_[i] - ping_times_[i]);
      }
    }
    return rclcpp::Duration(sum.nanoseconds() / count_received_pongs());
  }

private:
  rclcpp::Node * node_;
  const std::string prefix_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr ping_timer_{};
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ping_publisher_{};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pong_subscription_{};
  std::vector<rclcpp::Time> ping_times_{};
  std::vector<rclcpp::Time> pong_times_{};
};


PingNode::PingNode()
: rclcpp::Node("ping_node")
{
  auto low_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  assert(low_cb_group == get_low_prio_callback_group());

  high_subnode_ = std::make_shared<PingSubnode>(
    this, "high",
    get_high_prio_callback_group());
  low_subnode_ = std::make_shared<PingSubnode>(
    this, "low",
    get_low_prio_callback_group());
}


rclcpp::CallbackGroup::SharedPtr PingNode::get_high_prio_callback_group()
{
  return get_callback_groups()[0].lock();  // ... the default callback group.
}


rclcpp::CallbackGroup::SharedPtr PingNode::get_low_prio_callback_group()
{
  return get_callback_groups()[1].lock();  // ... the second callback group created in the ctor.
}


void PingNode::print_statistics()
{
  RCLCPP_INFO(
    get_logger(), "High prio path: Sent %d pings, received %d pongs.",
    high_subnode_->count_sent_pings(), high_subnode_->count_received_pongs());
  if (high_subnode_->count_received_pongs() > 0) {
    RCLCPP_INFO(
      get_logger(), "High prio path: Average latency is %3.1f ms.",
      high_subnode_->calc_average_latency().seconds() * 1000.0);
  }

  RCLCPP_INFO(
    get_logger(), "Low prio path: Sent %d pings, received %d pongs",
    low_subnode_->count_sent_pings(), low_subnode_->count_received_pongs());
  if (low_subnode_->count_received_pongs() > 0) {
    RCLCPP_INFO(
      get_logger(), "Low prio path: Average latency is %3.1f ms.",
      low_subnode_->calc_average_latency().seconds() * 1000.0);
  }
}

}  // namespace cbg_executor_demo
