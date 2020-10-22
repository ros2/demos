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

#include "PingSide.hpp"

#include <cassert>

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

PingSide::PingSide(
  rclcpp::Node::SharedPtr node, const std::string & topics_prefix,
  const std::chrono::microseconds send_period)
: topics_prefix_(topics_prefix)
{
  assert(node != nullptr);
  assert(!topics_prefix.empty());

  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  ping_sent_timestamps_.reserve(10000000); // TODO(Ralph): Adjust size to experiment duration and send period.
  pong_received_timestamps_.resize(10000000);

  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = callback_group_;

  ping_timer_ = node->create_wall_timer(send_period, std::bind(&PingSide::ping_timer_callback,
      this), callback_group_);
  ping_publisher_ = node->create_publisher<std_msgs::msg::Int32>(topics_prefix_ + "_ping", rclcpp::SystemDefaultsQoS());
  pong_subscription_ = node->create_subscription<std_msgs::msg::Int32>(topics_prefix_ + "_pong", rclcpp::SystemDefaultsQoS(),
      std::bind(&PingSide::pong_subscription_callback, this,
      _1), options);
}


void PingSide::ping_timer_callback()
{
  std_msgs::msg::Int32 message;
  message.data = ping_sent_count_;
  ping_sent_timestamps_.push_back(std::chrono::system_clock::now());
  ping_publisher_->publish(message);
  ++ping_sent_count_;
}


void PingSide::pong_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  pong_received_timestamps_[msg->data] = std::chrono::system_clock::now();
  ++pong_received_count_;
}


void PingSide::print_statistics()
{
  std::cout << "Sent " << ping_sent_count_ << " pings on " << topics_prefix_ << "_ping topic" <<
    std::endl;
  std::cout << "Received " << pong_received_count_ << " pongs on " << topics_prefix_ <<
    "_pong topic" << std::endl;

  std::chrono::system_clock::duration latencyMax = 0us;
  std::chrono::system_clock::duration latencyMin = 100000000s;
  std::chrono::system_clock::duration latencySum = 0us;

  for (size_t i = 0; i < ping_sent_timestamps_.size(); ++i) {
    if (pong_received_timestamps_[i] >= ping_sent_timestamps_[i]) {
      std::chrono::system_clock::duration latency = pong_received_timestamps_[i] -
        ping_sent_timestamps_[i];
      latencyMax = std::max(latencyMax, latency);
      latencyMin = std::min(latencyMin, latency);
      latencySum += latency;
    }
  }
  if (pong_received_count_ > 0) {
    std::chrono::system_clock::duration latencyAvg = latencySum / pong_received_count_;

    std::cout << "latency on " << topics_prefix_ << " path: min=" <<
      std::chrono::duration_cast<std::chrono::microseconds>(latencyMin).count() <<
      "us max=" << std::chrono::duration_cast<std::chrono::microseconds>(latencyMax).count() <<
      "us avg=" << std::chrono::duration_cast<std::chrono::microseconds>(latencyAvg).count() <<
      "us " << std::endl;
  }
}
