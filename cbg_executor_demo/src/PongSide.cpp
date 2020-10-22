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

#include "PongSide.hpp"

#include <cassert>
#include <chrono>

using std::placeholders::_1;


PongSide::PongSide(
  rclcpp::Node::SharedPtr node, const std::string & topics_prefix,
  std::chrono::microseconds cpu_load)
: topics_prefix_(topics_prefix), cpu_load_(cpu_load)
{
  assert(node != nullptr);
  assert(!topics_prefix.empty());
  assert(cpu_load_ >= std::chrono::microseconds::zero());

  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = callback_group_;

  ping_subscription_ = node->create_subscription<std_msgs::msg::Int32>(topics_prefix_ + "_ping",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PongSide::ping_subscription_callback, this,
      _1), options);
  pong_publisher_ = node->create_publisher<std_msgs::msg::Int32>(topics_prefix_ + "_pong", rclcpp::SystemDefaultsQoS());
}


void PongSide::ping_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  burn_cpu_cycles();
  pong_publisher_->publish(*msg);
}


void PongSide::burn_cpu_cycles()
{
  if (cpu_load_ > std::chrono::microseconds::zero()) {
    clockid_t clockId;
    pthread_getcpuclockid(pthread_self(), &clockId);
    timespec startTimeP;
    clock_gettime(clockId, &startTimeP);
    auto endTime = cpu_load_ +
      std::chrono::seconds{startTimeP.tv_sec}
    +std::chrono::nanoseconds{startTimeP.tv_nsec};
    int x = 0;
    bool doAgain = true;
    while (doAgain) {
      while (x != std::rand() && x % 1000 != 0) {
        x++;
      }
      timespec currentTimeP;
      clock_gettime(clockId, &currentTimeP);
      auto currentTime = std::chrono::seconds{currentTimeP.tv_sec}
      +std::chrono::nanoseconds{currentTimeP.tv_nsec};
      doAgain = (currentTime < endTime);
    }
  }
}
