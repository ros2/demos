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

#ifndef CBG_EXECUTOR_PING_PONG__PING_SUB_NODE_HPP_
#define CBG_EXECUTOR_PING_PONG__PING_SUB_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"


/// This class implements the Ping side of one ping-pong path of the test-bench. See README.md
/// for a simple architecture diagram and a description of the whole test bench.
class PingSide
{
public:
  typedef std::shared_ptr<PingSide> SharedPtr;

  PingSide(
    rclcpp::Node::SharedPtr node, const std::string & topics_prefix,
    const std::chrono::microseconds send_period);

  virtual ~PingSide() = default;

  rclcpp::CallbackGroup::SharedPtr get_callback_group() {return callback_group_;}

  /// Prints out the measured message throughput and latency.
  void print_statistics();

private:
  /// The callback group for the timer and the pong subscription.
  rclcpp::CallbackGroup::SharedPtr callback_group_{};

  /// Prefix for the ping and pong topics - here RT or BE.
  const std::string topics_prefix_;

  /// Timer for sending the pings in the given send period (cf. ctor).
  rclcpp::TimerBase::SharedPtr ping_timer_{};

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ping_publisher_{};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pong_subscription_{};

  size_t ping_sent_count_ = 0;
  size_t pong_received_count_ = 0;
  std::vector<std::chrono::system_clock::time_point> ping_sent_timestamps_{};
  std::vector<std::chrono::system_clock::time_point> pong_received_timestamps_{};

  void ping_timer_callback();

  void pong_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);
};

#endif  // CBG_EXECUTOR_PING_PONG__PING_SUB_NODE_HPP_
