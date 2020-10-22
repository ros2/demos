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

#ifndef CBG_EXECUTOR_PING_PONG__PONG_SUB_NODE_HPP_
#define CBG_EXECUTOR_PING_PONG__PONG_SUB_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

/// This class implements the Pong side of one ping-pong path of the test-bench. See README.md
/// for a simple architecture diagram and a description of the whole test bench.
class PongSide
{
public:
  typedef std::shared_ptr<PongSide> SharedPtr;

  PongSide(
    rclcpp::Node::SharedPtr node, const std::string & topics_prefix,
    std::chrono::microseconds cpu_load);

  virtual ~PongSide() = default;

  rclcpp::CallbackGroup::SharedPtr get_callback_group() {return callback_group_;}

private:
  /// The callback group for the ping subscription.
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  /// Prefix for the ping and pong topics - here RT or BE.
  const std::string topics_prefix_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ping_subscription_{};

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pong_publisher_{};

  /// The given duration (cf. ctor) to simulate some processing on receiving a ping message.
  const std::chrono::microseconds cpu_load_;

  void ping_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);

  /// Burns CPU cycles for the cpu_load_ duration.
  void burn_cpu_cycles();
};

#endif  // CBG_EXECUTOR_PING_PONG__PONG_SUB_NODE_HPP_
