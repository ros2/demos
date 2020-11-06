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

#ifndef CBG_EXECUTOR_DEMO__PINGNODE_HPP_
#define CBG_EXECUTOR_DEMO__PINGNODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace cbg_executor_demo
{

class PingNode : public rclcpp::Node
{
public:
  PingNode();

  virtual ~PingNode() = default;

  rclcpp::CallbackGroup::SharedPtr get_high_prio_callback_group();

  rclcpp::CallbackGroup::SharedPtr get_low_prio_callback_group();

private:
  // The members for the high-prio side of the ping node:

  rclcpp::TimerBase::SharedPtr high_ping_timer_{};

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr high_ping_publisher_{};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr high_pong_subscription_{};

  std::vector<std::pair<rclcpp::Time, rclcpp::Time>> high_timestamps_{};

  void high_ping_timer_callback();

  void high_pong_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);

  // Now, the same for the low-prio side:

  rclcpp::TimerBase::SharedPtr low_ping_timer_{};

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr low_ping_publisher_{};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr low_pong_subscription_{};

  std::vector<std::pair<rclcpp::Time, rclcpp::Time>> low_timestamps_{};

  void low_ping_timer_callback();

  void low_pong_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);
};

} // namespace cbg_executor_demo

#endif  // CBG_EXECUTOR_DEMO__PINGNODE_HPP_
