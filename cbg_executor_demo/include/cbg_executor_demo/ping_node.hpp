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

struct PingSubnode;

class PingNode : public rclcpp::Node
{
public:
  PingNode();
  virtual ~PingNode() = default;
  rclcpp::CallbackGroup::SharedPtr get_high_prio_callback_group();
  rclcpp::CallbackGroup::SharedPtr get_low_prio_callback_group();
  void print_statistics();

private:
  std::shared_ptr<PingSubnode> high_subnode_;
  std::shared_ptr<PingSubnode> low_subnode_;
};

}  // namespace cbg_executor_demo

#endif  // CBG_EXECUTOR_DEMO__PINGNODE_HPP_
