// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef INTRA_PROCESS_DEMO__TWO_NODE_PIPELINE__PRODUCER_COMPONENT_HPP_
#define INTRA_PROCESS_DEMO__TWO_NODE_PIPELINE__PRODUCER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "intra_process_demo/visibility_control.h"

namespace intra_process_demo
{
namespace two_node_pipeline
{

class Producer : public rclcpp::Node
{
public:
  INTRA_PROCESS_DEMO_PUBLIC
  explicit Producer(const rclcpp::NodeOptions & options);

  explicit Producer(
    const std::string & name,
    const std::string & output,
    const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace two_node_pipeline
}  // namespace intra_process_demo

#endif  // INTRA_PROCESS_DEMO__TWO_NODE_PIPELINE__PRODUCER_COMPONENT_HPP_
