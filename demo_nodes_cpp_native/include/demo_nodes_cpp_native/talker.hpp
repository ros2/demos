// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef DEMO_NODES_CPP_NATIVE__TALKER_HPP_
#define DEMO_NODES_CPP_NATIVE__TALKER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_publisher.hpp"

#include "demo_nodes_cpp_native/visibility_control.h"


namespace demo_nodes_cpp_native
{

class Talker : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_NATIVE_PUBLIC
  explicit Talker(const rclcpp::NodeOptions & options);

  DEMO_NODES_CPP_NATIVE_PUBLIC
  void on_timer();

private:
  size_t count_;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp_native

#endif  // DEMO_NODES_CPP_NATIVE__TALKER_HPP_
