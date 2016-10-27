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

#include "composition/listener_component.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

Listener::Listener()
: Node("listener")
{
  auto callback =
    [](const typename std_msgs::msg::String::SharedPtr msg) -> void
    {
      printf("I heard: [%s]\n", msg->data.c_str());
      std::flush(std::cout);
    };

  sub_ = create_subscription<std_msgs::msg::String>(
    "chatter", callback);
}

}  // namespace composition

#include "class_loader/class_loader_register_macro.h"

CLASS_LOADER_REGISTER_CLASS(composition::Listener, rclcpp::Node)
