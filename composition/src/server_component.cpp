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

#include "composition/server_component.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace composition
{

Server::Server(const rclcpp::NodeOptions & options)
: Node("Server", options)
{
  auto handle_add_two_ints =
    [this](
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
    ) -> void
    {
      RCLCPP_INFO(
        this->get_logger(), "Incoming request: [a: %" PRId64 ", b: %" PRId64 "]",
        request->a, request->b);
      std::flush(std::cout);
      response->sum = request->a + request->b;
    };

  srv_ = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", handle_add_two_ints);
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Server)
