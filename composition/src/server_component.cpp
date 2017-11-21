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

Server::Server()
: Node("Server")
{
  auto handle_add_two_ints =
    [](
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
    ) -> void
    {
      printf("Incoming request: [a: %" PRId64 ", b: %" PRId64 "]\n",
        request->a, request->b);
      std::flush(std::cout);
      response->sum = request->a + request->b;
    };

  srv_ = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", handle_add_two_ints);
}

}  // namespace composition

#include "class_loader/class_loader_register_macro.h"

CLASS_LOADER_REGISTER_CLASS(composition::Server, rclcpp::Node)
