// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{

class ServerNode final : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ServerNode(const rclcpp::NodeOptions & options)
  : Node("add_two_ints_server", options)
  {
    auto handle_add_two_ints = [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(
          this->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64,
          request->a, request->b);
        response->sum = request->a + request->b;

        saw_request_ = true;
      };
    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", handle_add_two_ints);

    bool one_shot = this->declare_parameter("one_shot", false);
    if (one_shot) {
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          if (saw_request_) {
            rclcpp::shutdown();
          }
        });
    }
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool saw_request_{false};
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ServerNode)
