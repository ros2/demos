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

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{
class ClientNode : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ClientNode(const rclcpp::NodeOptions & options)
  : Node("add_two_ints_client", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    queue_async_request();
  }

  DEMO_NODES_CPP_PUBLIC
  void queue_async_request()
  {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 2;
    request->b = 3;

    // We give the async_send_request() method a callback that will get executed once the response
    // is received.
    // This way we can return immediately from this method and allow other work to be done by the
    // executor in `spin` while waiting for the response.
    using ServiceResponseFuture =
      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %" PRId64, result->sum);
        rclcpp::shutdown();
      };
    auto future_result = client_->async_send_request(request, response_received_callback);
  }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ClientNode)
