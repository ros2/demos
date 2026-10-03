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

#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_publisher.hpp"

#include "demo_nodes_cpp_native/visibility_control.h"

using namespace std::chrono_literals;
namespace demo_nodes_cpp_native
{
class Talker : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_NATIVE_PUBLIC
  explicit Talker(const rclcpp::NodeOptions & options)
  : Node("talker_native", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rcl_node_t * rcl_node = get_node_base_interface()->get_rcl_node_handle();
    rmw_node_t * rmw_node = rcl_node_get_rmw_handle(rcl_node);
    eprosima::fastdds::dds::DomainParticipant * p =
      rmw_fastrtps_cpp::get_domain_participant(rmw_node);
    RCLCPP_INFO(
      this->get_logger(),
      "eprosima::fastdds::dds::DomainParticipant * %zu",
      reinterpret_cast<size_t>(p));

    auto publish =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::String>();
        msg_->data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
        pub_->publish(std::move(msg_));
      };
    timer_ = create_wall_timer(500ms, publish);
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

    rcl_publisher_t * rcl_pub = pub_->get_publisher_handle().get();
    rmw_publisher_t * rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);
    eprosima::fastdds::dds::DataWriter * dw = rmw_fastrtps_cpp::get_datawriter(rmw_pub);
    RCLCPP_INFO(
      this->get_logger(),
      "eprosima::fastdds::dds::DataWriter * %zu",
      reinterpret_cast<size_t>(dw));
  }

private:
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp_native

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp_native::Talker)
