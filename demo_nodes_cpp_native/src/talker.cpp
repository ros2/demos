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


#include "talker.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace demo_nodes_cpp_native
{

Talker::Talker(const rclcpp::NodeOptions options)
: Node("talker", options), count_(0)
{

  timer_ = create_wall_timer(500ms, std::bind(&Talker::on_timer, this));  

  pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
  
  rcl_node_t * rcl_node = get_node_base_interface()->get_rcl_node_handle();
  rmw_node_t * rmw_node = rcl_node_get_rmw_handle(rcl_node);
  eprosima::fastrtps::Participant * participant = rmw_fastrtps_cpp::get_participant(rmw_node);
  RCLCPP_INFO(
    this->get_logger(), "eprosima::fastrtps::Participant * %zu", reinterpret_cast<size_t>(participant));
    

  rcl_publisher_t * rcl_pub = pub_->get_publisher_handle();
  rmw_publisher_t * rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);
  eprosima::fastrtps::Publisher * pub = rmw_fastrtps_cpp::get_publisher(rmw_pub);
  RCLCPP_INFO(
    this->get_logger(), "eprosima::fastrtps::Publisher * %zu", reinterpret_cast<size_t>(pub));

}

void Talker::on_timer()
{

  msg_ = std::make_unique<std_msgs::msg::String>();
  msg_->data = "Hello World: " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
  pub_->publish(std::move(msg_));

}

}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp_native::Talker)


