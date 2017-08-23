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

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("talker_native");

  {
    rcl_node_t * rcl_node = node->get_node_base_interface()->get_rcl_node_handle();
    rmw_node_t * rmw_node = rcl_node_get_rmw_handle(rcl_node);
    eprosima::fastrtps::Participant * p = rmw_fastrtps_cpp::get_participant(rmw_node);
    printf("eprosima::fastrtps::Participant * %zu\n", reinterpret_cast<size_t>(p));
  }

  auto pub = node->create_publisher<std_msgs::msg::String>("chatter");

  {
    rcl_publisher_t * rcl_pub = pub->get_publisher_handle();
    rmw_publisher_t * rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);
    eprosima::fastrtps::Publisher * p = rmw_fastrtps_cpp::get_publisher(rmw_pub);
    printf("eprosima::fastrtps::Publisher * %zu\n", reinterpret_cast<size_t>(p));
  }

  rclcpp::WallRate loop_rate(2);

  auto msg = std::make_shared<std_msgs::msg::String>();
  auto i = 1;

  while (rclcpp::ok()) {
    msg->data = "Hello World: " + std::to_string(i++);
    std::cout << "Publishing: '" << msg->data << "'" << std::endl;
    pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
