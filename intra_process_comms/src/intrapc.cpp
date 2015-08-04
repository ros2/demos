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

#include <functional>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

void on_message(const std_msgs::msg::String::ConstSharedPtr & msg)
{
  printf("I heard [%s]\n", msg->data.c_str());
  rclcpp::sleep_for(1_s);
  if (msg->data.at(msg->data.length() - 1) == '9') {
    raise(SIGINT);
  }
}

void on_timer(rclcpp::Publisher::SharedPtr publisher, int & i)
{
  std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String());
  msg->data = "Hello World: " + std::to_string(i++);
  if (publisher) {
    printf("Publishing: '%s'\n", msg->data.c_str());
    publisher->publish(msg);
  } else {
    fprintf(stderr, "Invalid publisher!\n");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node1 = rclcpp::Node::make_shared("node1", true);
  auto node2 = rclcpp::Node::make_shared("node2", true);

  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 2;

  auto publisher = node1->create_publisher<std_msgs::msg::String>("chatter", qos);
  auto subscription =
    node2->create_subscription<std_msgs::msg::String>("chatter", qos, on_message);

  int i = 0;
  using std::placeholders::_1;
  auto timer_bound = std::bind(on_timer, publisher, std::ref(i));
  auto timer = node1->create_wall_timer(0.5_s, timer_bound);

  // rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node1);
  executor.add_node(node2);

  executor.spin();

  return 0;
}
