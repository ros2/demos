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

#include <chrono>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// Node that produces messages.
struct Producer : public rclcpp::Node
{
  Producer(const std::string & name, const std::string & output)
  : Node(name, true)
  {
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output, rmw_qos_profile_default);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    timer_ = this->create_wall_timer(1_s, [captured_pub]() -> bool {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr) {
        return false;
      }
      static int32_t count = 0;
      std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
      msg->data = count++;
      printf("Published message with value: %d, and address: %p\n", msg->data, msg.get());
      pub_ptr->publish(msg);
      return false;
    });
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::WallTimer::SharedPtr timer_;
};

// Node that consumes messages.
struct Consumer : public rclcpp::Node
{
  Consumer(const std::string & name, const std::string & input)
  : Node(name, true)
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input, [](std_msgs::msg::Int32::UniquePtr msg) {
      printf(" Received message with value: %d, and address: %p\n", msg->data, msg.get());
    }, rmw_qos_profile_default);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto producer = std::make_shared<Producer>("consumer", "number");
  auto consumer = std::make_shared<Consumer>("producer", "number");

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();
  return 0;
}
