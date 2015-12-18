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
#include <cinttypes>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// This node receives an Int32, waits 1 second, then increments and sends it.
struct IncrementerPipe : public rclcpp::Node
{
  IncrementerPipe(const std::string & name, const std::string & in, const std::string & out)
  : Node(name, true)
  {
    // Create a publisher on the output topic.
    pub = this->create_publisher<std_msgs::msg::Int32>(out, rmw_qos_profile_default);
    std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> captured_pub = pub;
    // Create a subscription on the input topic.
    sub = this->create_subscription<std_msgs::msg::Int32>(
      in, [captured_pub](std_msgs::msg::Int32::UniquePtr msg) {
      auto pub_ptr = captured_pub.lock();
      if (!pub_ptr) {
        return;
      }
      printf(
        "Received message with value:         %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
      printf("  sleeping for 1 second...\n");
      if (!rclcpp::sleep_for(1_s)) {
        return;    // Return if the sleep failed (e.g. on ctrl-c).
      }
      printf("  done.\n");
      msg->data++;    // Increment the message's data.
      printf(
        "Incrementing and sending with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        reinterpret_cast<std::uintptr_t>(msg.get()));
      pub_ptr->publish(msg);    // Send the message along to the output topic.
    }, rmw_qos_profile_default);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
};

int main(int argc, char * argv[])
{
  setbuf(stdout, NULL);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create a simple loop by connecting the in and out topics of two IncrementerPipe's.
  // The expectation is that the address of the message being passed between them never changes.
  auto pipe1 = std::make_shared<IncrementerPipe>("pipe1", "topic1", "topic2");
  auto pipe2 = std::make_shared<IncrementerPipe>("pipe2", "topic2", "topic1");
  rclcpp::sleep_for(1_s);  // Wait for subscriptions to be established to avoid race conditions.
  // Publish the first message (kicking off the cycle).
  std::unique_ptr<std_msgs::msg::Int32> msg(new std_msgs::msg::Int32());
  msg->data = 42;
  printf(
    "Published first message with value:  %d, and address: 0x%" PRIXPTR "\n", msg->data,
    reinterpret_cast<std::uintptr_t>(msg.get()));
  pipe1->pub->publish(msg);

  executor.add_node(pipe1);
  executor.add_node(pipe2);
  executor.spin();
  return 0;
}
