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
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker : public rclcpp::Node
{
public:

  DEMO_NODES_CPP_PUBLIC
  explicit Talker(const rclcpp::NodeOptions & options)
  : Node("talker", options)
  {
    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message =
      [this]() -> void
      {
        if (launch_thread_) {
          future_ = std::async(std::launch::async, &Talker::create_publishers_thread, this);
          launch_thread_ = false;
        }
        msg_ = std::make_unique<std_msgs::msg::String>();
        msg_->data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg_));
      };
    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

  void create_publishers_thread()
  {
    fprintf(stderr, "creating_publishers\n");
    auto callback =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void
      {};

    for(size_t i = 0; i < 1000; i++) {
      std::ostringstream oss;
      fprintf(stderr, "loop\n");
      oss << "chatter" << i;
      pubs_.emplace_back(this->create_publisher<std_msgs::msg::String>(oss.str(), 10));
      fprintf(stderr, "pub created\n");
      subs_.emplace_back(this->create_subscription<std_msgs::msg::String>(oss.str(), 10, callback));
      fprintf(stderr, "end loop\n");
    }
    pubs_.resize(0);
    subs_.resize(0);
    fprintf(stderr, "created\n");
  }

  ~Talker() override {
    fprintf(stderr, "waiting async thread to finish\n");
    future_.wait();
    fprintf(stderr, "done ...\n");
  }

private:
  bool launch_thread_{true};
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
  std::future<void> future_;
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Talker)
