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
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "demo_nodes_cpp/visibility_control.h"
#include <sstream>

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{

// Create a class which has a "talker" which uses a timer in a dedicated thread,
// as well as a "listener" for that "talker" which uses a subscription.
// The main function below will instantiate this class as a ROS node and run it
// in a single threaded executor.
class TalkerAndListener : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit TalkerAndListener(const rclcpp::NodeOptions & options)
  : Node("talker_and_listener", options)
  {
    // listener id maps to 1
    id_map_[std::this_thread::get_id()] = 1;

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Beginning to spin listener in thread " << id_map_[std::this_thread::get_id()]);

    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        std::unique_ptr<std_msgs::msg::String> msg = std::make_unique<std_msgs::msg::String>(); 
        std::stringstream ss;
        ss << "Hello World: " << count_++ <<
          ", from thread " << id_map_[std::this_thread::get_id()];
        msg->data = ss.str();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg));
      };

    // Create a publisher with a custom Quality of Service profile.
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 7);

    // Create a callback group for the publisher's timer, and explicitly set
    // automatically_add_to_executor_with_node = false, so that it is not
    // automatically added to the executor when this node is added and spun.
    publisher_callback_group_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive,
      false  /* automatically_add_to_executor_with_node */);

    // Use a timer to schedule periodic message publishing.
    publisher_timer_ = this->create_wall_timer(1s, publish_message, publisher_callback_group_);

    talker_thread_ = std::thread([this]() {
      // talker id maps to 2
      id_map_[std::this_thread::get_id()] = 2;
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Beginning to spin talker in thread " << id_map_[std::this_thread::get_id()]);
      talker_executor_.add_callback_group(
        this->publisher_callback_group_,
        this->get_node_base_interface());
      talker_executor_.spin();
    });

    // Create a callback function for when messages are received.
    auto callback =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void
      {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "I heard: [" << msg->data << "], from thread " <<
          id_map_[std::this_thread::get_id()]);
      };

    // Create a subscription to the topic which can be matched with one or more 
    // compatible ROS publishers.
    // Note that this subscription is implicitly being added to the default
    // callback group of the node, but it could have been part of a custom
    // callback group with automatically_add_to_executor_with_node = true.
    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
  }

  ~TalkerAndListener()
  {
    talker_executor_.cancel();
    talker_thread_.join();
  }

private:
  size_t count_ = 1;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::CallbackGroup::SharedPtr publisher_callback_group_;
  rclcpp::executors::SingleThreadedExecutor talker_executor_;
  std::thread talker_thread_;

  // Create a map for thread ids and assigned ids for talker and listener.
  std::map <std::thread::id,int> id_map_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto tl_node = std::make_shared<demo_nodes_cpp::TalkerAndListener>(options);

  // rclcpp::spin(tl_node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tl_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
