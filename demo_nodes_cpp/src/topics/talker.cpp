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
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/cmdline_parser.h"

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
    std::vector<std::string> args = options.arguments();
    if (find_command_option(args, "-h")) {
      print_usage();
      rclcpp::shutdown();
    } else {
      std::string tmptopic = get_command_option(args, "-t");
      if (!tmptopic.empty()) {
        topic_name_ = tmptopic;
      }
      auto publish_message =
        [this]() -> void
        {
          msg_ = std::make_unique<std_msgs::msg::String>();
          msg_->data = "Hello World: " + std::to_string(count_++);
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
          // Put the message into a queue to be processed by the middleware.
          // This call is non-blocking.
          pub_->publish(std::move(msg_));
        };
      // Create a publisher with a custom Quality of Service profile.
      rclcpp::QoS qos(rclcpp::KeepLast(7));
      pub_ = this->create_publisher<std_msgs::msg::String>(topic_name_, qos);

      // Use a timer to schedule periodic message publishing.
      timer_ = this->create_wall_timer(1s, publish_message);
    }
  }

  DEMO_NODES_CPP_PUBLIC
  void print_usage()
  {
    printf("Usage for talker app:\n");
    printf("talker [-t topic_name] [-h]\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
  }

  DEMO_NODES_CPP_PUBLIC
  bool find_command_option(const std::vector<std::string> & args, const std::string & option)
  {
    return std::find(args.begin(), args.end(), option) != args.end();
  }

  DEMO_NODES_CPP_PUBLIC
  std::string get_command_option(const std::vector<std::string> & args, const std::string & option)
  {
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end() && ++it != args.end()) {
      return *it;
    }
    return std::string();
  }

private:
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string topic_name_ = "chatter";
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Talker)
