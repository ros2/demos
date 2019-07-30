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

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"

#include "demo_nodes_cpp/visibility_control.h"
namespace demo_nodes_cpp
{
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit Listener(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
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
      auto callback =
        [this](const std_msgs::msg::String::SharedPtr msg) -> void
        {
          RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        };

      // Create a subscription to the topic which can be matched with one or more compatible ROS
      // publishers.
      // Note that not all publishers on the same topic with the same type will be compatible:
      // they must have compatible Quality of Service policies.
      sub_ = create_subscription<std_msgs::msg::String>(topic_name_, 10, callback);
    }
  }

  DEMO_NODES_CPP_PUBLIC
  void print_usage()
  {
    printf("Usage for listener app:\n");
    printf("listener [-t topic_name] [-h]\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-t topic_name : Specify the topic on which to subscribe. Defaults to chatter.\n");
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string topic_name_ = "chatter";
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Listener)
