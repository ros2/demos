// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#include <vector>

#include "rcl/types.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "demo_nodes_cpp/visibility_control.h"
namespace demo_nodes_cpp
{
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class SerializedMessageListener : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit SerializedMessageListener(const rclcpp::NodeOptions & options)
  : Node("serialized_message_listener", options)
  {
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
      // We create a callback to a rmw_serialized_message_t here. This will pass a serialized
      // message to the callback. We can then further deserialize it and convert it into
      // a ros2 compliant message.
      auto callback =
        [](const std::shared_ptr<rmw_serialized_message_t> msg) -> void
        {
          // Print the serialized data message in HEX representation
          // This output corresponds to what you would see in e.g. Wireshark
          // when tracing the RTPS packets.
          std::cout << "I heard data of length: " << msg->buffer_length << std::endl;
          for (size_t i = 0; i < msg->buffer_length; ++i) {
            printf("%02x ", msg->buffer[i]);
          }
          printf("\n");

          // In order to deserialize the message we have to manually create a ROS2
          // message in which we want to convert the serialized data.
          auto string_msg = std::make_shared<std_msgs::msg::String>();
          auto string_ts =
            rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::String>();
          // The rmw_deserialize function takes the serialized data and a corresponding typesupport
          // which is responsible on how to convert this data into a ROS2 message.
          auto ret = rmw_deserialize(msg.get(), string_ts, string_msg.get());
          if (ret != RMW_RET_OK) {
            fprintf(stderr, "failed to deserialize serialized message\n");
            return;
          }
          // Finally print the ROS2 message data
          std::cout << "serialized data after deserialization: " << string_msg->data << std::endl;
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
  rclcpp::Subscription<rmw_serialized_message_t>::SharedPtr sub_;
  std::string topic_name_ = "chatter";
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::SerializedMessageListener)
