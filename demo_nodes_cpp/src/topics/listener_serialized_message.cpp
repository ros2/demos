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

#include <cstdio>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

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
    // We create a callback to a rmw_serialized_message_t here. This will pass a serialized
    // message to the callback. We can then further deserialize it and convert it into
    // a ros2 compliant message.
    auto callback =
      [](const std::shared_ptr<rclcpp::SerializedMessage> msg) -> void
      {
        // Print the serialized data message in HEX representation
        // This output corresponds to what you would see in e.g. Wireshark
        // when tracing the RTPS packets.
        std::cout << "I heard data of length: " << msg->size() << std::endl;
        for (size_t i = 0; i < msg->size(); ++i) {
          printf("%02x ", msg->get_rcl_serialized_message().buffer[i]);
        }
        printf("\n");

        // In order to deserialize the message we have to manually create a ROS 2
        // message in which we want to convert the serialized data.
        using MessageT = std_msgs::msg::String;
        MessageT string_msg;
        auto serializer = rclcpp::Serialization<MessageT>();
        serializer.deserialize_message(msg.get(), &string_msg);
        // Finally print the ROS 2 message data
        std::cout << "serialized data after deserialization: " << string_msg.data << std::endl;
      };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::SerializedMessageListener)
