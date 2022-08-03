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

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "rclcpp/serialization.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{

class SerializedMessageTalker : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit SerializedMessageTalker(const rclcpp::NodeOptions & options)
  : Node("serialized_message_talker", options),
    serialized_msg_(0u)
  {
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        // In this example we send a std_msgs/String as serialized data.
        // This is the manual CDR serialization of a string message with the content of
        // Hello World: <count_> equivalent to talker example.
        // The serialized data is composed of a 8 Byte header
        // plus the length of the actual message payload.
        // If we were to compose this serialized message by hand, we would execute the following:
        // rcutils_snprintf(
        //   serialized_msg_.buffer, serialized_msg_.buffer_length, "%c%c%c%c%c%c%c%c%s %zu",
        //   0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, "Hello World:", count_++);

        // In order to ease things up, we call the rmw_serialize function,
        // which can do the above conversion for us.
        // For this, we initially fill up a std_msgs/String message and fill up its content
        auto string_msg = std::make_shared<std_msgs::msg::String>();
        string_msg->data = "Hello World:" + std::to_string(count_++);

        // We know the size of the data to be sent, and thus can pre-allocate the
        // necessary memory to hold all the data.
        // This is specifically interesting to do here, because this means
        // no dynamic memory allocation has to be done down the stack.
        // If we don't allocate enough memory, the serialized message will be
        // dynamically allocated before sending it to the wire.
        auto message_header_length = 8u;
        auto message_payload_length = static_cast<size_t>(string_msg->data.size());
        serialized_msg_.reserve(message_header_length + message_payload_length);

        static rclcpp::Serialization<std_msgs::msg::String> serializer;
        serializer.serialize_message(string_msg.get(), &serialized_msg_);

        // For demonstration we print the ROS 2 message format
        printf("ROS message:\n");
        printf("%s\n", string_msg->data.c_str());
        // And after the corresponding binary representation
        printf("serialized message:\n");
        for (size_t i = 0; i < serialized_msg_.size(); ++i) {
          printf("%02x ", serialized_msg_.get_rcl_serialized_message().buffer[i]);
        }
        printf("\n");

        pub_->publish(serialized_msg_);
      };

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 1;
  rclcpp::SerializedMessage serialized_msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::SerializedMessageTalker)
