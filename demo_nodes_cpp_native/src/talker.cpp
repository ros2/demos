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

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_publisher.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker_native")
  {
    {
      rcl_node_t * rcl_node = get_node_base_interface()->get_rcl_node_handle();
      rmw_node_t * rmw_node = rcl_node_get_rmw_handle(rcl_node);
      eprosima::fastrtps::Participant * p = rmw_fastrtps_cpp::get_participant(rmw_node);
      RCLCPP_INFO(
        this->get_logger(), "eprosima::fastrtps::Participant * %zu", reinterpret_cast<size_t>(p));
    }

    msg_ = std::make_shared<std_msgs::msg::String>();
    auto publish =
      [this]() -> void
      {
        msg_->data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
        pub_->publish(msg_);
      };
    timer_ = create_wall_timer(500ms, publish);
    pub_ = create_publisher<std_msgs::msg::String>("chatter");

    {
      rcl_publisher_t * rcl_pub = pub_->get_publisher_handle();
      rmw_publisher_t * rmw_pub = rcl_publisher_get_rmw_handle(rcl_pub);
      eprosima::fastrtps::Publisher * p = rmw_fastrtps_cpp::get_publisher(rmw_pub);
      RCLCPP_INFO(
        this->get_logger(), "eprosima::fastrtps::Publisher * %zu", reinterpret_cast<size_t>(p));
    }
  }

private:
  size_t count_ = 1;
  std::shared_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Talker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
