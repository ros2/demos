// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"

static constexpr auto chatter_topic = "lifecycle_chatter";

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name, intra_process_comms)
  {}

  void publish()
  {
    if (!pub_->is_activated())
    {
      printf("Lifecycle publisher is currently inactive. Messages are not published.\n");
    }
    static size_t count = 0;
    msg_->data = "Lifecycle HelloWorld #" + std::to_string(++count);
    pub_->publish(msg_);
  }

  rcl_lifecycle_ret_t on_configure(const rclcpp_lifecycle::State &)
  {
    // initialize communication entities
    msg_ = std::make_shared<std_msgs::msg::String>();
    pub_ = this->create_publisher<std_msgs::msg::String>(chatter_topic);
    timer_ = this->create_wall_timer(
      1_s, std::bind(&LifecycleTalker::publish, this));

    printf("[%s] on_configure() is called.\n", get_name());
    return RCL_LIFECYCLE_RET_OK;
  }

  rcl_lifecycle_ret_t on_activate(const rclcpp_lifecycle::State &)
  {
    pub_->on_activate();
    printf("[%s] on_activate() is called.\n", get_name());
    return RCL_LIFECYCLE_RET_OK;
  }

  rcl_lifecycle_ret_t on_deactivate(const rclcpp_lifecycle::State &)
  {
    pub_->on_deactivate();
    printf("[%s] on_deactivate() is called.\n", get_name());
    return RCL_LIFECYCLE_RET_OK;
  }

  rcl_lifecycle_ret_t on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    pub_.reset();
    printf("[%s] on cleanup is called.\n", get_name());
    return RCL_LIFECYCLE_RET_OK;
  }

private:
  std::shared_ptr<std_msgs::msg::String> msg_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<LifecycleTalker> lc_node =
    std::make_shared<LifecycleTalker>("lc_talker");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  return 0;
}
