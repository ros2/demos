// Copyright 2023 Sony Group Corporation.
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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "rcl_interfaces/srv/get_logger_levels.hpp"
#include "rcl_interfaces/srv/set_logger_levels.hpp"

using namespace std::chrono_literals;

// This demo program shows how to enable logger service and control logger level via logger service.
// Class LoggerServiceNode enable logger service and create a subscription. The callback of
// subscription output received message by different log functions.
// Class TestNode can set/get logger level of LoggerServiceNode and send message to it.

class LoggerServiceNode : public rclcpp::Node
{
public:
  explicit LoggerServiceNode(const std::string & node_name)
  : Node(node_name, rclcpp::NodeOptions().enable_logger_service(true))
  {
    auto callback = [this](std_msgs::msg::String::ConstSharedPtr msg)-> void {
        RCLCPP_DEBUG(this->get_logger(), "%s with DEBUG logger level.", msg->data.c_str());
        RCLCPP_INFO(this->get_logger(), "%s with INFO logger level.", msg->data.c_str());
        RCLCPP_WARN(this->get_logger(), "%s with WARN logger level.", msg->data.c_str());
        RCLCPP_ERROR(this->get_logger(), "%s with ERROR logger level.", msg->data.c_str());
      };

    sub_ = this->create_subscription<std_msgs::msg::String>("output", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

class TestNode : public rclcpp::Node
{
public:
  explicit TestNode(const std::string & remote_node_name)
  : Node("TestNode"),
    remote_node_name_(remote_node_name)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("output", 10);
    logger_set_client_ = this->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      remote_node_name + "/set_logger_levels");
    logger_get_client_ = this->create_client<rcl_interfaces::srv::GetLoggerLevels>(
      remote_node_name + "/get_logger_levels");
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_pub()
  {
    return pub_;
  }

  bool set_logger_level_on_remote_node(
    rclcpp::Logger::Level logger_level)
  {
    if (!logger_set_client_->wait_for_service(2s)) {
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    auto set_logger_level = rcl_interfaces::msg::LoggerLevel();
    set_logger_level.name = remote_node_name_;
    set_logger_level.level = static_cast<uint32_t>(logger_level);
    request->levels.emplace_back(set_logger_level);

    auto result = logger_set_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }

    auto ret_result = result.get();
    if (!ret_result->results[0].successful) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to change logger level: %s",
        ret_result->results[0].reason.c_str());
      return false;
    }
    return true;
  }

  bool get_logger_level_on_remote_node(uint32_t & level)
  {
    if (!logger_get_client_->wait_for_service(2s)) {
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetLoggerLevels::Request>();
    request->names.emplace_back(remote_node_name_);
    auto result = logger_get_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }

    auto ret_result = result.get();
    level = ret_result->levels[0].level;
    return true;
  }

private:
  const std::string remote_node_name_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Client<rcl_interfaces::srv::SetLoggerLevels>::SharedPtr logger_set_client_;
  rclcpp::Client<rcl_interfaces::srv::GetLoggerLevels>::SharedPtr logger_get_client_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::string node_name = "LoggerServiceNode";
  auto logger_service_node = std::make_shared<LoggerServiceNode>(
    node_name);
  auto test_node = std::make_shared<TestNode>(node_name);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(logger_service_node);

  std::thread thread([&executor]() {
      executor.spin();
    });

  auto get_logger_level_func = [&test_node] {
      uint32_t get_logger_level = 0;
      if (test_node->get_logger_level_on_remote_node(get_logger_level)) {
        RCLCPP_INFO(test_node->get_logger(), "Current logger level: %u", get_logger_level);
      } else {
        RCLCPP_ERROR(
          test_node->get_logger(),
          "Failed to get logger level via logger service !");
      }
    };

  // Output with default logger level
  RCLCPP_INFO(test_node->get_logger(), "Output with default logger level:");
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Output 1";
    test_node->get_pub()->publish(std::move(msg));
  }
  std::this_thread::sleep_for(200ms);

  // Get logger level. Logger level should be 0 (Unset)
  get_logger_level_func();

  // Output with debug logger level
  RCLCPP_INFO(test_node->get_logger(), "Output with debug logger level:");
  if (test_node->set_logger_level_on_remote_node(rclcpp::Logger::Level::Debug)) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Output 2";
    test_node->get_pub()->publish(std::move(msg));
    std::this_thread::sleep_for(200ms);
  } else {
    RCLCPP_ERROR(test_node->get_logger(), "Failed to set debug logger level via logger service !");
  }

  // Get logger level. Logger level should be 10 (Debug)
  get_logger_level_func();

  // Output with warn logger level
  RCLCPP_INFO(test_node->get_logger(), "Output with warn logger level:");
  if (test_node->set_logger_level_on_remote_node(rclcpp::Logger::Level::Warn)) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Output 3";
    test_node->get_pub()->publish(std::move(msg));
    std::this_thread::sleep_for(200ms);
  } else {
    RCLCPP_ERROR(test_node->get_logger(), "Failed to set warn logger level via logger service !");
  }

  // Get logger level. Logger level should be 30 (Warn)
  get_logger_level_func();

  // Output with error logger level
  RCLCPP_INFO(test_node->get_logger(), "Output with error logger level:");
  if (test_node->set_logger_level_on_remote_node(rclcpp::Logger::Level::Error)) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Output 4";
    test_node->get_pub()->publish(std::move(msg));
    std::this_thread::sleep_for(200ms);
  } else {
    RCLCPP_ERROR(test_node->get_logger(), "Failed to set error logger level via logger service !");
  }

  // Get logger level. Logger level should be 40 (Error)
  get_logger_level_func();

  executor.cancel();
  if (thread.joinable()) {
    thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
