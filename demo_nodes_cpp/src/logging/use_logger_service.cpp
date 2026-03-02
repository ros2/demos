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

#include "example_interfaces/msg/string.hpp"

#include "rcl_interfaces/srv/get_logger_levels.hpp"
#include "rcl_interfaces/srv/set_logger_levels.hpp"

using namespace std::chrono_literals;

// This demo program shows how to enable logger service and control logger level via logger service.
// Class LoggerServiceNode enables logger service, creates a child logger, and subscribes to a
// topic. The subscription callback outputs received messages using both the parent and child
// loggers at different severity levels, demonstrating how child loggers inherit or independently
// override the parent logger level.
// Class TestNode can set/get logger level of LoggerServiceNode and send message to it.
//
// Usage:
//   Default (demo mode): runs the full automated demo sequence with TestNode.
//   --service-only:      starts only LoggerServiceNode and spins forever,
//                         so you can interact with it via ros2 CLI tools.
//
//   Example ros2 CLI commands (use in a separate terminal while --service-only is running).
//   Note: each multi-line command below must be joined into a single line before executing.
//
//   Get logger level (parent):
//     ros2 service call /LoggerServiceNode/get_logger_levels
//       rcl_interfaces/srv/GetLoggerLevels "{names: ['LoggerServiceNode']}"
//
//   Get logger level (child):
//     ros2 service call /LoggerServiceNode/get_logger_levels
//       rcl_interfaces/srv/GetLoggerLevels "{names: ['LoggerServiceNode.child']}"
//
//   Set parent logger level to DEBUG (10):
//     ros2 service call /LoggerServiceNode/set_logger_levels
//       rcl_interfaces/srv/SetLoggerLevels
//       "{levels: [{name: 'LoggerServiceNode', level: 10}]}"
//
//   Set child logger level to ERROR (40):
//     ros2 service call /LoggerServiceNode/set_logger_levels
//       rcl_interfaces/srv/SetLoggerLevels
//       "{levels: [{name: 'LoggerServiceNode.child', level: 40}]}"
//
//   Publish a test message:
//     ros2 topic pub /output example_interfaces/msg/String "{data: hello}" --once

class LoggerServiceNode : public rclcpp::Node
{
public:
  explicit LoggerServiceNode(const std::string & node_name)
  : Node(node_name, rclcpp::NodeOptions().enable_logger_service(true)),
    child_logger_(this->get_logger().get_child("child"))
  {
    auto callback = [this](example_interfaces::msg::String::ConstSharedPtr msg)-> void {
        RCLCPP_DEBUG(this->get_logger(), "%s with DEBUG logger level.", msg->data.c_str());
        RCLCPP_INFO(this->get_logger(), "%s with INFO logger level.", msg->data.c_str());
        RCLCPP_WARN(this->get_logger(), "%s with WARN logger level.", msg->data.c_str());
        RCLCPP_ERROR(this->get_logger(), "%s with ERROR logger level.", msg->data.c_str());
        RCLCPP_DEBUG(child_logger_, "[child] %s with DEBUG logger level.", msg->data.c_str());
        RCLCPP_INFO(child_logger_, "[child] %s with INFO logger level.", msg->data.c_str());
        RCLCPP_WARN(child_logger_, "[child] %s with WARN logger level.", msg->data.c_str());
        RCLCPP_ERROR(child_logger_, "[child] %s with ERROR logger level.", msg->data.c_str());
      };

    sub_ = this->create_subscription<example_interfaces::msg::String>("output", 10, callback);
  }

private:
  rclcpp::Logger child_logger_;
  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr sub_;
};

class TestNode : public rclcpp::Node
{
public:
  explicit TestNode(const std::string & remote_node_name)
  : Node("TestNode"),
    remote_node_name_(remote_node_name)
  {
    pub_ = this->create_publisher<example_interfaces::msg::String>("output", 10);
    logger_set_client_ = this->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      remote_node_name + "/set_logger_levels");
    logger_get_client_ = this->create_client<rcl_interfaces::srv::GetLoggerLevels>(
      remote_node_name + "/get_logger_levels");
  }

  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr get_pub()
  {
    return pub_;
  }

  bool set_logger_level_on_remote_node(
    rclcpp::Logger::Level logger_level,
    const std::string & logger_name = "")
  {
    if (!logger_set_client_->wait_for_service(2s)) {
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    auto set_logger_level = rcl_interfaces::msg::LoggerLevel();
    set_logger_level.name = logger_name.empty() ? remote_node_name_ : logger_name;
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

  bool get_logger_level_on_remote_node(
    uint32_t & level,
    const std::string & logger_name = "")
  {
    if (!logger_get_client_->wait_for_service(2s)) {
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetLoggerLevels::Request>();
    request->names.emplace_back(logger_name.empty() ? remote_node_name_ : logger_name);
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
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr pub_;
  rclcpp::Client<rcl_interfaces::srv::SetLoggerLevels>::SharedPtr logger_set_client_;
  rclcpp::Client<rcl_interfaces::srv::GetLoggerLevels>::SharedPtr logger_get_client_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Check for --service-only flag before ROS 2 consumes the arguments
  bool service_only = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--service-only") {
      service_only = true;
      break;
    }
  }

  const std::string node_name = "LoggerServiceNode";
  auto logger_service_node = std::make_shared<LoggerServiceNode>(
    node_name);

  if (service_only) {
    // Service-only mode: spin LoggerServiceNode forever so that users can
    // interact with it using ros2 CLI tools (ros2 service, ros2 topic, etc.)
    RCLCPP_INFO(
      logger_service_node->get_logger(),
      "Started in service-only mode. Use ros2 CLI to interact with this node.");
    rclcpp::spin(logger_service_node);
    rclcpp::shutdown();
    return 0;
  }

  // Default demo mode: run the full automated sequence
  auto test_node = std::make_shared<TestNode>(node_name);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(logger_service_node);

  std::thread thread([&executor]() {
      executor.spin();
    });

  const std::string child_logger_name = node_name + ".child";

  auto get_logger_level_func = [&test_node, &child_logger_name] {
      uint32_t get_logger_level = 0;
      if (test_node->get_logger_level_on_remote_node(get_logger_level)) {
        RCLCPP_INFO(test_node->get_logger(), "Current logger level: %u", get_logger_level);
      } else {
        RCLCPP_ERROR(
          test_node->get_logger(),
          "Failed to get logger level via logger service !");
      }
      uint32_t child_level = 0;
      if (test_node->get_logger_level_on_remote_node(child_level, child_logger_name)) {
        RCLCPP_INFO(
          test_node->get_logger(), "Current child logger level: %u", child_level);
      } else {
        RCLCPP_ERROR(
          test_node->get_logger(),
          "Failed to get child logger level via logger service !");
      }
    };

  // Output with default logger level
  RCLCPP_INFO(test_node->get_logger(), "Output with default logger level:");
  {
    auto msg = std::make_unique<example_interfaces::msg::String>();
    msg->data = "Output 1";
    test_node->get_pub()->publish(std::move(msg));
  }
  std::this_thread::sleep_for(200ms);

  // Get logger level. Logger level should be 0 (Unset)
  get_logger_level_func();

  // Output with parent=debug, child=error logger level
  // Parent shows all messages, child only shows error.
  RCLCPP_INFO(
    test_node->get_logger(), "Output with parent=debug, child=error logger level:");
  if (test_node->set_logger_level_on_remote_node(rclcpp::Logger::Level::Debug) &&
    test_node->set_logger_level_on_remote_node(
      rclcpp::Logger::Level::Error, child_logger_name))
  {
    auto msg = std::make_unique<example_interfaces::msg::String>();
    msg->data = "Output 2";
    test_node->get_pub()->publish(std::move(msg));
    std::this_thread::sleep_for(200ms);
  } else {
    RCLCPP_ERROR(test_node->get_logger(), "Failed to set logger levels via logger service !");
  }

  // Parent should be 10 (Debug), child should be 40 (Error)
  get_logger_level_func();

  // Output with parent=warn, child=debug logger level
  // Parent suppresses debug/info, child shows everything.
  RCLCPP_INFO(
    test_node->get_logger(), "Output with parent=warn, child=debug logger level:");
  if (test_node->set_logger_level_on_remote_node(rclcpp::Logger::Level::Warn) &&
    test_node->set_logger_level_on_remote_node(
      rclcpp::Logger::Level::Debug, child_logger_name))
  {
    auto msg = std::make_unique<example_interfaces::msg::String>();
    msg->data = "Output 3";
    test_node->get_pub()->publish(std::move(msg));
    std::this_thread::sleep_for(200ms);
  } else {
    RCLCPP_ERROR(test_node->get_logger(), "Failed to set logger levels via logger service !");
  }

  // Parent should be 30 (Warn), child should be 10 (Debug)
  get_logger_level_func();

  // Output with parent=error, child=debug logger level
  // Parent only shows error, child shows everything.
  RCLCPP_INFO(
    test_node->get_logger(), "Output with parent=error, child=debug logger level:");
  if (test_node->set_logger_level_on_remote_node(rclcpp::Logger::Level::Error) &&
    test_node->set_logger_level_on_remote_node(
      rclcpp::Logger::Level::Debug, child_logger_name))
  {
    auto msg = std::make_unique<example_interfaces::msg::String>();
    msg->data = "Output 4";
    test_node->get_pub()->publish(std::move(msg));
    std::this_thread::sleep_for(200ms);
  } else {
    RCLCPP_ERROR(test_node->get_logger(), "Failed to set logger levels via logger service !");
  }

  // Parent should be 40 (Error), child should be 10 (Debug)
  get_logger_level_func();

  executor.cancel();
  if (thread.joinable()) {
    thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
