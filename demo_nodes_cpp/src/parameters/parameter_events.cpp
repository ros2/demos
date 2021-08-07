// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <cstring>
#include <future>
#include <memory>
#include <sstream>
#include <utility>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

bool on_parameter_event(
  rcl_interfaces::msg::ParameterEvent::UniquePtr event, rclcpp::Logger logger)
{
  // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
  std::stringstream ss;
  // ignore qos overrides
  event->new_parameters.erase(
    std::remove_if(
      event->new_parameters.begin(),
      event->new_parameters.end(),
      [](const auto & item) {
        const char * param_override_prefix = "qos_overrides.";
        return std::strncmp(
          item.name.c_str(), param_override_prefix, sizeof(param_override_prefix) - 1) == 0u;
      }),
    event->new_parameters.end());
  if (
    !event->new_parameters.size() && !event->changed_parameters.size() &&
    !event->deleted_parameters.size())
  {
    return false;
  }
  ss << "\nParameter event:\n new parameters:";
  for (auto & new_parameter : event->new_parameters) {
    ss << "\n  " << new_parameter.name;
  }
  ss << "\n changed parameters:";
  for (auto & changed_parameter : event->changed_parameters) {
    ss << "\n  " << changed_parameter.name;
  }
  ss << "\n deleted parameters:";
  for (auto & deleted_parameter : event->deleted_parameters) {
    ss << "\n  " << deleted_parameter.name;
  }
  ss << "\n";
  RCLCPP_INFO(logger, "%s", ss.str().c_str());
  return true;
}

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("parameter_events");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto events_received_promise = std::make_shared<std::promise<void>>();
  auto events_received_future = events_received_promise->get_future();

  // Setup callback for changes to parameters.
  auto sub = parameters_client->on_parameter_event(
    [node, promise = std::move(events_received_promise)](
      rcl_interfaces::msg::ParameterEvent::UniquePtr event) -> void
    {
      static size_t n_times_called = 0u;
      if (on_parameter_event(std::move(event), node->get_logger())) {
        ++n_times_called;
      }
      if (10u == n_times_called) {
        // This callback will be called 10 times, set the promise when that happens.
        promise->set_value();
      }
    });

  // Declare parameters that may be set on this node
  node->declare_parameter("foo", 0);
  node->declare_parameter("bar", "");
  node->declare_parameter("baz", 0.);
  node->declare_parameter("foobar", false);

  // Set several different types of parameters.
  auto set_parameters_results = parameters_client->set_parameters(
  {
    rclcpp::Parameter("foo", 2),
    rclcpp::Parameter("bar", "hello"),
    rclcpp::Parameter("baz", 1.45),
    rclcpp::Parameter("foobar", true),
  });

  // Change the value of some of them.
  set_parameters_results = parameters_client->set_parameters(
  {
    rclcpp::Parameter("foo", 3),
    rclcpp::Parameter("bar", "world"),
  });

  // TODO(wjwwood): Create and use delete_parameter

  rclcpp::spin_until_future_complete(node, events_received_future.share());
  rclcpp::shutdown();

  return 0;
}
