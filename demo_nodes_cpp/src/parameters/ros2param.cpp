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

#include <inttypes.h>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

#define USAGE \
  "USAGE:\n  ros2param get <node/variable>\n  ros2param set <node/variable> <value>\n" \
  "  ros2param list <node>"

typedef enum
{
  PARAM_NONE,
  PARAM_GET,
  PARAM_SET,
  PARAM_LIST,
} param_operation_t;

rclcpp::parameter::ParameterVariant
parse_args(int argc, char ** argv, std::string & remote_node, param_operation_t & op)
{
  if (argc < 3) {
    return rclcpp::parameter::ParameterVariant();
  }

  std::string verb = argv[1];
  std::string name = argv[2];

  if (verb == "list") {
    op = PARAM_LIST;
    remote_node = name;
    return rclcpp::parameter::ParameterVariant();
  }

  size_t slash = name.find('/');
  if ((slash == std::string::npos) ||
    (slash == 0) ||
    (slash == (name.size() - 1)))
  {
    return rclcpp::parameter::ParameterVariant();
  }
  remote_node = name.substr(0, slash);
  std::string variable = name.substr(slash + 1, name.size() - slash - 1);


  if ((verb == "get") && (argc == 3)) {
    op = PARAM_GET;
    return rclcpp::parameter::ParameterVariant(variable, 0);
  }
  if ((verb == "set") && (argc == 4)) {
    op = PARAM_SET;
    std::string value = argv[3];
    char * endptr;
    int l = strtol(value.c_str(), &endptr, 10);
    if ((errno == 0) && (*endptr == '\0')) {
      return rclcpp::parameter::ParameterVariant(variable, l);
    }
    errno = 0;
    double d = strtod(value.c_str(), &endptr);
    if ((errno == 0) && (*endptr == '\0')) {
      return rclcpp::parameter::ParameterVariant(variable, d);
    }
    if ((value == "true") || (value == "True")) {
      return rclcpp::parameter::ParameterVariant(variable, true);
    }
    if ((value == "false") || (value == "False")) {
      return rclcpp::parameter::ParameterVariant(variable, false);
    }
    return rclcpp::parameter::ParameterVariant(variable, value);
  }
  return rclcpp::parameter::ParameterVariant();
}

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  std::string remote_node;
  param_operation_t op = PARAM_NONE;

  auto var = parse_args(argc, argv, remote_node, op);

  if (op == PARAM_NONE) {
    fprintf(stderr, "%s\n", USAGE);
    return 1;
  }

  auto node = rclcpp::Node::make_shared("ros2param");
  auto parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node, remote_node);
  auto parameter_service = std::make_shared<rclcpp::ParameterService>(node);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.")
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...")
  }

  if (op == PARAM_GET) {
    auto get_parameters_result = parameters_client->get_parameters({var.get_name()});
    auto get_result = rclcpp::spin_until_future_complete(
      node, get_parameters_result, std::chrono::milliseconds(1000));
    if ((get_result != rclcpp::executor::FutureReturnCode::SUCCESS) ||
      (get_parameters_result.get().size() != 1) ||
      (get_parameters_result.get()[0].get_type() == rclcpp::parameter::PARAMETER_NOT_SET))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to get parameter")
      return 1;
    }

    auto result = get_parameters_result.get()[0];
    if (result.get_type() == rclcpp::parameter::PARAMETER_BOOL) {
      RCLCPP_INFO(node->get_logger(), "%s", result.get_value<bool>() ? "true" : "false")
    } else if (result.get_type() == rclcpp::parameter::PARAMETER_INTEGER) {
      RCLCPP_INFO(node->get_logger(), "%" PRId64, result.get_value<int64_t>())
    } else if (result.get_type() == rclcpp::parameter::PARAMETER_DOUBLE) {
      RCLCPP_INFO(node->get_logger(), "%f", result.get_value<double>())
    } else if (result.get_type() == rclcpp::parameter::PARAMETER_STRING) {
      RCLCPP_INFO(node->get_logger(), result.get_value<std::string>().c_str())
    } else if (result.get_type() == rclcpp::parameter::PARAMETER_BYTE_ARRAY) {
      RCLCPP_ERROR(node->get_logger(), "BYTES type not implemented")
      return 1;
    }

  } else if (op == PARAM_SET) {
    auto set_parameters_result = parameters_client->set_parameters({var});
    auto set_result = rclcpp::spin_until_future_complete(
      node, set_parameters_result, std::chrono::milliseconds(1000));
    if (set_result != rclcpp::executor::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to set parameter")
      return 1;
    }
    auto result = set_parameters_result.get().at(0);
    if (!result.successful) {
      RCLCPP_ERROR(node->get_logger(), "Error setting parameter: %s", result.reason.c_str())
    }
  } else if (op == PARAM_LIST) {
    auto list_parameters_result = parameters_client->list_parameters({}, 10);
    auto list_result = rclcpp::spin_until_future_complete(
      node, list_parameters_result, std::chrono::milliseconds(10000));
    if (list_result == rclcpp::executor::FutureReturnCode::SUCCESS) {
      std::stringstream ss;
      ss << "Node " << remote_node.c_str() << " has ";
      ss << list_parameters_result.get().names.size() << " parameters:";
      for (auto name : list_parameters_result.get().names) {
        ss << "\n" << name.c_str();
      }
      RCLCPP_INFO(node->get_logger(), ss.str().c_str())
    } else if (list_result == rclcpp::executor::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(node->get_logger(), "Timed out trying to list parameters: 10 seconds")
      return 1;
    } else {
      RCLCPP_ERROR(node->get_logger(), "Error listing parameters")
      return 1;
    }
  } else {
    fprintf(stderr, "%s\n", USAGE);
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
