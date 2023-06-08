// Copyright 2021 Open Source Robotics Foundation, Inc.
// Copyright (c) 2020 Intel Corporation
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

#include <cinttypes>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"

// A utility class to assist in spinning a separate node
class NodeThread
{
public:
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
  : node_(node_base)
  {
    thread_ = std::make_unique<std::thread>(
      [&]()
      {
        executor_.add_node(node_);
        executor_.spin();
        executor_.remove_node(node_);
      });
  }

  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  ~NodeThread()
  {
    executor_.cancel();
    thread_->join();
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  const char * node_name = "this_node";
  const char * param_name = "an_int_param";

  // Create a node with an integer parameter
  auto node = rclcpp::Node::make_shared(node_name);
  node->declare_parameter(param_name, 0);

  // Let's create another "remote" node in a separate namespace with its own string parameter
  auto remote_node_name = "a_remote_node";
  auto remote_node_namespace = "/a_namespace";
  auto remote_param_name = "a_string_param";
  auto remote_node = rclcpp::Node::make_shared(remote_node_name, remote_node_namespace);
  remote_node->declare_parameter(remote_param_name, "default_string_value");
  auto remote_thread = std::make_unique<NodeThread>(remote_node);

  // Now, create a parameter subscriber that can be used to monitor parameter changes on
  // our own local node as well as other remote nodes
  auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(node);

  // First, set a callback for the local integer parameter. In this case, we don't
  // provide a node name (the third, optional, parameter).
  auto cb1 = [&node](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        node->get_logger(),
        "cb1: Received an update to parameter \"%s\" of type %s: \"%" PRId64 "\"",
        p.get_name().c_str(),
        p.get_type_name().c_str(),
        p.as_int());
    };
  auto handle1 = param_subscriber->add_parameter_callback(param_name, cb1);

  // Now, add a callback to monitor any changes to the remote node's parameter. In this
  // case, we supply the remote node name.
  auto cb2 = [&node](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        node->get_logger(), "cb2: Received an update to parameter \"%s\" of type: %s: \"%s\"",
        p.get_name().c_str(),
        p.get_type_name().c_str(),
        p.as_string().c_str());
    };
  auto fqn = remote_node_namespace + std::string("/") + remote_node_name;
  auto handle2 = param_subscriber->add_parameter_callback(
    remote_param_name, cb2, fqn);

  // We can also monitor all parameter changes and do our own filtering/searching
  auto cb3 =
    [fqn, remote_param_name, &node](const rcl_interfaces::msg::ParameterEvent & event) {
      // Use a regular expression to scan for any updates to parameters in "/a_namespace"
      // as well as any parameter changes to our own node
      std::regex re("(/a_namespace/.*)|(/this_node)");
      if (regex_match(event.node, re)) {
        // You can use 'get_parameter_from_event' if you know the node name and parameter name
        // that you're looking for
        rclcpp::Parameter p;
        if (rclcpp::ParameterEventHandler::get_parameter_from_event(
            event, p,
            remote_param_name, fqn))
        {
          RCLCPP_INFO(
            node->get_logger(), "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_string().c_str());
        }

        // You can also use 'get_parameter*s*_from_event' to enumerate all changes that came
        // in on this event
        auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
        for (auto & p : params) {
          RCLCPP_INFO(
            node->get_logger(), "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.value_to_string().c_str());
        }
      }
    };
  auto handle3 = param_subscriber->add_parameter_event_callback(cb3);

  printf("This demo is monitoring for the following parameter changes:\n\n");
  printf("\tnode: \"%s\"\n", node_name);
  printf("\tparameter: \"%s\"\n", param_name);
  printf("\n");
  printf("\tnode: \"%s\"\n", fqn.c_str());
  printf("\tparameter: \"%s\"\n", remote_param_name);
  printf("\n");
  printf(
    "To activate the callbacks, from a separate shell/console window, "
    "execute either of the following example commands:\n\n");
  printf("\t$ ros2 param set %s %s 21\n", node_name, param_name);
  printf("\t$ ros2 param set %s %s \"string value to set\"\n\n", fqn.c_str(), remote_param_name);

  // Process messages until ^C
  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
