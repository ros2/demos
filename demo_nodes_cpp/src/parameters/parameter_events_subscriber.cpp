// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>

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

  // Create a node with an integer parameter
  auto node = rclcpp::Node::make_shared("this_node");
  node->declare_parameter("an_int_param", 0);

  // Let's create another "remote" node in a namespace with its own string parameter
  auto remote_node_name = "a_remote_node";
  auto remote_node_namespace = "/a_namespace";
  auto remote_param_name = "a_string_param";
  auto remote_node = rclcpp::Node::make_shared(remote_node_name, remote_node_namespace);
  remote_node->declare_parameter(remote_param_name, "default_string_value");
  auto remote_thread = std::make_unique<NodeThread>(remote_node);

  // Now, create a parameter subscriber that can be used to monitor parameter changes on
  // our own local node as well as other remote nodes
  auto param_subscriber = std::make_shared<rclcpp::ParameterEventsSubscriber>(node);

  // First, set a callback for the local integer parameter. In this case, we don't
  // provide a node name (the third, optional, parameter).
  auto cb1 = [&node](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        node->get_logger(), "Received an update to parameter \"%s\" of type %s: %ld",
        p.get_name().c_str(),
        p.get_type_name().c_str(),
        p.as_int());
    };
  auto handle1 = param_subscriber->add_parameter_callback("an_int_param", cb1);

  // Now, add a callback to monitor any changes to the remote node's parameter. In this
  // case, we supply the remote node name.
  auto cb2 = [&node](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        node->get_logger(), "Received an update to parameter \"%s\" of type: %s: \"%s\"",
        p.get_name().c_str(),
        p.get_type_name().c_str(),
        p.as_string().c_str());
    };
  auto fqn = remote_node_namespace + std::string("/") + remote_node_name;
  auto handle2 = param_subscriber->add_parameter_callback(
    remote_param_name, cb2, fqn);

  // Process messages until ^C
  rclcpp::spin(node->get_node_base_interface());

  // Remove the callbacks and shut down
  param_subscriber->remove_parameter_callback(handle1.get());
  param_subscriber->remove_parameter_callback(handle2.get());

  rclcpp::shutdown();
  return 0;
}
