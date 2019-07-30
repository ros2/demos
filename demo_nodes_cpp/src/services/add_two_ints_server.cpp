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

#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/srv/add_two_ints.hpp"

#include "demo_nodes_cpp/visibility_control.h"
namespace demo_nodes_cpp
{

class ServerNode : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit ServerNode(const rclcpp::NodeOptions & options)
  : Node("add_two_ints_server", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    std::vector<std::string> args = options.arguments();
    if (find_command_option(args, "-h")) {
      print_usage();
      rclcpp::shutdown();
    } else {
      std::string tmptopic = get_command_option(args, "-s");
      if (!tmptopic.empty()) {
        service_name_ = tmptopic;
      }
      execute();
    }
    // Create a callback function for when service requests are received.
  }

  DEMO_NODES_CPP_PUBLIC
  void print_usage()
  {
    printf("Uage for add_two_ints_server app:\n");
    printf("add_two_ints_server [-s service_name] [-h]\n");
    printf("options:\n");
    printf("-h : Print this help function\n");
    printf("-s service_name : Specify the service name for this sever. Defaults to add_two_ints\n");
  }

  DEMO_NODES_CPP_PUBLIC
  void execute()
  {
    auto handle_add_two_ints =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64,
          request->a, request->b);
        response->sum = request->a + request->b;
      };
    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<example_interfaces::srv::AddTwoInts>(service_name_, handle_add_two_ints);
  }

  DEMO_NODES_CPP_PUBLIC
  bool find_command_option(const std::vector<std::string> & args, const std::string & option)
  {
    return std::find(args.begin(), args.end(), option) != args.end();
  }

  DEMO_NODES_CPP_PUBLIC
  std::string get_command_option(const std::vector<std::string> & args, const std::string & option)
  {
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end() && ++it != args.end()) {
      return *it;
    }
    return std::string();
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
  std::string service_name_ = "add_two_ints";
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ServerNode)
