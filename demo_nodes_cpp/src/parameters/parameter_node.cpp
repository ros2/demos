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

#include <memory>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

class ParameterNode : public rclcpp::Node
{
public:
  ParameterNode()
  : Node("parameter_node")
  {
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

    auto on_parameter_event_callback =
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
      {
        // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
        std::stringstream ss;
        ss << "Parameter event:\n new parameters:";
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
        RCLCPP_INFO(this->get_logger(), ss.str().c_str())
      };

    // Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
  }

private:
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<ParameterNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
