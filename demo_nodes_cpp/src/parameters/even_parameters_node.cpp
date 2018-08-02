// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <vector>

#include "rclcpp/rclcpp.hpp"

class EvenParameterNode : public rclcpp::Node
{
public:
  EvenParameterNode()
  : Node("even_parameters_node")
  {
    // Declare a parameter change request callback
    // This function will enforce that only setting even integer parameters is allowed
    // any other change will be discarded
    auto param_change_callback =
      [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto parameter : parameters) {
          rclcpp::ParameterType parameter_type = parameter.get_type();
          if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
            RCLCPP_INFO(this->get_logger(),
              "parameter '%s' deleted successfully",
              parameter.get_name().c_str()
            );
            result.successful &= true;
          } else if (rclcpp::ParameterType::PARAMETER_INTEGER == parameter_type) {
            if (parameter.as_int() % 2 != 0) {
              RCLCPP_INFO(this->get_logger(),
                "Requested value '%d' for parameter '%s' is not an even number:"
                " rejecting change...",
                parameter.as_int(),
                parameter.get_name().c_str()
              );
              result.successful = false;
            } else {
              RCLCPP_INFO(this->get_logger(),
                "parameter '%s' has changed and is now: %s",
                parameter.get_name().c_str(),
                parameter.value_to_string().c_str()
              );
              result.successful &= true;
            }
          } else {
            RCLCPP_INFO(this->get_logger(),
              "only integer parameters can be set\n"
              "requested value for parameter '%s' is not an even number, rejecting change...",
              parameter.get_name().c_str()
            );
            result.successful = false;
          }
        }
        return result;
      };
    this->register_param_change_callback(param_change_callback);
  }
};

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<EvenParameterNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
