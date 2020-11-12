// Copyright (c) 2020 Robert Bosch GmbH
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

#ifndef CBG_EXECUTOR_DEMO__PARAMETER_HELPER_HPP_
#define CBG_EXECUTOR_DEMO__PARAMETER_HELPER_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace cbg_executor_demo
{

std::chrono::nanoseconds get_nanos_from_secs_parameter(rclcpp::Node* node, std::string name);

}  // namespace cbg_executor_demo

#endif  // CBG_EXECUTOR_DEMO__PONGNODE_HPP_
