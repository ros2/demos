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

#include "cbg_executor_demo/parameter_helper.hpp"

namespace cbg_executor_demo
{

std::chrono::nanoseconds get_nanos_from_secs_parameter(rclcpp::Node* node, std::string name) {
    double seconds = 0.0;
    node->get_parameter(name, seconds);
    auto nanos = std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1000000000.0));
    return nanos;
}

}  // namespace cbg_executor_demo
