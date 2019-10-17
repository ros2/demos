// Copyright 2019 Open Source Robotics Foundation, Inc.
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
#ifndef POLICY_MAPS_HPP_
#define POLICY_MAPS_HPP_

#include <map>
#include <string>

namespace image_tools
{

static
std::map<std::string, rmw_qos_reliability_policy_t> name_to_reliability_policy_map = {
  {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
  {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

static
std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
  {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
  {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}
};

}  // namespace image_tools

#endif  // POLICY_MAPS_HPP_
