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

#ifndef IMAGE_TOOLS__OPTIONS_HPP_
#define IMAGE_TOOLS__OPTIONS_HPP_

#include <string>
#include <vector>

#include "rmw/types.h"

/// Find "option" in the argument vector.
/**
 * \param[in] args The argument vector
 * \param[in] option The option to search for
 * \return True if option was found in args, false otherwise.
 */
bool find_command_option(
  const std::vector<std::string> & args, const std::string & option);

/// Get the value corresponding to option.
/**
 * \param[in] args The argument vector to search in
 * \param[in] option The option to search for
 * \return The value that comes after "option"
 */
std::string get_command_option(
  const std::vector<std::string> & args, const std::string & option);

/// Parse the C-style argument vector and return demo-specific parameters.
/**
 * \param[in] argc Size of the argument vector.
 * \param[in] argv Argument vector, an array of C-style strings.
 * \param[in] depth The queue size for the KEEP_LAST QoS policy.
 * \param[in] reliability_policy The reliability policy (RELIABLE or BEST_EFFORT).
 * \param[in] show_camera True to show the input stream, false or NULL otherwise.
 * \param[in] freq The frequency at which to publish images.
 * \param[in] width The width of the image to get, 320 by default.
 * \param[in] height The height of the image to get, 240 by default.
 * \param[in] burger_mode If true, produce images of burgers rather than use a camera.
 */
bool parse_command_options(
  int argc, char ** argv, size_t * depth,
  rmw_qos_reliability_policy_t * reliability_policy,
  rmw_qos_history_policy_t * history_policy, bool * show_camera = nullptr, double * freq = nullptr,
  size_t * width = nullptr, size_t * height = nullptr, bool * burger_mode = nullptr,
  std::string * topic = nullptr);

#endif  // IMAGE_TOOLS__OPTIONS_HPP_
