// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <atomic>
#include <thread>

#include "rclcpp/qos.hpp"
#include "rmw/types.h"

/// Convert rmw_time_t to seconds (represented by a floating point number).
double
rmw_time_to_seconds(const rmw_time_t & time);

/// Print the given QoS settings to stdout.
void
print_qos(const rclcpp::QoS & qos);

class CommandGetter
{
public:
  /// Whether or not the command getter is currently expecting listening for user inputs.
  bool is_active() const;

  /// Start listening for user inputs (character key presses).
  void start();

  /// Stop listening for user inputs.
  void stop();

  /// Function prototype for handling user inputs.
  virtual void handle_cmd(const char cmd) const = 0;

  /// Helper function with event loop for the command getting thread.
  void operator()() const;

private:
  /// Get a key press from running terminal
  char getch() const;

  std::thread thread_;
  std::atomic<bool> run_;
};

#endif  // UTILS_HPP_
