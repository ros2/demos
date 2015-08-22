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

#ifndef PENDULUM_DEMO_PENDULUM_CONTROLLER_HPP_
#define PENDULUM_DEMO_PENDULUM_CONTROLLER_HPP_

#include <chrono>

#include <pendulum_msgs/msg/joint_command.hpp>
#include <pendulum_msgs/msg/joint_state.hpp>

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum_control
{

struct PIDProperties
{
  // Properties of a PID controller
  double p = 1;
  double i = 0;
  double d = 0;
  double command = PI / 2;
};

class PendulumController
{
public:
  PendulumController(std::chrono::nanoseconds period, PIDProperties pid)
  : publish_period_(period), pid_(pid),
    command_message_(std::make_shared<pendulum_msgs::msg::JointCommand>()),
    message_ready_(false)
  {
    command_message_->position = pid_.command;
    dt_ = publish_period_.count() / (1000.0 * 1000.0 * 1000.0);
    if (isnan(dt_) || dt_ == 0) {
      throw std::runtime_error("Invalid dt_ calculated in PendulumController constructor");
    }
  }

  // Calculate new command based on new sensor state and PID controller properties
  void on_sensor_message(const pendulum_msgs::msg::JointState::SharedPtr msg)
  {
    ++messages_received;

    if (isnan(msg->position)) {
      throw std::runtime_error("Sensor value was NaN in on_sensor_message callback");
    }
    double error = pid_.command - msg->position;
    double p_gain = pid_.p * error;
    i_gain_ = pid_.i * (i_gain_ + error * dt_);
    double d_gain = pid_.d * (error - last_error_) / dt_;
    last_error_ = error;

    // TODO consider filtering the PID output
    command_message_->position = msg->position + p_gain + i_gain_ + d_gain;
    // limits
    if (command_message_->position > PI) {
      command_message_->position = PI;
    } else if (command_message_->position < 0) {
      command_message_->position = 0;
    }

    if (isnan(command_message_->position)) {
      throw std::runtime_error("Resulting command was NaN in on_sensor_message callback");
    }
    message_ready_ = true;
  }

  const pendulum_msgs::msg::JointCommand::SharedPtr get_next_command_message()
  {
    return command_message_;
  }

  bool next_message_ready() const
  {
    return message_ready_;
  }

  std::chrono::nanoseconds get_publish_period() const
  {
    return publish_period_;
  }

  void set_pid_properties(const PIDProperties & properties)
  {
    pid_ = properties;
  }

  const PIDProperties & get_pid_properties() const
  {
    return pid_;
  }

  void set_command(double command)
  {
    pid_.command = command;
  }

  double get_command() const
  {
    return pid_.command;
  }

  // gather statistics
  size_t messages_received = 0;

private:
  // controller should publish less frequently than the motor
  std::chrono::nanoseconds publish_period_;
  PIDProperties pid_;
  pendulum_msgs::msg::JointCommand::SharedPtr command_message_;
  bool message_ready_;

  // state for PID controller
  double last_error_ = 0;
  double i_gain_ = 0;
  double dt_;
};

}  /* namespace pendulum_demo */

#endif  /* PENDULUM_DEMO_PENDULUM_CONTROLLER_HPP_ */
