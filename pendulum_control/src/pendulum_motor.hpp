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

#ifndef PENDULUM_DEMO_PENDULUM_MOTOR_HPP_
#define PENDULUM_DEMO_PENDULUM_MOTOR_HPP_

#include <chrono>

#include <rttest/rttest.h>
#include <rttest/utils.h>

#include <pendulum_msgs/msg/joint_command.hpp>
#include <pendulum_msgs/msg/joint_state.hpp>

#ifndef GRAVITY
#define GRAVITY 9.80665
#endif

#ifndef PI
#define PI 3.14159265359
#endif

namespace pendulum_control
{

struct PendulumProperties
{
  double mass = 0.01;  // mass of the weight on the end of the pendulum in kilograms
  double length = 0.5;  // length of the pendulum in meters
};

struct PendulumState
{
  double position = 0;  // Angle from the ground in radians (p in diagram)
  double velocity = 0;  // Angular velocity in radians/sec
  double acceleration = 0;  // Angular acceleration in radians/sec^2
  double torque = 0;  // Torque on the joint (currently unused)
};

class PendulumMotor
{
public:
  PendulumMotor(std::chrono::nanoseconds period, PendulumProperties properties)
  : publish_period_(period), properties_(properties),
    physics_update_period_(std::chrono::nanoseconds(1000000)),
    sensor_message_(std::make_shared<pendulum_msgs::msg::JointState>()),
    message_ready_(false), done_(false)
  {
    dt_ = physics_update_period_.count() / (1000.0 * 1000.0 * 1000.0);
    long_to_timespec(physics_update_period_.count(), &physics_update_timespec_);

    pthread_attr_init(&thread_attr_);
    struct sched_param thread_param;
    thread_param.sched_priority = 90;
    pthread_attr_setschedparam(&thread_attr_, &thread_param);
    pthread_attr_setschedpolicy(&thread_attr_, SCHED_RR);
    pthread_create(&physics_update_thread_, &thread_attr_,
      &pendulum_control::PendulumMotor::physics_update_wrapper, this);
  }

  // Update forces on motor based on command
  void on_command_message(const pendulum_msgs::msg::JointCommand::SharedPtr msg)
  {
    ++messages_received;
    // Assume direct, instantaneous position control
    // TODO(jacquelinekay): do we want to simulate a motor model?
    state_.position = msg->position;

    if (state_.position > PI) {
      state_.position = PI;
    } else if (state_.position < 0) {
      state_.position = 0;
    }

    if (isnan(state_.position)) {
      throw std::runtime_error("Tried to set state to NaN in on_command_message callback");
    }
  }

  const pendulum_msgs::msg::JointState::SharedPtr get_next_sensor_message()
  {
    return sensor_message_;
  }

  bool next_message_ready() const
  {
    return message_ready_;
  }

  void set_done(bool done)
  {
    done_ = done;
  }

  bool done() const
  {
    return done_;
  }

  std::chrono::nanoseconds get_publish_period() const
  {
    return publish_period_;
  }

  double get_position() const
  {
    return state_.position;
  }

  PendulumState get_state() const
  {
    return state_;
  }

  void set_state(const PendulumState & state)
  {
    state_ = state;
  }

  const PendulumProperties & get_properties() const
  {
    return properties_;
  }

  void set_properties(const PendulumProperties & properties)
  {
    properties_ = properties;
  }

  size_t messages_received = 0;

private:
  static void * physics_update_wrapper(void * args)
  {
    PendulumMotor * motor = static_cast<PendulumMotor *>(args);
    if (!motor) {
      return NULL;
    }
    return motor->physics_update();
  }
  // Set kinematic and dynamic properties of the pendulum based on state inputs
  void * physics_update()
  {
    rttest_lock_and_prefault_dynamic();
    while (!done_) {
      state_.acceleration = GRAVITY * std::sin(state_.position - PI / 2.0) / properties_.length +
        state_.torque / (properties_.mass + properties_.length);
      state_.velocity += state_.acceleration * dt_;
      state_.position += state_.velocity * dt_;
      if (state_.position > PI) {
        state_.position = PI;
      } else if (state_.position < 0) {
        state_.position = 0;
      }

      if (isnan(state_.position)) {
        throw std::runtime_error("Tried to set state to NaN in on_command_message callback");
      }

      sensor_message_->velocity = state_.velocity;
      // Simulate a noisy sensor on position
      sensor_message_->position = state_.position;

      message_ready_ = true;
      // high resolution sleep
      clock_nanosleep(CLOCK_MONOTONIC, 0, &physics_update_timespec_, NULL);
    }
    return 0;
  }

  // motor should publish more frequently than the controller
  std::chrono::nanoseconds publish_period_;

  // Physics should update most frequently, in separate RT thread
  struct timespec physics_update_timespec_;
  double dt_;

  // Physical qualities of the pendulum
  // *INDENT-OFF* (prevent uncrustify from ruining my sweet ASCII art)
  /*
       M
        \
         \ length
       p  \
     0 ----------- pi
   */
  // *INDENT-ON*

  PendulumProperties properties_;
  PendulumState state_;

  std::chrono::nanoseconds physics_update_period_;
  pendulum_msgs::msg::JointState::SharedPtr sensor_message_;
  bool message_ready_;
  bool done_;

  pthread_t physics_update_thread_;
  pthread_attr_t thread_attr_;
};

}  /* namespace pendulum_demo */

#endif  /* PENDULUM_DEMO_PENDULUM_MOTOR_HPP_ */
