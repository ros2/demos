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

#include <rclcpp/memory_strategies.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>

#include <rttest/rttest.h>

#include <pendulum_msgs/msg/joint_command.hpp>
#include <pendulum_msgs/msg/joint_state.hpp>

#include "pendulum_controller.hpp"
#include "pendulum_motor.hpp"
#include "rtt_executor.hpp"

struct OutputThreadArgs
{
  std::shared_ptr<pendulum_control::RttExecutor> executor;
  std::shared_ptr<pendulum_control::PendulumMotor> pendulum_motor;
  std::shared_ptr<pendulum_control::PendulumController> pendulum_controller;

  OutputThreadArgs(std::shared_ptr<pendulum_control::RttExecutor> exec,
    std::shared_ptr<pendulum_control::PendulumMotor> motor,
    std::shared_ptr<pendulum_control::PendulumController> controller)
  : executor(exec), pendulum_motor(motor), pendulum_controller(controller) {}
};

void * live_output_thread(void * args)
{
  OutputThreadArgs * ptr_args = static_cast<OutputThreadArgs *>(args);
  auto executor = ptr_args->executor;
  auto pendulum_motor = ptr_args->pendulum_motor;
  auto pendulum_controller = ptr_args->pendulum_controller;

  // background task: don't want to interfere with performance
  if (rttest_set_sched_priority(0, SCHED_IDLE) != 0) {
    perror("Couldn't set priority of output thread to IDLE");
  }

  struct rttest_results stats;

  while (executor->is_rttest_ready()) {
    // Collect statistics
    if (executor->is_running()) {
      executor->get_rtt_results(stats);

      printf("Commanded motor angle: %f\n", pendulum_controller->get_command());
      printf("Actual motor angle: %f\n", pendulum_motor->get_position());

      printf("Mean latency: %f ns\n", stats.mean_latency);
      printf("Min latency: %d ns\n", stats.min_latency);
      printf("Max latency: %d ns\n", stats.max_latency);

      printf("Minor pagefaults during execution: %lu\n", stats.minor_pagefaults);
      printf("Major pagefaults during execution: %lu\n\n", stats.major_pagefaults);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return 0;
}

using namespace rclcpp::strategies::message_pool_memory_strategy;
using rclcpp::memory_strategies::static_memory_strategy::StaticMemoryStrategy;

int main(int argc, char * argv[])
{
  // Initialization phase

  // TODO(jacquelinekay) runtime tuning of controller properties and physical characteristics
  pendulum_control::PendulumProperties properties;
  pendulum_control::PIDProperties pid;

  auto pendulum_motor = std::make_shared<pendulum_control::PendulumMotor>(
    std::chrono::nanoseconds(970000), properties);
  auto pendulum_controller = std::make_shared<pendulum_control::PendulumController>(
    std::chrono::nanoseconds(960000), pid);

  rttest_read_args(argc, argv);
  rclcpp::init(argc, argv);

  rclcpp::memory_strategies::static_memory_strategy::ObjectPoolBounds bounds;
  auto memory_strategy = std::make_shared<StaticMemoryStrategy>(bounds);
  auto state_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointState, 1>>();
  auto command_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointCommand, 1>>();

  // The controller node represents user code
  auto controller_node = rclcpp::node::Node::make_shared("pendulum_controller");

  // The "motor" node simulates motors and sensors
  // It provides sensor data and changes the physical model based on the command
  auto motor_node = rclcpp::node::Node::make_shared("pendulum_motor");

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.depth = 1;

  auto sensor_pub = motor_node->create_publisher<pendulum_msgs::msg::JointState>("pendulum_sensor",
      qos_profile);

  auto motor_subscribe_callback =
    [&pendulum_motor](const pendulum_msgs::msg::JointCommand::SharedPtr msg) -> void
    {
      pendulum_motor->on_command_message(msg);
    };

  auto command_sub = motor_node->create_subscription<pendulum_msgs::msg::JointCommand>
      ("pendulum_command", qos_profile, motor_subscribe_callback,
      nullptr, false, command_msg_strategy);

  auto controller_subscribe_callback =
    [&pendulum_controller](const pendulum_msgs::msg::JointState::SharedPtr msg) -> void
    {
      pendulum_controller->on_sensor_message(msg);
    };

  auto command_pub = controller_node->create_publisher<pendulum_msgs::msg::JointCommand>(
    "pendulum_command", qos_profile);
  auto sensor_sub = controller_node->create_subscription<pendulum_msgs::msg::JointState>
      ("pendulum_sensor", qos_profile, controller_subscribe_callback,
      nullptr, false, state_msg_strategy);

  auto executor = std::make_shared<pendulum_control::RttExecutor>(memory_strategy);
  executor->add_node(motor_node);
  executor->add_node(controller_node);

  auto motor_publish_callback =
    [&sensor_pub, &pendulum_motor]()
    {
      if (pendulum_motor->next_message_ready()) {
        auto msg = pendulum_motor->get_next_sensor_message();
        sensor_pub->publish(msg);
      }
    };

  auto controller_publish_callback =
    [&command_pub, &pendulum_controller]()
    {
      if (pendulum_controller->next_message_ready()) {
        auto msg = pendulum_controller->get_next_command_message();
        command_pub->publish(msg);
      }
    };

  // Add timers for publishers
  auto motor_publisher_timer = motor_node->create_wall_timer
      (pendulum_motor->get_publish_period(), motor_publish_callback);
  auto controller_publisher_timer = controller_node->create_wall_timer
      (pendulum_controller->get_publish_period(), controller_publish_callback);
  if (rttest_set_sched_priority(98, SCHED_RR)) {
    perror("Couldn't set scheduling priority and policy");
  }

  // Create output thread
  // Must be a pthread so that we can set the scheduler
  pthread_t output_thread;
  OutputThreadArgs ptr_args(executor, pendulum_motor, pendulum_controller);
  void * args = static_cast<void *>(&ptr_args);
  pthread_create(&output_thread, NULL, live_output_thread, static_cast<void *>(args));

  if (rttest_lock_and_prefault_dynamic() != 0) {
    perror("Couldn't lock memory");
  }

  // End initialization phase
  // Execution phase
  executor->spin();
  pendulum_motor->set_done(true);

  // End execution phase
  // Teardown phase
  void * status;
  pthread_join(output_thread, &status);
  printf("PendulumMotor received %lu messages\n", pendulum_motor->messages_received);
  printf("PendulumController received %lu messages\n", pendulum_controller->messages_received);
}
