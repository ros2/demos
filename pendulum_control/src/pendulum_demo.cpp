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

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>

#include "rttest/rttest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "tlsf_cpp/tlsf.hpp"

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"
#include "pendulum_msgs/msg/rttest_results.hpp"

#include "pendulum_control/pendulum_controller.hpp"
#include "pendulum_control/pendulum_motor.hpp"
#include "pendulum_control/rtt_executor.hpp"

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

int main(int argc, char * argv[])
{
  // Initialization phase.
  // In the initialization phase of a realtime program, non-realtime-safe operations such as
  // allocation memory are permitted.

  // Create a structure with the default physical properties of the pendulum (length and mass).
  pendulum_control::PendulumProperties properties;
  // Instantiate a PendulumMotor class which simulates the physics of the inverted pendulum
  // and provide a sensor message for the current position.
  // Run the callback for the motor slightly faster than the executor update loop.
  auto pendulum_motor = std::make_shared<pendulum_control::PendulumMotor>(
    std::chrono::nanoseconds(970000), properties);

  // Create the properties of the PID controller.
  pendulum_control::PIDProperties pid;
  // Instantiate a PendulumController class which will calculate the next motor command.
  // Run the callback for the controller slightly faster than the executor update loop.
  auto pendulum_controller = std::make_shared<pendulum_control::PendulumController>(
    std::chrono::nanoseconds(960000), pid);

  // Pass the input arguments to rttest.
  // rttest will store relevant parameters and allocate buffers for data collection
  rttest_read_args(argc, argv);

  // Pass the input arguments to rclcpp and initialize the signal handler.
  rclcpp::init(argc, argv);

  // The MessagePoolMemoryStrategy preallocates a pool of messages to be used by the subscription.
  // Typically, one MessagePoolMemoryStrategy is used per subscription type, and the size of the
  // message pool is determined by the number of threads (the maximum number of concurrent accesses
  // to the subscription).
  // Since this example is single-threaded, we choose a message pool size of 1 for each strategy.
  auto state_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointState, 1>>();
  auto command_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointCommand, 1>>();
  auto setpoint_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointCommand, 1>>();

  // The controller node represents user code. This example implements a simple PID controller.
  auto controller_node = rclcpp::Node::make_shared("pendulum_controller");

  // The "motor" node simulates motors and sensors.
  // It provides sensor data and changes the physical model based on the command.
  auto motor_node = rclcpp::Node::make_shared("pendulum_motor");

  // The quality of service profile is tuned for real-time performance.
  // More QoS settings may be exposed by the rmw interface in the future to fulfill real-time
  // requirements.
  auto qos = rclcpp::QoS(
    // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
    // are sent, to aid with recovery in the event of dropped messages.
    // "depth" specifies the size of this buffer.
    // In this example, we are optimizing for performance and limited resource usage (preventing
    // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
    rclcpp::KeepLast(1)
  );
  // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
  // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
  qos.best_effort();

  // Initialize the publisher for the sensor message (the current position of the pendulum).
  auto sensor_pub =
    motor_node->create_publisher<pendulum_msgs::msg::JointState>("pendulum_sensor", qos);

  // Create a lambda function to invoke the motor callback when a command is received.
  auto motor_subscribe_callback =
    [&pendulum_motor](pendulum_msgs::msg::JointCommand::ConstSharedPtr msg) -> void
    {
      pendulum_motor->on_command_message(msg);
    };

  // Initialize the subscription to the command message.
  // Notice that we pass the MessagePoolMemoryStrategy<JointCommand> initialized above.
  auto command_sub = motor_node->create_subscription<pendulum_msgs::msg::JointCommand>(
    "pendulum_command", qos, motor_subscribe_callback,
    rclcpp::SubscriptionOptions(), command_msg_strategy);

  // Create a lambda function to invoke the controller callback when a command is received.
  auto controller_subscribe_callback =
    [&pendulum_controller](pendulum_msgs::msg::JointState::ConstSharedPtr msg) -> void
    {
      pendulum_controller->on_sensor_message(msg);
    };

  // Initialize the publisher for the command message.
  auto command_pub = controller_node->create_publisher<pendulum_msgs::msg::JointCommand>(
    "pendulum_command", qos);

  // Initialize the subscriber for the sensor message.
  // Notice that we pass the MessageMemoryPoolStrategy<JointState> initialized above.
  auto sensor_sub = controller_node->create_subscription<pendulum_msgs::msg::JointState>(
    "pendulum_sensor", qos, controller_subscribe_callback,
    rclcpp::SubscriptionOptions(), state_msg_strategy);

  // Create a lambda function to accept user input to command the pendulum
  auto controller_command_callback =
    [&pendulum_controller](pendulum_msgs::msg::JointCommand::ConstSharedPtr msg) -> void
    {
      pendulum_controller->on_pendulum_setpoint(msg);
    };

  // Receive the most recently published message from the teleop node publisher.
  auto qos_setpoint_sub = qos;
  qos_setpoint_sub.transient_local();

  auto setpoint_sub = controller_node->create_subscription<pendulum_msgs::msg::JointCommand>(
    "pendulum_setpoint", qos_setpoint_sub, controller_command_callback,
    rclcpp::SubscriptionOptions(), setpoint_msg_strategy);

  // Initialize the logger publisher.
  auto logger_pub = controller_node->create_publisher<pendulum_msgs::msg::RttestResults>(
    "pendulum_statistics", qos);
  std::chrono::nanoseconds logger_publisher_period(1000000);

  // Initialize the executor.
  rclcpp::ExecutorOptions options;
  // One of the arguments passed to the Executor is the memory strategy, which delegates the
  // runtime-execution allocations to the TLSF allocator.
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
  options.memory_strategy = memory_strategy;
  // RttExecutor is a special single-threaded executor instrumented to calculate and record
  // real-time performance statistics.
  auto executor = std::make_shared<pendulum_control::RttExecutor>(options);

  // Add the motor and controller nodes to the executor.
  executor->add_node(motor_node);
  executor->add_node(controller_node);

  // Create a lambda function that will fire regularly to publish the next sensor message.
  auto motor_publish_callback =
    [&sensor_pub, &pendulum_motor]()
    {
      if (pendulum_motor->next_message_ready()) {
        auto msg = pendulum_motor->get_next_sensor_message();
        sensor_pub->publish(msg);
      }
    };

  // Create a lambda function that will fire regularly to publish the next command message.
  auto controller_publish_callback =
    [&command_pub, &pendulum_controller]()
    {
      if (pendulum_controller->next_message_ready()) {
        auto msg = pendulum_controller->get_next_command_message();
        command_pub->publish(msg);
      }
    };

  // Create a lambda function that will fire regularly to publish the next results message.
  auto logger_publish_callback =
    [&logger_pub, &executor, &pendulum_motor, &pendulum_controller]() {
      pendulum_msgs::msg::RttestResults results_msg;
      if (!executor->set_rtt_results_message(results_msg)) {
        // No data is available, just get out instead of publishing bogus data.
        return;
      }
      results_msg.command = pendulum_controller->get_next_command_message();
      results_msg.state = pendulum_motor->get_next_sensor_message();
      logger_pub->publish(results_msg);
    };

  // Add a timer to enable regular publication of sensor messages.
  auto motor_publisher_timer = motor_node->create_wall_timer(
    pendulum_motor->get_publish_period(), motor_publish_callback);
  // Add a timer to enable regular publication of command messages.
  auto controller_publisher_timer = controller_node->create_wall_timer(
    pendulum_controller->get_publish_period(), controller_publish_callback);
  // Add a timer to enable regular publication of results messages.
  auto logger_publisher_timer = controller_node->create_wall_timer(
    logger_publisher_period, logger_publish_callback);

  // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
  // deterministic (real-time safe) algorithm, round robin.
  if (rttest_set_sched_priority(98, SCHED_RR)) {
    perror("Couldn't set scheduling priority and policy");
  }

  // Lock the currently cached virtual memory into RAM, as well as any future memory allocations,
  // and do our best to prefault the locked memory to prevent future pagefaults.
  // Will return with a non-zero error code if something went wrong (insufficient resources or
  // permissions).
  // Always do this as the last step of the initialization phase.
  // See README.md for instructions on setting permissions.
  // See rttest/rttest.cpp for more details.
  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory.\n");
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM will be recorded.\n");
  }

  // End initialization phase

  // Unlike the default SingleThreadedExecutor::spin function, RttExecutor::spin runs in
  // bounded time (for as many iterations as specified in the rttest parameters).
  executor->spin();
  // Once the executor has exited, notify the physics simulation to stop running.
  pendulum_motor->set_done(true);

  // End execution phase

  printf("PendulumMotor received %zu messages\n", pendulum_motor->messages_received);
  printf("PendulumController received %zu messages\n", pendulum_controller->messages_received);

  rclcpp::shutdown();

  return 0;
}
