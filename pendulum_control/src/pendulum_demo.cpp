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

#include <execinfo.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <rclcpp/memory_strategies.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>

#include <rttest/rttest.h>

#include <pendulum_msgs/msg/joint_command.hpp>
#include <pendulum_msgs/msg/joint_state.hpp>

#include "pendulum_control/pendulum_controller.hpp"
#include "pendulum_control/pendulum_motor.hpp"
#include "pendulum_control/rtt_executor.hpp"

// Initialize a malloc hook so we can show that no mallocs are

/// Declare a function pointer into which we will store the default malloc.
static void *(*prev_malloc_hook)(size_t, const void *);

// Use pragma to ignore a warning for using __malloc_hook, which is deprecated (but still awesome).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Implement a custom malloc.
/**
 * Our custom malloc backtraces to find the address of the function that called malloc and formats
 * the line as a string (if the code was compiled with debug symbols.
 * \param[in] size Requested malloc size.
 * \param[in] caller pointer to the caller of this function (unused).
 * \return Pointer to the allocated memory
 */
static void * testing_malloc(size_t size, const void * caller)
{
  (void)caller;
  // Maximum depth we are willing to traverse into the stack trace.
  static const int MAX_DEPTH = 2;
  // Instantiate a buffer to store the traced symbols.
  void * backtrace_buffer[MAX_DEPTH];
  // Set the malloc implementation to the default malloc hook so that we can call it implicitly
  // to initialize a string, otherwise this function will loop infinitely.
  __malloc_hook = prev_malloc_hook;
  // Backtrace and get the depth of the stack that we traversed.
  int stack_depth = backtrace(backtrace_buffer, MAX_DEPTH);
  // Format the symbols as human-readable strings if possible.
  char ** function_names = backtrace_symbols(backtrace_buffer, stack_depth);
  // The 0th level of the stack is not very useful; try printing the 1st level.
  if (stack_depth > 1) {
    printf("malloc(%u) called from %s [%p]\n",
      (unsigned)size, function_names[1], backtrace_buffer[1]);
  } else {
    // If the first level was unavailable, default to the 0th level.
    printf("malloc(%u) called from %s [%p]\n",
      (unsigned)size, function_names[0], backtrace_buffer[0]);
  }
  // Release the string that was allocated for printing.
  free(function_names);
  // Execute the requested malloc.
  void * mem = malloc(size);
  // Set the malloc hook back to this function, so that we can intercept future mallocs.
  __malloc_hook = testing_malloc;
  return mem;
}

/// Function to be called when the malloc hook is initialized.
void init_malloc_hook()
{
  // Store the default malloc.
  prev_malloc_hook = __malloc_hook;
  // Set our custom malloc to the malloc hook.
  __malloc_hook = testing_malloc;
}
#pragma GCC diagnostic pop

/// Set the hook for malloc initialize so that init_malloc_hook gets called.
void(*volatile __malloc_initialize_hook)(void) = init_malloc_hook;

// Structure for passing arguments to a pthread.
// We need to use pthreads in this example, since the standard C++ thread API does not provide
// options for setting thread priorities.
struct OutputThreadArgs
{
  // The instrumented executor passed to the thread (see rtt_executor.hpp)
  std::shared_ptr<pendulum_control::RttExecutor> executor;
  // The pendulum motor model (see pendulum_motor.hpp)
  std::shared_ptr<pendulum_control::PendulumMotor> pendulum_motor;
  // The pendulum controller model (see pendulum_controller.hpp)
  std::shared_ptr<pendulum_control::PendulumController> pendulum_controller;

  /// Default constructor
  /**
   * \param[in] exec The real-time executor (needed to know when to start and stop printing).
   * \param[in] motor The motor controlling the pendulum (needed for its position).
   * \param[in] controller The pendulum controller (needed for its commanded position).
   */
  OutputThreadArgs(std::shared_ptr<pendulum_control::RttExecutor> exec,
    std::shared_ptr<pendulum_control::PendulumMotor> motor,
    std::shared_ptr<pendulum_control::PendulumController> controller)
  : executor(exec), pendulum_motor(motor), pendulum_controller(controller) {}
};

// The output thread for printing statistics collected during runtime.
// This thread will run at a low priority, since console output is not real-time safe
void * live_output_thread(void * args)
{
  OutputThreadArgs * ptr_args = static_cast<OutputThreadArgs *>(args);
  auto executor = ptr_args->executor;
  auto pendulum_motor = ptr_args->pendulum_motor;
  auto pendulum_controller = ptr_args->pendulum_controller;

  // Notify the scheduler that this thread is a low-priority "idle" task.
  if (rttest_set_sched_priority(0, SCHED_IDLE) != 0) {
    perror("Couldn't set priority of output thread to IDLE");
  }

  struct rttest_results stats;

  // Exit this thread when rttest has finished running.
  while (executor->is_rttest_ready()) {
    // Only print output if the executor is running (spinning).
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

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::HeapPoolMemoryStrategy;

int main(int argc, char * argv[])
{
  // Initialization phase.
  // In the initialization phase of a realtime program, non-realtime-safe operations such as
  // allocation memory are permitted.

  // Create a structure with the default physical propreties of the pendulum (length and mass).
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

  // The ObjectPoolBounds struct specifies the maximum allowable numbers for subscriptions,
  // services, clients, executables, and a shared memory pool.
  rclcpp::memory_strategies::heap_pool_memory_strategy::ObjectPoolBounds bounds;
  // max_subscriptions needs to be twice the number of expected subscriptions, since a different
  // handle is allocated for intra-process subscriptions
  bounds.set_max_subscriptions(4).set_max_services(0).set_max_clients(0);
  bounds.set_max_executables(1).set_memory_pool_size(0);

  // These bounds are passed to the HeapPoolMemoryStrategy, which preallocates pools for each object
  // type. If the HeapPoolMemoryStrategy is used, the executor will re-use these static object pools
  // instead of malloc'ing and freeing these objects during execution (after spin is called).
  auto memory_strategy = std::make_shared<HeapPoolMemoryStrategy>(bounds);

  // The MessagePoolMemoryStrategy preallocates a pool of messages to be used by the subscription.
  // Typically, one MessagePoolMemoryStrategy is used per subscription type, and the size of the
  // message pool is determined by the number of threads (the maximum number of concurrent accesses
  // to the subscription).
  // Since this example is single-threaded, we choose a message pool size of 1 for each strategy.
  auto state_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointState, 1>>();
  auto command_msg_strategy =
    std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointCommand, 1>>();

  // The controller node represents user code. This example implements a simple PID controller.
  auto controller_node = rclcpp::node::Node::make_shared("pendulum_controller");

  // The "motor" node simulates motors and sensors.
  // It provides sensor data and changes the physical model based on the command.
  auto motor_node = rclcpp::node::Node::make_shared("pendulum_motor");

  // The quality of service profile is tuned for real-time performance.
  // More QoS settings may be exposed by the rmw interface in the future to fulfill real-time
  // requirements.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
  // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
  qos_profile.reliability = RMW_QOS_POLICY_BEST_EFFORT;
  // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
  // are sent, to aid with recovery in the event of dropped messages.
  // "depth" specifies the size of this buffer.
  // In this example, we are optimizing for performance and limited resource usage (preventing page
  // faults), instead of reliability. Thus, we set the size of the history buffer to 1.
  qos_profile.history = RMW_QOS_POLICY_KEEP_LAST_HISTORY;
  qos_profile.depth = 1;

  // Initialize the publisher for the sensor message (the current position of the pendulum).
  auto sensor_pub = motor_node->create_publisher<pendulum_msgs::msg::JointState>("pendulum_sensor",
      qos_profile);

  // Create a lambda function to invoke the motor callback when a command is received.
  auto motor_subscribe_callback =
    [&pendulum_motor](const pendulum_msgs::msg::JointCommand::SharedPtr msg) -> void
    {
      pendulum_motor->on_command_message(msg);
    };

  // Initialize the subscription to the command message.
  // Notice that we pass the MessagePoolMemoryStrategy<JointCommand> initialized above.
  auto command_sub = motor_node->create_subscription<pendulum_msgs::msg::JointCommand>
      ("pendulum_command", motor_subscribe_callback, qos_profile,
      nullptr, false, command_msg_strategy);

  // Create a lambda function to invoke the controller callback when a command is received.
  auto controller_subscribe_callback =
    [&pendulum_controller](const pendulum_msgs::msg::JointState::SharedPtr msg) -> void
    {
      pendulum_controller->on_sensor_message(msg);
    };

  // Initialize the publisher for the command message.
  auto command_pub = controller_node->create_publisher<pendulum_msgs::msg::JointCommand>(
    "pendulum_command", qos_profile);

  // Initialize the subscriber for the sensor message.
  // Notice that we pass the MessageMemoryPoolStrategy<JointState> initialized above.
  auto sensor_sub = controller_node->create_subscription<pendulum_msgs::msg::JointState>
      ("pendulum_sensor", controller_subscribe_callback, qos_profile,
      nullptr, false, state_msg_strategy);

  // Initialize the executor.
  // RttExecutor is a special single-threaded executor instrumented to calculate and record
  // real-time performance statistics.
  auto executor = std::make_shared<pendulum_control::RttExecutor>(memory_strategy);

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

  // Add a timer to enable regular publication of sensor messages.
  auto motor_publisher_timer = motor_node->create_wall_timer
      (pendulum_motor->get_publish_period(), motor_publish_callback);
  // Add a timer to enable regular publication of command messages.
  auto controller_publisher_timer = controller_node->create_wall_timer
      (pendulum_controller->get_publish_period(), controller_publish_callback);

  // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
  // deterministic (real-time safe) algorithm, round robin.
  if (rttest_set_sched_priority(98, SCHED_RR)) {
    perror("Couldn't set scheduling priority and policy");
  }

  // Create a low-priority output pthread for printing the results to the console.
  pthread_t output_thread;
  OutputThreadArgs ptr_args(executor, pendulum_motor, pendulum_controller);
  void * args = static_cast<void *>(&ptr_args);
  pthread_create(&output_thread, NULL, live_output_thread, static_cast<void *>(args));

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

  // Execution phase

  // Unlike the default SingleThreadedExecutor::spin function, RttExecutor::spin runs in
  // bounded time (for as many iterations as specified in the rttest parameters).
  executor->spin();
  // Once the executor has exited, notify the physics simulation to stop running.
  pendulum_motor->set_done(true);

  // End execution phase

  // Teardown phase

  // Join the output thread.
  pthread_join(output_thread, NULL);
  printf("PendulumMotor received %lu messages\n", pendulum_motor->messages_received);
  printf("PendulumController received %lu messages\n", pendulum_controller->messages_received);
}
