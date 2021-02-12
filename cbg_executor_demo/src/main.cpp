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

#include <ctime>
#include <cstdlib>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#ifndef _WIN32  // i.e., POSIX platform.
#include <pthread.h>
#else  // i.e., Windows platform.
#include <windows.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include "cbg_executor_demo/ping_node.hpp"
#include "cbg_executor_demo/pong_node.hpp"

/// Sets the priority of the given thread to max or min priority (in the SCHED_FIFO real-time
/// policy) and pins the thread to the given cpu (if cpu_id >= 0).
bool configure_thread(std::thread & thread, bool set_high_prio, int cpu_id)
{
  bool success = true;
#ifndef _WIN32  // i.e., POSIX platform.
  sched_param params;
  int policy;
  success &= (pthread_getschedparam(thread.native_handle(), &policy, &params) == 0);
  if (set_high_prio) {
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
  } else {
    params.sched_priority = sched_get_priority_min(SCHED_FIFO);
  }

  success &= (pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &params) == 0);

  if (cpu_id >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    success &= (pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuset) == 0);
  }
#else  // i.e., Windows platform.
  auto thread_handle = thread.native_handle();
  success &= (SetThreadPriority(thread_handle, set_high_prio ? 1 : -1) != 0);
  if (cpu_id >= 0) {
    DWORD_PTR cpuset = 1;
    cpuset <<= cpu_id;
    success &= (SetThreadAffinityMask(thread_handle, cpuset) != 0);
  }
#endif
  return success;
}


/// Returns the time of the given clock as std::chrono timestamp. When used with a
/// thread's clock, this function may be used to measure the thread's CPU time.
std::chrono::nanoseconds get_current_thread_clock_time(std::thread & thread)
{
#ifndef _WIN32  // i.e., POSIX platform.
  clockid_t id;
  pthread_getcpuclockid(thread.native_handle(), &id);
  timespec spec;
  clock_gettime(id, &spec);
  return std::chrono::seconds{spec.tv_sec} + std::chrono::nanoseconds{spec.tv_nsec};
#else  // i.e., Windows platform.
  FILETIME creation_filetime;
  FILETIME exit_filetime;
  FILETIME kernel_filetime;
  FILETIME user_filetime;
  GetThreadTimes(
    thread.native_handle(), &creation_filetime, &exit_filetime, &kernel_filetime, &user_filetime);
  ULARGE_INTEGER kernel_time;
  kernel_time.LowPart = kernel_filetime.dwLowDateTime;
  kernel_time.HighPart = kernel_filetime.dwHighDateTime;
  ULARGE_INTEGER user_time;
  user_time.LowPart = user_filetime.dwLowDateTime;
  user_time.HighPart = user_filetime.dwHighDateTime;
  std::chrono::nanoseconds t(100);  // Unit in FILETIME is 100ns.
  t *= (kernel_time.QuadPart + user_time.QuadPart);
  return t;
#endif
}


/// The main function composes the Ping and Pong node (depending on the arguments)
/// and runs the experiment. See README.md for a simple architecture diagram.
/// Here: rt = real-time = high scheduler priority and be = best-effort = low scheduler priority.
int main(int argc, char * argv[])
{
  using std::chrono::seconds;
  using std::chrono::milliseconds;
  using std::chrono::nanoseconds;

  using namespace std::chrono_literals;

  using cbg_executor_demo::PingNode;
  using cbg_executor_demo::PongNode;

  const std::chrono::seconds EXPERIMENT_DURATION = 10s;

  rclcpp::init(argc, argv);

  // Create two executors within this process.
  rclcpp::executors::SingleThreadedExecutor high_prio_executor;
  rclcpp::executors::SingleThreadedExecutor low_prio_executor;

#ifdef ADD_PING_NODE
  auto ping_node = std::make_shared<PingNode>();
  high_prio_executor.add_node(ping_node);
  rclcpp::Logger logger = ping_node->get_logger();
#endif

#ifdef ADD_PONG_NODE
  auto pong_node = std::make_shared<PongNode>();
  high_prio_executor.add_callback_group(
    pong_node->get_high_prio_callback_group(), pong_node->get_node_base_interface());
  low_prio_executor.add_callback_group(
    pong_node->get_low_prio_callback_group(), pong_node->get_node_base_interface());
#ifndef ADD_PING_NODE
  rclcpp::Logger logger = pong_node->get_logger();
#endif
#endif

  std::thread high_prio_thread([&]() {
      high_prio_executor.spin();
    });
  bool areThreadPriosSet = configure_thread(high_prio_thread, true, 0);

  std::thread low_prio_thread([&]() {
      low_prio_executor.spin();
    });
  areThreadPriosSet &= configure_thread(low_prio_thread, false, 0);

  // Creating the threads immediately started them. Therefore, get start CPU time of each
  // thread now.
  nanoseconds high_prio_thread_begin = get_current_thread_clock_time(high_prio_thread);
  nanoseconds low_prio_thread_begin = get_current_thread_clock_time(low_prio_thread);

  if (!areThreadPriosSet) {
    RCLCPP_WARN(logger, "Thread priorities are not configured correctly!");
    RCLCPP_WARN(logger, "Are you root (sudo)? Experiment is performed anyway.");
  }

  RCLCPP_INFO(
    logger, "Running experiment from now on for %ld s ...", EXPERIMENT_DURATION.count());

  std::this_thread::sleep_for(EXPERIMENT_DURATION);

  // Get end CPU time of each thread ...
  nanoseconds high_prio_thread_end = get_current_thread_clock_time(high_prio_thread);
  nanoseconds low_prio_thread_end = get_current_thread_clock_time(low_prio_thread);

  // ... and stop the experiment.
  rclcpp::shutdown();
  high_prio_thread.join();
  low_prio_thread.join();

#ifdef ADD_PING_NODE
  ping_node->print_statistics();
#endif

  // Print CPU times.
  int64_t high_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    high_prio_thread_end - high_prio_thread_begin).count();
  int64_t low_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    low_prio_thread_end - low_prio_thread_begin).count();
  RCLCPP_INFO(
    logger, "High priority executor thread ran for %ld ms.", high_prio_thread_duration_ms);
  RCLCPP_INFO(
    logger, "Low priority executor thread ran for %ld ms.", low_prio_thread_duration_ms);
  if (!areThreadPriosSet) {
    RCLCPP_WARN(logger, "Again, thread priorities were not configured correctly!");
  }

  return 0;
}
