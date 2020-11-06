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

#include <functional>
#include <iostream>
#include <thread>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <string>
#include <vector>

#include <pthread.h>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include "cbg_executor_demo/PingNode.hpp"
#include "cbg_executor_demo/PongNode.hpp"


/// Sets the priority of the given thread to max or min priority (in the SCHED_FIFO real-time
/// policy) and pins the thread to the given cpu (if cpu_id != 0).
void configure_thread(std::thread & t, bool set_real_time, int cpu_id)
{
  sched_param params;
  int policy;
  pthread_getschedparam(t.native_handle(), &policy, &params);
  if (set_real_time) {
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
  } else {
    params.sched_priority = sched_get_priority_min(SCHED_FIFO);
  }

  if (pthread_setschedparam(t.native_handle(), SCHED_FIFO, &params)) {
    throw std::runtime_error("Failed to set scheduler parameters of thread!");
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_id, &cpuset);
  pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &cpuset);
}


/// Returns the time of the given clock as std::chrono timestamp. When used with a
/// thread's clock, this function may be used to measure the thread's CPU time.
std::chrono::nanoseconds get_current_thread_clock_time(clockid_t id)
{
  timespec spec;
  clock_gettime(id, &spec);
  return std::chrono::seconds{spec.tv_sec} + std::chrono::nanoseconds{spec.tv_nsec};
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

  using namespace cbg_executor_demo;

  const std::chrono::seconds EXPERIMENT_DURATION = 10s;

  rclcpp::init(argc, argv);

  // Create two executors within this process.
  rclcpp::executors::SingleThreadedExecutor high_prio_executor;
  rclcpp::executors::SingleThreadedExecutor low_prio_executor;

#ifdef ADD_PING_NODE
  PingNode ping_node;
  high_prio_executor.add_callback_group(
    ping_node.get_high_prio_callback_group(), ping_node.get_node_base_interface());
  low_prio_executor.add_callback_group(
    ping_node.get_low_prio_callback_group(), ping_node.get_node_base_interface());
#endif

#ifdef ADD_PONG_NODE
  PongNode pong_node;
  high_prio_executor.add_callback_group(
    pong_node.get_high_prio_callback_group(), pong_node.get_node_base_interface());
  low_prio_executor.add_callback_group(
    pong_node.get_low_prio_callback_group(), pong_node.get_node_base_interface());
#endif

  // Create and configure thread for the real-time executor, i.e. the high-priority executor.
  std::thread high_prio_thread([&]() {
      std::cout << "Thread with id=" << std::this_thread::get_id() << " is going to call high_prio_executor.spin() ..." << std::endl;
      high_prio_executor.spin();
    });
  configure_thread(high_prio_thread, true, 1);

  // Create and configure thread for the best-effort executor, i.e. the low-priority executor.
  std::thread low_prio_thread([&]() {
      std::cout << "Thread with id=" << std::this_thread::get_id() << " is going to call low_prio_executor.spin() ..." << std::endl;
      low_prio_executor.spin();
    });
  configure_thread(low_prio_thread, false, 1);

  // Creating the threads immediately started them. Therefore, get start CPU time of each
  /// thread now.
  clockid_t high_prio_thread_clock_id, low_prio_thread_clock_io;
  pthread_getcpuclockid(high_prio_thread.native_handle(), &high_prio_thread_clock_id);
  pthread_getcpuclockid(low_prio_thread.native_handle(), &low_prio_thread_clock_io);
  nanoseconds high_prio_thread_begin = get_current_thread_clock_time(high_prio_thread_clock_id);
  nanoseconds low_prio_thread_begin = get_current_thread_clock_time(low_prio_thread_clock_io);

  std::this_thread::sleep_for(EXPERIMENT_DURATION);

  // Get end CPU time of each thread ...
  nanoseconds high_prio_thread_end = get_current_thread_clock_time(high_prio_thread_clock_id);
  nanoseconds low_prio_thread_end = get_current_thread_clock_time(low_prio_thread_clock_io);

  // ... and stop the experiment.
  rclcpp::shutdown();
  high_prio_thread.join();
  low_prio_thread.join();

  // // Print out throughput and latency statistics measured in the PingNode instances.
  // if (ping_node) {
  //   ping_rt->print_statistics();
  //   ping_be->print_statistics();
  // }

  // Print CPU times.
  long high_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    high_prio_thread_end - high_prio_thread_begin).count();
  long low_prio_thread_duration_ms = std::chrono::duration_cast<milliseconds>(
    low_prio_thread_end - low_prio_thread_begin).count();
  std::cout << "High prio thread ran for " << high_prio_thread_duration_ms << "ms" << std::endl;
  std::cout << "Low prio thread ran for " << low_prio_thread_duration_ms << "ms" << std::endl;

  // std::cout << "TxPeriod: " << rt_ping_period_us.count() << "us " << be_ping_period_us.count() << "us";
  // std::cout << " CTime: " << rt_busyloop_us.count() << "us " << be_busyloop_us.count() << "us";
  // std::cout << " ThdRunTime: " << rt_thread_duration_ms << "ms " << be_thread_duration_ms << "ms" << std::endl;

  return 0;
}
