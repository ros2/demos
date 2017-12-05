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

#ifndef PENDULUM_CONTROL__RTT_EXECUTOR_HPP_
#define PENDULUM_CONTROL__RTT_EXECUTOR_HPP_

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "rttest/rttest.h"
#include "rttest/utils.h"

#include "rmw/rmw.h"

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"

namespace pendulum_control
{
/// Instrumented executor that syncs Executor::spin functions with rttest_spin.
class RttExecutor : public rclcpp::executor::Executor
{
public:
  /// Constructor
  /**
   * Extends default Executor constructor
   */
  RttExecutor(
    const rclcpp::executor::ExecutorArgs & args =
    rclcpp::executor::create_default_executor_arguments())
  : rclcpp::executor::Executor(args), running(false)
  {
    rttest_ready = rttest_running();
    memset(&start_time_, 0, sizeof(timespec));
  }

  /// Default destructor
  virtual ~RttExecutor() {}

  /// Return the status of rttest.
  // \return True if rttest has initialized, false if it is uninitialized or has finished.
  bool is_rttest_ready() const
  {
    return rttest_ready;
  }

  /// Return true if the executor is currently spinning.
  // \return True if rclcpp is running and if the "running" boolean is set to true.
  bool is_running() const
  {
    return rclcpp::ok() && running;
  }

  /// Retrieve the results measured by rttest
  // \param[in] output A struct containing performance statistics.
  void get_rtt_results(rttest_results & output) const
  {
    output = results;
  }

  void set_rtt_results_message(pendulum_msgs::msg::RttestResults::SharedPtr msg) const
  {
    msg->cur_latency = last_sample;
    msg->mean_latency = results.mean_latency;
    msg->min_latency = results.min_latency;
    msg->max_latency = results.max_latency;
    msg->minor_pagefaults = results.minor_pagefaults;
    msg->major_pagefaults = results.major_pagefaults;
    timespec curtime;
    clock_gettime(CLOCK_MONOTONIC, &curtime);
    msg->stamp.sec = curtime.tv_sec;
    msg->stamp.nanosec = curtime.tv_nsec;
  }

  /// Wrap executor::spin into rttest_spin.
  // Do all the work available to the executor for as many iterations specified by rttest.
  void spin()
  {
    // This call will block until rttest is finished, calling loop_callback at periodic intervals
    // specified on the command line.
    rttest_spin(RttExecutor::loop_callback, static_cast<void *>(this));

    // Clean up state and write results after rttest has finished spinning.
    running = false;
    rttest_write_results();
    if (rttest_running()) {
      rttest_finish();
    }
    rttest_ready = rttest_running();
  }

  /// Instrumented "spin_some"
  /**
   * This function can have unexpected results if it is called in succession with a non-monotonic
   * input value. It is up to the user to ensure "i" increases linearly.
   * \param[in] The iteration for this spin operation, and the index into rttest's data buffer.
   * \return Pass the error code from rttest (0 on success, non-zero error code on failure).
   */
  int rtt_spin_some(size_t i)
  {
    // Initialize the start time  if this is the first iteration.
    if (i == 0) {
      clock_gettime(0, &start_time_);
    }
    // Wrap Executor::spin_some into rttest.
    return rttest_spin_once(RttExecutor::loop_callback, static_cast<void *>(this), &start_time_, i);
  }

  /// Core component of the executor. Do a little bit of work and update extra state.
  // \param[in] Anonymous argument, will be casted as a pointer to an RttExecutor.
  static void * loop_callback(void * arg)
  {
    // Cast the argument so that we can access the executor's state.
    RttExecutor * executor = static_cast<RttExecutor *>(arg);
    // If the input pointer was NULL or invalid, or if rclcpp has stopped, signal rttest to stop.
    if (!executor || !rclcpp::ok()) {
      rttest_finish();
      return 0;
    }
    // Single-threaded spin_some: do as much work as we have available.
    executor->spin_some();

    // Retrieve rttest statistics accumulated so far and store them in the executor.
    rttest_get_statistics(&executor->results);
    rttest_get_sample_at(executor->results.iteration, &executor->last_sample);
    // In case this boolean wasn't set, notify that we've recently run the callback.
    executor->running = true;
    return 0;
  }

  // For storing accumulated performance statistics.
  rttest_results results;
  // True if the executor is spinning.
  bool running;
  // True if rttest has initialized and hasn't been stopped yet.
  bool rttest_ready;

  int last_sample;

protected:
  // Absolute timestamp at which the first data point was collected in rttest.
  timespec start_time_;

private:
  RCLCPP_DISABLE_COPY(RttExecutor)
};

}  // namespace pendulum_control

#endif  // PENDULUM_CONTROL__RTT_EXECUTOR_HPP_
