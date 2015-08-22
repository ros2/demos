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

#ifndef PENDULUM_DEMO_RTT_EXECUTOR_HPP_
#define PENDULUM_DEMO_RTT_EXECUTOR_HPP_

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>

#include <rttest/rttest.h>
#include <rttest/utils.h>

#include <rmw/rmw.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/memory_strategies.hpp>

namespace pendulum_control
{
class RttExecutor : public executor::Executor
{
public:
  RttExecutor(memory_strategy::MemoryStrategy::SharedPtr ms =
    memory_strategy::create_default_strategy())
  : executor::Executor(ms), running(false)
  {
    rttest_ready = rttest_running();
  }

  virtual ~RttExecutor() {}

  bool is_rttest_ready() const
  {
    return rttest_ready;
  }

  bool is_running() const
  {
    return rclcpp::ok() && running;
  }

  void get_rtt_results(struct rttest_results & output) const
  {
    output = results;
  }

  void spin()
  {
    rttest_spin(RttExecutor::loop_callback, static_cast<void *>(this));
    running = false;
    rttest_write_results();
    rttest_finish();
    rttest_ready = rttest_running();
  }

  int rtt_spin_some(size_t i)
  {
    running = true;
    if (i == 0) {
      clock_gettime(0, &start_time_);
    }
    return rttest_spin_once(RttExecutor::loop_callback, static_cast<void *>(this), &start_time_, i);
  }

  static void * loop_callback(void * arg)
  {
    RttExecutor * executor = static_cast<RttExecutor *>(arg);
    if (!executor || !rclcpp::utilities::ok()) {
      return 0;
    }
    // Single-threaded spin_some: do as much work as we have available
    executor->spin_some();

    rttest_get_statistics(executor->results);
    executor->running = true;
    return 0;
  }

  struct rttest_results results;
  bool running;
  bool rttest_ready;

protected:
  struct timespec start_time_;

private:
  RCLCPP_DISABLE_COPY(RttExecutor);

};

} /* namespace pendulum_demo */

#endif /* PENDULUM_DEMO_RTT_EXECUTOR_HPP_ */
