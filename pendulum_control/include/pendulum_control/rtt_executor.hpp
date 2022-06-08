#ifndef PENDULUM_CONTROL__RTT_EXECUTOR_HPP_
#define PENDULUM_CONTROL__RTT_EXECUTOR_HPP_

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "rttest/rttest.h"
#include "rttest/utils.hpp"

#include "rmw/rmw.h"

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"

namespace pendulum_control
{
/// Instrumented executor that syncs Executor::spin functions with rttest_spin.
class RttExecutor : public rclcpp::Executor
{
public:
  /// Constructor
  /**
   * Extends default Executor constructor
   */
  RttExecutor(
    rclcpp::Executor::SharedPtr decorableExecutor,
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions())
  : rclcpp::Executor(options)
  , running(false)
  , decorableExecutor(decorableExecutor)
  {
    rttest_ready = rttest_running();
    memset(&start_time_, 0, sizeof(timespec));
  }

  /// Default destructor
  virtual ~RttExecutor() {}

  /// Return true if the executor is currently spinning.
  // \return True if rclcpp is running and if the "running" boolean is set to true.
  bool is_running() const
  {
    return rclcpp::ok() && running;
  }

  bool set_rtt_results_message(pendulum_msgs::msg::RttestResults & msg) const
  {
    if (!results_available) {
      return false;
    }
    msg.cur_latency = last_sample;
    msg.mean_latency = results.mean_latency;
    assert(results.min_latency >= 0);
    assert(results.max_latency >= 0);
    msg.min_latency = results.min_latency;
    msg.max_latency = results.max_latency;
    msg.minor_pagefaults = results.minor_pagefaults;
    msg.major_pagefaults = results.major_pagefaults;
    timespec curtime;
    clock_gettime(CLOCK_MONOTONIC, &curtime);
    msg.stamp.sec = curtime.tv_sec;
    msg.stamp.nanosec = curtime.tv_nsec;

    return true;
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
    if (rttest_get_statistics(&executor->results) >= 0) {
      executor->results_available = true;
    }
    rttest_get_sample_at(executor->results.iteration, &executor->last_sample);
    // In case this boolean wasn't set, notify that we've recently run the callback.
    executor->running = true;
    return 0;
  }

  void spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override
  {
    decorableExecutor->spin_some(max_duration);
  }

  void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override
  {
    decorableExecutor->add_node(node_ptr, notify);
  }

  // For storing accumulated performance statistics.
  rttest_results results;
  bool results_available{false};
  // True if the executor is spinning.
  bool running;
  // True if rttest has initialized and hasn't been stopped yet.
  bool rttest_ready;

  int64_t last_sample;

protected:
  // Absolute timestamp at which the first data point was collected in rttest.
  timespec start_time_;

private:
  rclcpp::Executor::SharedPtr decorableExecutor;
  RCLCPP_DISABLE_COPY(RttExecutor)
};

}  // namespace pendulum_control

#endif  // PENDULUM_CONTROL__RTT_EXECUTOR_HPP_
