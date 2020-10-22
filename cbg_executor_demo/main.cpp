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

#include "PingSide.hpp"
#include "PongSide.hpp"


using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

using rclcpp::Executor;
using rclcpp::executors::SingleThreadedExecutor;

const std::chrono::seconds EXPERIMENT_DURATION = 10s;


/// This function prints details on the arguments of this executable.
void print_help()
{
  std::cout << "Call with arguments [type] [rt_ping_period_us] [be_ping_period_us] " <<
    "[rt_busyloop_us] [be_busyloop_us] [cpu_id]." << std::endl;
  std::cout << "  type: determines the nodes included in this process:" << std::endl;
  std::cout << "    i: ping node only" << std::endl;
  std::cout << "    o: pong node only" << std::endl;
  std::cout << "    io: ping node and pong node" << std::endl;
  std::cout << "  rt_ping_period_us: microseconds between publishing of ping messages " << "by real-time thread in ping node" << std::endl;
  std::cout << "  be_ping_period_us: microseconds between publishing of ping messages " << "by best-effort thread in ping node" << std::endl;
  std::cout << "  rt_busyloop_us: microseconds of computation by real-time thread in " << "pong node before answering with pong" << std::endl;
  std::cout << "  be_busyloop_us: microseconds of computation by best-effort thread in " << "pong node before answering with pong" << std::endl;
  std::cout << "  cpu_id (optional): pins both, real-time thread and best-effort thread, to " << "the given cpu" << std::endl;
}


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
  using std::chrono::milliseconds;
  using std::chrono::nanoseconds;

  rclcpp::init(argc, argv);

  if (argc < 6 || argc > 7) {
    print_help();
    return 0;
  }

  std::string processType(argv[1]);

  microseconds rt_ping_period_us(atol(argv[2]));
  microseconds be_ping_period_us(atol(argv[3]));
  microseconds rt_busyloop_us(atol(argv[4]));
  microseconds be_busyloop_us(atol(argv[5]));

  int cpuId = 0;
  if (argc == 7) {
    cpuId = atoi(argv[6]);
  }

  // Create two executors within this process.
  rclcpp::executors::SingleThreadedExecutor executor_rt;
  rclcpp::executors::SingleThreadedExecutor executor_be;

  // If requested by the process-type argument, compose the Ping node from two PingSide
  // instances.
  rclcpp::Node::SharedPtr ping_node = nullptr;
  PingSide::SharedPtr ping_rt = nullptr;
  PingSide::SharedPtr ping_be = nullptr;
  if (processType.find("i") != std::string::npos) {
    std::cout << "Creating ping node." << std::endl;
    ping_node = std::make_shared<rclcpp::Node>("ping");
    ping_rt = std::make_shared<PingSide>(ping_node, "RT", rt_ping_period_us);
    ping_be = std::make_shared<PingSide>(ping_node, "BE", be_ping_period_us);
    executor_rt.add_callback_group(ping_rt->get_callback_group(), ping_node->get_node_base_interface());
    executor_be.add_callback_group(ping_be->get_callback_group(), ping_node->get_node_base_interface());
  }

  // If requested by the process-type argument, compose the Pong node from two PongSide
  // instances.
  rclcpp::Node::SharedPtr pong_node = nullptr;
  PongSide::SharedPtr pong_rt = nullptr;
  PongSide::SharedPtr pong_be = nullptr;
  if (processType.find("o") != std::string::npos) {
    std::cout << "Creating pong node." << std::endl;
    pong_node = std::make_shared<rclcpp::Node>("pong");
    pong_rt = std::make_shared<PongSide>(pong_node, "RT", rt_busyloop_us);
    pong_be = std::make_shared<PongSide>(pong_node, "BE", be_busyloop_us);
    executor_rt.add_callback_group(pong_rt->get_callback_group(), pong_node->get_node_base_interface());
    executor_be.add_callback_group(pong_be->get_callback_group(), pong_node->get_node_base_interface());
  }

  // Create and configure thread for the real-time executor, i.e. the high-priority executor.
  std::thread thread_rt([&]() {
      std::cout << "Thread with id=" << std::this_thread::get_id() << " is going to call executor_rt.spin() ..." << std::endl;
      executor_rt.spin();
    });
  configure_thread(thread_rt, true, cpuId);

  // Create and configure thread for the best-effort executor, i.e. the low-priority executor.
  std::thread thread_be([&]() {
      std::cout << "Thread with id=" << std::this_thread::get_id() << " is going to call executor_be.spin() ..." << std::endl;
      executor_be.spin();
    });
  configure_thread(thread_be, false, cpuId);

  // Creating the threads immediately started them. Therefore, get start CPU time of each
  /// thread now.
  clockid_t thread_rt_clock_id, thread_be_clock_id;
  pthread_getcpuclockid(thread_rt.native_handle(), &thread_rt_clock_id);
  pthread_getcpuclockid(thread_be.native_handle(), &thread_be_clock_id);
  nanoseconds rt_thread_begin = get_current_thread_clock_time(thread_rt_clock_id);
  nanoseconds be_thread_begin = get_current_thread_clock_time(thread_be_clock_id);

  std::this_thread::sleep_for(EXPERIMENT_DURATION);

  // Get end CPU time of each thread ...
  nanoseconds rt_thread_end = get_current_thread_clock_time(thread_rt_clock_id);
  nanoseconds be_thread_end = get_current_thread_clock_time(thread_be_clock_id);

  // ... and stop the experiment.
  rclcpp::shutdown();
  thread_rt.join();
  thread_be.join();

  // Print out throughput and latency statistics measured in the PingSide instances.
  if (ping_node) {
    ping_rt->print_statistics();
    ping_be->print_statistics();
  }

  // Print CPU times.
  long be_thread_duration_ms = std::chrono::duration_cast<milliseconds>(be_thread_end - be_thread_begin).count();
  long rt_thread_duration_ms = std::chrono::duration_cast<milliseconds>(rt_thread_end - rt_thread_begin).count();
  std::cout << "RT thread ran for " << rt_thread_duration_ms << "ms" << std::endl;
  std::cout << "BE thread ran for " << be_thread_duration_ms << "ms" << std::endl;

  std::cout << "TxPeriod: " << rt_ping_period_us.count() << "us " << be_ping_period_us.count() << "us";
  std::cout << " CTime: " << rt_busyloop_us.count() << "us " << be_busyloop_us.count() << "us";
  std::cout << " ThdRunTime: " << rt_thread_duration_ms << "ms " << be_thread_duration_ms << "ms" << std::endl;

  return 0;
}
