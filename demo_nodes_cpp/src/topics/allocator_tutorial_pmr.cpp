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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <list>
#include <memory>
#include <memory_resource>
#include <new>
#include <string>
#include <utility>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"

#include "example_interfaces/msg/u_int32.hpp"
#include "rcl/allocator.h"

using namespace std::chrono_literals;

// For demonstration purposes only, not necessary for allocator_traits
static uint32_t num_allocs = 0;
static uint32_t num_deallocs = 0;
// A very simple custom memory resource. Counts calls to do_allocate and do_deallocate.
class CustomMemoryResource : public std::pmr::memory_resource
{
private:
  void * do_allocate(std::size_t bytes, std::size_t alignment) override
  {
    num_allocs++;
    (void)alignment;
    return std::malloc(bytes);
  }

  void do_deallocate(
    void * p, std::size_t bytes,
    std::size_t alignment) override
  {
    num_deallocs++;
    (void)bytes;
    (void)alignment;
    std::free(p);
  }

  bool do_is_equal(
    const std::pmr::memory_resource & other) const noexcept override
  {
    return this == &other;
  }
};


// Override global new and delete to count calls during execution.

static bool is_running = false;
static uint32_t global_runtime_allocs = 0;
static uint32_t global_runtime_deallocs = 0;

// Due to GCC bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=103993, we
// always inline the overridden new and delete operators.

#if defined(__GNUC__) || defined(__clang__)
#define NOINLINE __attribute__((noinline))
#else
#define NOINLINE
#endif

NOINLINE void * operator new(std::size_t size)
{
  if (size == 0) {
    ++size;
  }

  if (is_running) {
    global_runtime_allocs++;
  }

  void * ptr = std::malloc(size);
  if (ptr != nullptr) {
    return ptr;
  }

  throw std::bad_alloc{};
}

NOINLINE void operator delete(void * ptr, size_t size) noexcept
{
  (void)size;
  if (ptr != nullptr) {
    if (is_running) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
  }
}

NOINLINE void operator delete(void * ptr) noexcept
{
  if (ptr != nullptr) {
    if (is_running) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
  }
}

// rclcpp needs a C-style rcl allocator, but rcl's deallocate does not pass the block size that
// std::pmr::memory_resource::deallocate requires. So get_rcl_allocator() stores each block's
// size in a header in front of it.
template<typename T = void>
struct PmrAllocator : std::pmr::polymorphic_allocator<T>
{
  using std::pmr::polymorphic_allocator<T>::polymorphic_allocator;

  template<typename U>
  PmrAllocator(const PmrAllocator<U> & other) noexcept  // NOLINT(runtime/explicit)
  : std::pmr::polymorphic_allocator<T>(other.resource()) {}

  template<typename U>
  struct rebind
  {
    using other = PmrAllocator<U>;
  };

  rcl_allocator_t get_rcl_allocator() const
  {
    rcl_allocator_t rcl_allocator = rcl_get_default_allocator();
    rcl_allocator.allocate = &allocate_block;
    rcl_allocator.deallocate = &deallocate_block;
    rcl_allocator.reallocate = &reallocate_block;
    rcl_allocator.zero_allocate = &zero_allocate_block;
    rcl_allocator.state = this->resource();
    return rcl_allocator;
  }

private:
  static constexpr size_t header = alignof(std::max_align_t);

  // Called from C, so like malloc they return nullptr instead of throwing.
  static void * allocate_block(size_t size, void * state) noexcept
  {
    if (size > (std::numeric_limits<size_t>::max)() - header) {
      return nullptr;
    }
    try {
      auto resource = static_cast<std::pmr::memory_resource *>(state);
      auto base = static_cast<char *>(resource->allocate(header + size, header));
      std::memcpy(base, &size, sizeof(size));
      return base + header;
    } catch (...) {
      return nullptr;
    }
  }

  static void deallocate_block(void * pointer, void * state) noexcept
  {
    if (pointer == nullptr) {
      return;
    }
    auto base = static_cast<char *>(pointer) - header;
    size_t size;
    std::memcpy(&size, base, sizeof(size));
    static_cast<std::pmr::memory_resource *>(state)->deallocate(base, header + size, header);
  }

  static void * reallocate_block(void * pointer, size_t size, void * state) noexcept
  {
    void * new_pointer = allocate_block(size, state);
    if (new_pointer == nullptr) {
      // like realloc: on failure the original block is left untouched
      return nullptr;
    }
    if (pointer != nullptr) {
      size_t old_size;
      std::memcpy(&old_size, static_cast<char *>(pointer) - header, sizeof(old_size));
      std::memcpy(new_pointer, pointer, old_size < size ? old_size : size);
      deallocate_block(pointer, state);
    }
    return new_pointer;
  }

  static void * zero_allocate_block(size_t count, size_t size, void * state) noexcept
  {
    if (size != 0 && count > (std::numeric_limits<size_t>::max)() / size) {
      return nullptr;
    }
    void * pointer = allocate_block(count * size, state);
    if (pointer != nullptr) {
      std::memset(pointer, 0, count * size);
    }
    return pointer;
  }
};

int main(int argc, char ** argv)
{
  using Alloc = PmrAllocator<void>;
  using MessageAllocTraits =
    rclcpp::allocator::AllocRebind<example_interfaces::msg::UInt32, Alloc>;
  using MessageAlloc = MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, example_interfaces::msg::UInt32>;
  using MessageUniquePtr = std::unique_ptr<example_interfaces::msg::UInt32, MessageDeleter>;
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node;

  std::list<std::string> keys = {"intra", "intraprocess", "intra-process", "intra_process"};
  bool intra_process = false;

  printf(
    "This simple demo shows off a custom memory allocator to count all\n"
    "instances of new/delete in the program.  It can be run in either regular\n"
    "mode (no arguments), or in intra-process mode (by passing 'intra' as a\n"
    "command-line argument).  It will then publish a message to the\n"
    "'/allocator_tutorial' topic every 10 milliseconds until Ctrl-C is pressed.\n"
    "At that time it will print a count of the number of allocations and\n"
    "deallocations that happened during the program.\n\n");

  if (argc > 1) {
    for (auto & key : keys) {
      if (std::string(argv[1]) == key) {
        intra_process = true;
        break;
      }
    }
  }

  if (intra_process) {
    printf("Intra-process pipeline is ON.\n");
    auto context = rclcpp::contexts::get_global_default_context();
    auto options = rclcpp::NodeOptions()
      .context(context)
      .use_intra_process_comms(true);

    node = rclcpp::Node::make_shared("allocator_tutorial", options);
  } else {
    auto options = rclcpp::NodeOptions()
      .use_intra_process_comms(false);
    printf("Intra-process pipeline is OFF.\n");
    node = rclcpp::Node::make_shared("allocator_tutorial", options);
  }

  uint32_t counter = 0;
  auto callback = [&counter](example_interfaces::msg::UInt32::ConstSharedPtr msg) -> void
    {
      (void)msg;
      ++counter;
    };

  // Create a custom allocator and pass the allocator to the publisher and subscriber.
  CustomMemoryResource mem_resource{};
  auto alloc = std::make_shared<Alloc>(&mem_resource);
  rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
  publisher_options.allocator = alloc;
  auto publisher = node->create_publisher<example_interfaces::msg::UInt32>(
    "allocator_tutorial", 10, publisher_options);

  rclcpp::SubscriptionOptionsWithAllocator<Alloc> subscription_options;
  subscription_options.allocator = alloc;
  auto msg_mem_strat = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      example_interfaces::msg::UInt32, Alloc>>(alloc);
  auto subscriber = node->create_subscription<example_interfaces::msg::UInt32>(
    "allocator_tutorial", 10, callback, subscription_options, msg_mem_strat);

  rclcpp::ExecutorOptions options;
  rclcpp::executors::SingleThreadedExecutor executor(options);

  // Add our node to the executor.
  executor.add_node(node);

  MessageDeleter message_deleter;
  MessageAlloc message_alloc = *alloc;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);

  rclcpp::sleep_for(1ms);
  is_running = true;

  uint32_t i = 0;
  while (rclcpp::ok()) {
    // Create a message with the custom allocator, so that when the Executor deallocates the
    // message on the execution path, it will use the custom deallocate.
    auto ptr = MessageAllocTraits::allocate(message_alloc, 1);
    MessageAllocTraits::construct(message_alloc, ptr);
    MessageUniquePtr msg(ptr, message_deleter);
    msg->data = i;
    ++i;
    publisher->publish(std::move(msg));
    rclcpp::sleep_for(10ms);
    executor.spin_some();
  }
  is_running = false;

  uint32_t final_global_allocs = global_runtime_allocs;
  uint32_t final_global_deallocs = global_runtime_deallocs;
  printf("Global new was called %u times during spin\n", final_global_allocs);
  printf("Global delete was called %u times during spin\n", final_global_deallocs);

  printf("Allocator new was called %u times during spin\n", num_allocs);
  printf("Allocator delete was called %u times during spin\n", num_deallocs);

  return 0;
}
