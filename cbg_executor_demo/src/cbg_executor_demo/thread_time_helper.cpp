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

#include "cbg_executor_demo/thread_time_helper.hpp"

namespace cbg_executor_demo
{

/// Returns the time of the given thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
std::chrono::nanoseconds get_thread_time(std::thread & thread)
{
  return get_native_thread_time(thread.native_handle());
}

/// Returns the time of the current thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
std::chrono::nanoseconds get_current_thread_time()
{
#ifndef _WIN32  // i.e., POSIX platform.
  return get_native_thread_time(pthread_self());
#else  // i.e., Windows platform.
  return get_native_thread_time(GetCurrentThread());
#endif
}

}  // namespace cbg_executor_demo
