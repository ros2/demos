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

#ifndef CBG_EXECUTOR_DEMO__THREAD_TIME_UTIL_HPP_
#define CBG_EXECUTOR_DEMO__THREAD_TIME_UTIL_HPP_

#include <chrono>
#include <thread>

#ifndef _WIN32  // i.e., POSIX platform.
#include <pthread.h>
#else  // i.e., Windows platform.
#include <windows.h>
#endif

namespace cbg_executor_demo
{

/// Returns the time of the given native thread handle as std::chrono
/// timestamp. This allows measuring the execution time of this thread.
template<typename T>
std::chrono::nanoseconds get_native_thread_time(T native_thread_handle)
{
#ifndef _WIN32  // i.e., POSIX platform.
  clockid_t id;
  pthread_getcpuclockid(native_thread_handle, &id);
  timespec spec;
  clock_gettime(id, &spec);
  return std::chrono::seconds{spec.tv_sec} + std::chrono::nanoseconds{spec.tv_nsec};
#else  // i.e., Windows platform.
  FILETIME creation_filetime;
  FILETIME exit_filetime;
  FILETIME kernel_filetime;
  FILETIME user_filetime;
  GetThreadTimes(
    native_thread_handle, &creation_filetime, &exit_filetime, &kernel_filetime, &user_filetime);
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

/// Returns the time of the given thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
std::chrono::nanoseconds get_thread_time(std::thread & thread);

/// Returns the time of the current thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
std::chrono::nanoseconds get_current_thread_time();

}  // namespace cbg_executor_demo

#endif  // CBG_EXECUTOR_DEMO__THREAD_TIME_UTIL_HPP_
