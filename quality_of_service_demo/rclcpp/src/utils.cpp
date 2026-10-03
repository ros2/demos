// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#endif

#include <iostream>
#include <cctype>

#include "utils.hpp"

double
rmw_time_to_seconds(const rmw_time_t & time)
{
  double result = static_cast<double>(time.sec);
  result += 1e-9 * time.nsec;
  return result;
}

void
print_qos(const rclcpp::QoS & qos)
{
  const auto & rmw_qos = qos.get_rmw_qos_profile();
  std::cout << "HISTORY POLICY: ";
  switch (rmw_qos.history) {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      std::cout << "keep last";
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      std::cout << "keep all";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << " (depth: " << rmw_qos.depth << ')' << std::endl;

  std::cout << "RELIABILITY POLICY: ";
  switch (rmw_qos.reliability) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      std::cout << "reliable";
      break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      std::cout << "best effort";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << std::endl;

  std::cout << "DURABILITY POLICY: ";
  switch (rmw_qos.durability) {
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      std::cout << "transient local";
      break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      std::cout << "volatile";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << std::endl;

  std::cout << "DEADLINE: " << rmw_time_to_seconds(rmw_qos.deadline) << std::endl;

  std::cout << "LIFESPAN: " << rmw_time_to_seconds(rmw_qos.lifespan) << std::endl;

  std::cout << "LIVELINESS POLICY: ";
  switch (rmw_qos.liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      std::cout << "automatic";
      break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      std::cout << "manual by topic";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << " (lease duration: " << rmw_time_to_seconds(rmw_qos.liveliness_lease_duration) <<
    ')' << std::endl;
}


bool
CommandGetter::is_active() const
{
  return run_.load(std::memory_order_relaxed);
}

void
CommandGetter::start()
{
  thread_ = std::thread(std::ref(*this));
  run_.store(true, std::memory_order_relaxed);
}

void
CommandGetter::stop()
{
  run_.store(false, std::memory_order_relaxed);
  thread_.join();
}

void
CommandGetter::operator()() const
{
  while (run_.load(std::memory_order_relaxed)) {
    char cmd = getch();
    handle_cmd(cmd);
  }
}

char
CommandGetter::getch() const
{
#ifdef _WIN32
  char ch = _getch();
#else
  termios old_termios;
  tcgetattr(0, &old_termios);           /* grab old terminal i/o settings */

  termios new_termios = old_termios;    /* make new settings same as old settings */
  new_termios.c_lflag &= ~ICANON;       /* disable buffered i/o */
  new_termios.c_lflag &= ~ECHO;         /* set no echo mode */
  tcsetattr(0, TCSANOW, &new_termios);  /* use these new terminal i/o settings now */

  char ch = getchar();

  tcsetattr(0, TCSANOW, &old_termios);  /* restore old terminal i/o settings */
#endif
  return ch;
}
