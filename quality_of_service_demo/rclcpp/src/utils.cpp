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

#include <cctype>
#include <functional>
#include <iostream>
#include <stdexcept>

#include "utils.hpp"

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#else
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#endif

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

class KeyboardReader::KeyboardReaderImpl final
{
public:
  KeyboardReaderImpl()
  {
#ifdef _WIN32
    hstdin_ = GetStdHandle(STD_INPUT_HANDLE);
    if (hstdin_ == INVALID_HANDLE_VALUE) {
      throw std::runtime_error("Failed to get stdin handle");
    }
    if (!GetConsoleMode(hstdin_, &old_mode_)) {
      throw std::runtime_error("Failed to get old console mode");
    }
    DWORD new_mode = ENABLE_PROCESSED_INPUT;  // for Ctrl-C processing
    if (!SetConsoleMode(hstdin_, new_mode)) {
      throw std::runtime_error("Failed to set new console mode");
    }
#else
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0) {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0) {
      throw std::runtime_error("Failed to set new console mode");
    }
#endif
  }

  char readOne()
  {
    char c = 0;

#ifdef _WIN32
    INPUT_RECORD record;
    DWORD num_read;
    switch (WaitForSingleObject(hstdin_, 100)) {
      case WAIT_OBJECT_0:
        if (!ReadConsoleInput(hstdin_, &record, 1, &num_read)) {
          throw std::runtime_error("Read failed");
        }

        if (record.EventType != KEY_EVENT || !record.Event.KeyEvent.bKeyDown) {
          break;
        }

        c = static_cast<char>(record.Event.KeyEvent.wVirtualKeyCode);

        break;

      case WAIT_TIMEOUT:
        break;
    }

#else
    int rc = read(0, &c, 1);
    if (rc < 0) {
      throw std::runtime_error("read failed");
    }
#endif

    return c;
  }

  ~KeyboardReaderImpl()
  {
#ifdef _WIN32
    SetConsoleMode(hstdin_, old_mode_);
#else
    tcsetattr(0, TCSANOW, &cooked_);
#endif
  }

private:
#ifdef _WIN32
  HANDLE hstdin_;
  DWORD old_mode_;
#else
  struct termios cooked_;
#endif
};

KeyboardReader::KeyboardReader()
: pimpl_(new KeyboardReaderImpl)
{
}

char KeyboardReader::readOne()
{
  return pimpl_->readOne();
}

KeyboardReader::~KeyboardReader() = default;

static std::function<void(void)> user_ctrl_handler;

#ifdef _WIN32
static BOOL WINAPI console_ctrl_handler(DWORD ctrl_type)
{
  (void)ctrl_type;
  if (user_ctrl_handler) {
    user_ctrl_handler();
  }
  return true;
}
#else
static void signal_handler(int sig)
{
  (void)sig;
  if (user_ctrl_handler) {
    user_ctrl_handler();
  }
}
#endif

void install_ctrl_handler(std::function<void(void)> ctrl_handler)
{
  user_ctrl_handler = ctrl_handler;
#ifdef _WIN32
  SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
#else
  signal(SIGINT, signal_handler);
#endif
}
