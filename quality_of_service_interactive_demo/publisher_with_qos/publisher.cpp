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

#include <atomic>
#include <cctype>
#include <condition_variable>
#include <iostream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#endif

#include "std_msgs/msg/string.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

//
// helper function
//
double rmw_time_to_seconds(const rmw_time_t & time)
{
  double result = time.sec;
  result += 1e-9 * time.nsec;
  return result;
}

class CommandGetter
{
public:
  bool is_active() const
  {
    return run_.load(std::memory_order_relaxed);
  }

  void operator()()
  {
    while (run_.load(std::memory_order_relaxed)) {
      char cmd = getch();
      handle_cmd(cmd);
    }
  }

  virtual void handle_cmd(const char cmd) const = 0;

  void start()
  {
    thread_ = std::thread(std::ref(*this));
    run_.store(true, std::memory_order_relaxed);
  }

  void stop()
  {
    run_.store(false, std::memory_order_relaxed);
    thread_.join();
  }

private:
  /* Read 1 character */
  char getch()
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

  std::thread thread_;
  std::atomic<bool> run_;
};

void print_qos(const rmw_qos_profile_t & qos)
{
  std::cout << "HISTORY POLICY: ";
  switch (qos.history) {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      std::cout << "keep last";
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      std::cout << "keep all";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << " (depth: " << qos.depth << ')' << std::endl;

  std::cout << "RELIABILITY POLICY: ";
  switch (qos.reliability) {
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
  switch (qos.durability) {
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

  std::cout << "DEADLINE: " << rmw_time_to_seconds(qos.deadline) << std::endl;

  std::cout << "LIFESPAN: " << rmw_time_to_seconds(qos.lifespan) << std::endl;

  std::cout << "LIVELINESS POLICY: ";
  switch (qos.liveliness) {
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
      std::cout << "automatic";
      break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE:
      std::cout << "manual by node";
      break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
      std::cout << "manual by topic";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << " (lease duration: " << rmw_time_to_seconds(qos.liveliness_lease_duration) <<
    ')' << std::endl;
}

class PublisherWithQOS : public rclcpp::Node
{
public:
  PublisherWithQOS(const rclcpp::QoS & qos_profile, const std::chrono::milliseconds & delay)
  : Node("publisher"), publish_delay_(delay), count_(0)
  {
    rclcpp::PublisherOptions publisher_options;
    publisher_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineOfferedInfo & event) {
        RCLCPP_INFO(this->get_logger(), "Deadline missed - total %d (delta %d)",
          event.total_count, event.total_count_change);
      };
    publisher_options.event_callbacks.liveliness_callback =
      [this](rclcpp::QOSLivelinessLostInfo & event) {
        RCLCPP_INFO(this->get_logger(), "Liveliness lost - total %d (delta %d)",
          event.total_count, event.total_count_change);
      };

    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "qos_chatter",
      qos_profile,
      publisher_options);

    print_qos();
    toggle_send();
  }

  bool assert_node_liveliness() const
  {
    std::cout << "asserting node liveliness" << std::endl;
    return assert_liveliness();
  }

  bool assert_publisher_liveliness() const
  {
    std::cout << "asserting publisher liveliness" << std::endl;
    return publisher_->assert_liveliness();
  }

  void toggle_send()
  {
    if (timer_ == nullptr) {
      std::cout << "start sending messages" << std::endl;
      timer_ = this->create_wall_timer(publish_delay_, [this] {timer_callback();});
    } else {
      std::cout << "stop sending messages" << std::endl;
      timer_->cancel();
      timer_ = nullptr;
    }
  }

  void print_qos() const
  {
    ::print_qos(publisher_->get_actual_qos());
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds publish_delay_;
  size_t count_;
};

static rclcpp::executors::SingleThreadedExecutor & get_executor()
{
  static rclcpp::executors::SingleThreadedExecutor exec;
  return exec;
}

class CommandHandler : public CommandGetter
{
public:
  CommandHandler(PublisherWithQOS * publisher)
  : publisher_(publisher) {}

  virtual void handle_cmd(const char command) const override
  {
    const char cmd = tolower(command);

    if (cmd == 'n') {
      // manually assert liveliness of node
      publisher_->assert_node_liveliness();
    } else if (cmd == 'p') {
      // manually assert liveliness of publisher
      publisher_->assert_publisher_liveliness();
    } else if (cmd == 's') {
      // toggle publishing of messages
      publisher_->toggle_send();
    } else if (cmd == 'q') {
      // print the qos settings
      publisher_->print_qos();
    } else if (cmd == 'x') {
      // signal program exit
      get_executor().cancel();
    }
  }

private:
  PublisherWithQOS * publisher_;
};

static constexpr char OPTION_HELP[] = "--help";
static constexpr char OPTION_PUBLISH_DELAY[] = "--delay";
static constexpr char OPTION_DEADLINE_PERIOD[] = "--deadline";
static constexpr char OPTION_LIVELINESS_KIND[] = "--liveliness";
static constexpr char OPTION_LEASE_DURATION[] = "--lease";

void print_usage(const char * progname)
{
  std::cout << progname << " [OPTIONS]" << std::endl <<
    std::endl << "When starting the program:" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << OPTION_HELP <<
    "print this help message" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << OPTION_PUBLISH_DELAY <<
    "the amount of delay between publishing subsequent messages" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << OPTION_DEADLINE_PERIOD <<
    "deadline period in seconds" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << OPTION_LIVELINESS_KIND <<
    "liveliness kind" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << OPTION_LEASE_DURATION <<
    "lease duration for liveliness in seconds" << std::endl <<
    std::endl <<

    "When the program is executing:" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << 'n' <<
    "manually assert the liveliness of the node" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << 'p' <<
    "manually assert the liveliness of the publisher" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << 's' <<
    "toggle start/stop of publishing messages" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << 'q' <<
    "print the QoS settings of the publisher" << std::endl <<
    std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Required arguments
  rclcpp::QoS qos_settings(10);
  std::chrono::milliseconds publish_delay = 500ms;

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_HELP)) {
    print_usage(argv[0]);
    return 0;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_DELAY)) {
    publish_delay = std::chrono::milliseconds(static_cast<int>(1000 *
        std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_PUBLISH_DELAY))));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_DEADLINE_PERIOD)) {
    auto period = std::chrono::milliseconds(static_cast<int>(1000 *
        std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_DEADLINE_PERIOD))));
    qos_settings.deadline(period);
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LIVELINESS_KIND)) {
    std::string kind = rcutils_cli_get_option(argv, argv + argc, OPTION_LIVELINESS_KIND);
    if (kind == "AUTOMATIC") {
      qos_settings.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    } else if (kind == "MANUAL_BY_NODE") {
      qos_settings.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE);
    } else if (kind == "MANUAL_BY_TOPIC") {
      qos_settings.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    } else {
      std::cout << "error: invalid liveliness kind specified" << std::endl <<
        "must be one of: AUTOMATIC, MANUAL_BY_NODE, MANUAL_BY_TOPIC" << std::endl;
      return -1;
    }
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LEASE_DURATION)) {
    auto duration = std::chrono::milliseconds(static_cast<int>(1000 *
        std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_LEASE_DURATION))));
    qos_settings.liveliness_lease_duration(duration);
  }

  auto node = std::make_shared<PublisherWithQOS>(qos_settings, publish_delay);
  CommandHandler cmd_handler(node.get());

  cmd_handler.start();
  get_executor().add_node(node);
  get_executor().spin();
  get_executor().remove_node(node);
  cmd_handler.stop();

  rclcpp::shutdown();

  return 0;
}
