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

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp/rclcpp.hpp"

#include "quality_of_service_demo/common_nodes.hpp"

#include "./utils.hpp"

using namespace std::chrono_literals;

static constexpr char OPTION_HELP[] = "--help";
static constexpr char OPTION_PUBLISH_DELAY[] = "--delay";
static constexpr char OPTION_DEADLINE_PERIOD[] = "--deadline";
static constexpr char OPTION_LIVELINESS_KIND[] = "--liveliness";
static constexpr char OPTION_LEASE_DURATION[] = "--lease";

void print_usage(const char * progname)
{
  std::cout << progname << " [OPTIONS]" << std::endl <<
    std::endl << "Options when starting the demo:" << std::endl <<
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

    "Commands when the demo is running:" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << 'p' <<
    "manually assert the liveliness of the publisher" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << 's' <<
    "toggle start/stop of publishing messages" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << 'q' <<
    "print the QoS settings of the publisher" << std::endl <<
    std::left << std::setw(14) << std::setfill(' ') << 'x' <<
    "exit the demo" << std::endl <<
    std::endl;
}

class PublisherCommandHandler : public CommandGetter
{
public:
  PublisherCommandHandler(
    rclcpp::executors::SingleThreadedExecutor & exec,
    std::weak_ptr<Talker> publisher)
  : exec_(exec), publisher_(publisher) {}

  void handle_cmd(const char command) const override
  {
    const char cmd = tolower(command);
    if (cmd == 'x') {
      // signal program exit
      exec_.cancel();
      std::cout << "exiting the demo..." << std::endl;
    } else if (auto publisher = publisher_.lock()) {
      if (cmd == 'p') {
        // manually assert liveliness of publisher
        publisher->assert_publisher_liveliness();
      } else if (cmd == 's') {
        // toggle publishing of messages
        publisher->toggle_publish();
      } else if (cmd == 'q') {
        // print the qos settings
        publisher->print_qos();
      }
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor & exec_;
  std::weak_ptr<Talker> publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  // Required arguments
  rclcpp::QoS qos_settings(10);
  std::chrono::milliseconds publish_delay = 500ms;

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_HELP)) {
    print_usage(argv[0]);
    return 0;
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_DELAY)) {
    publish_delay = std::chrono::milliseconds(
      static_cast<int>(1000 *
      std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_PUBLISH_DELAY))));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_DEADLINE_PERIOD)) {
    auto period = std::chrono::milliseconds(
      static_cast<int>(1000 *
      std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_DEADLINE_PERIOD))));
    qos_settings.deadline(period);
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LIVELINESS_KIND)) {
    std::string kind = rcutils_cli_get_option(argv, argv + argc, OPTION_LIVELINESS_KIND);
    if (kind == "AUTOMATIC") {
      qos_settings.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    } else if (kind == "MANUAL_BY_TOPIC") {
      qos_settings.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    } else {
      std::cout << "error: invalid liveliness kind specified" << std::endl <<
        "must be one of: AUTOMATIC, MANUAL_BY_TOPIC" << std::endl;
      return -1;
    }
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_LEASE_DURATION)) {
    auto duration = std::chrono::milliseconds(
      static_cast<int>(1000 *
      std::stof(rcutils_cli_get_option(argv, argv + argc, OPTION_LEASE_DURATION))));
    qos_settings.liveliness_lease_duration(duration);
  }

  auto talker = std::make_shared<Talker>(qos_settings, DEFAULT_TOPIC_NAME, 0, publish_delay);
  talker->get_options().event_callbacks.deadline_callback =
    [node = talker.get()](rclcpp::QOSDeadlineOfferedInfo & event) {
      RCLCPP_INFO(
        node->get_logger(), "Deadline missed - total %d (delta %d)",
        event.total_count, event.total_count_change);
    };
  talker->get_options().event_callbacks.liveliness_callback =
    [node = talker.get()](rclcpp::QOSLivelinessLostInfo & event) {
      RCLCPP_INFO(
        node->get_logger(), "Liveliness lost - total %d (delta %d)",
        event.total_count, event.total_count_change);
    };

  PublisherCommandHandler cmd_handler(exec, talker);

  talker->initialize();
  talker->print_qos();

  cmd_handler.start();
  exec.add_node(talker);
  exec.spin();
  exec.remove_node(talker);
  cmd_handler.stop();

  rclcpp::shutdown();

  return 0;
}
