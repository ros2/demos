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

#include <chrono>
#include <memory>
#include <string>

#include "rcutils/cmdline_parser.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "quality_of_service_demo/common_nodes.hpp"

static const char * OPTION_POLICY = "--policy";
static const char * DEFAULT_POLICY = "AUTOMATIC";
static const char * OPTION_NODE_ASSERT_PERIOD = "--node-assert-period";
static const char * OPTION_TOPIC_ASSERT_PERIOD = "--topic-assert-period";
static const char * OPTION_KILL_PUBLISHER_AFTER = "--kill-publisher-after";
static const size_t DEFAULT_KILL_PUBLISHER_AFTER = 3000;

void print_usage()
{
  printf("Usage for liveliness:\n");
  printf("liveliness lease_duration [-p liveliness_policy] [-n node_assert_liveliness_period] "
    "[-t topic_assert_liveliness_period] [-h]\n");
  printf("required arguments:\n");
  printf("lease_duration: Duration (in ms) before an inactive Publisher is considered not-alive. "
    "0 means infinity.\n");
  printf("options:\n");
  printf("-h : Print this help message.\n");
  printf("%s liveliness_policy : You may specify AUTOMATIC, MANUAL_BY_NODE, or MANUAL_BY_TOPIC. "
    "Defaults to %s\n", OPTION_POLICY, DEFAULT_POLICY);
  printf("%s node_assert_period : How often the Publisher will assert the liveliness of its "
    "Node (in uint milliseconds). Defaults to 0 (never)\n", OPTION_NODE_ASSERT_PERIOD);
  printf("%s topic_assert_period : How often the Publisher will assert the liveliness of its "
    "Publisher (in uint milliseconds). Defaults to 0 (never)\n", OPTION_TOPIC_ASSERT_PERIOD);
  printf("%s kill_publisher_after : "
    "Kill the publisher after this amount of time (in uint milliseconds). "
    "In AUTOMATIC - destroy the whole node. "
    "In MANUAL_BY_NODE, stop node liveliness assertion. "
    "In MANUAL_BY_TOPIC, stop topic liveliness assertion. "
    "Defaults to %lu\n",
    OPTION_KILL_PUBLISHER_AFTER, DEFAULT_KILL_PUBLISHER_AFTER);
}

class LivelinessDemo
{
public:
  LivelinessDemo(
    const std::string & topic,
    rmw_qos_liveliness_policy_t liveliness_policy_kind,
    std::chrono::milliseconds liveliness_lease_duration,
    std::chrono::milliseconds node_assert_period,
    std::chrono::milliseconds topic_assert_period,
    std::chrono::milliseconds kill_publisher_after)
  : liveliness_policy_kind_(liveliness_policy_kind)
  {
    rclcpp::QoS qos_profile(10);
    qos_profile.liveliness(liveliness_policy_kind);
    qos_profile.liveliness_lease_duration(liveliness_lease_duration);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.event_callbacks.liveliness_callback = std::bind(
      &LivelinessDemo::print_liveliness_changed_event, this, std::placeholders::_1);

    rclcpp::PublisherOptions publisher_options;

    listener_ = std::make_shared<Listener>(
      topic, qos_profile, subscription_options);
    talker_ = std::make_shared<Talker>(
      topic, qos_profile, publisher_options, 0, node_assert_period, topic_assert_period);

    executor.add_node(listener_);
    executor.add_node(talker_);

    kill_publisher_timer_ = listener_->create_wall_timer(
      kill_publisher_after, std::bind(&LivelinessDemo::kill_publisher, this));
  }

  void
  spin()
  {
    executor.spin();
  }

private:
  void
  print_liveliness_changed_event(rclcpp::QOSLivelinessChangedInfo & event)
  {
    printf("Liveliness changed event: \n");
    printf("  alive_count: %d\n", event.alive_count);
    printf("  not_alive_count: %d\n", event.not_alive_count);
    printf("  alive_count_change: %d\n", event.alive_count_change);
    printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
  }

  void
  kill_publisher()
  {
    if (!talker_) {
      return;
    }
    switch (liveliness_policy_kind_) {
      case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
        executor.remove_node(talker_);
        talker_ = nullptr;
        break;
      case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE:
        talker_->stop_asserting_liveliness();
        break;
      case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
        talker_->stop_asserting_liveliness();
        break;
      default:
        break;
    }
  }

  rmw_qos_liveliness_policy_t liveliness_policy_kind_;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<Talker> talker_ = nullptr;
  std::shared_ptr<Listener> listener_ = nullptr;
  rclcpp::TimerBase::SharedPtr kill_publisher_timer_ = nullptr;
};

int main(int argc, char * argv[])
{
  // Argument count and usage
  if (argc < 2 || rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }
  // Configuration variables
  std::chrono::milliseconds liveliness_lease_duration(std::stoul(argv[1]));
  std::chrono::milliseconds node_assert_period(0);
  std::chrono::milliseconds topic_assert_period(0);
  std::chrono::milliseconds kill_publisher_after(DEFAULT_KILL_PUBLISHER_AFTER);
  const char * policy_name = DEFAULT_POLICY;
  rmw_qos_liveliness_policy_t liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  std::string topic("qos_liveliness_chatter");

  // Argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_NODE_ASSERT_PERIOD)) {
    node_assert_period = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_NODE_ASSERT_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_TOPIC_ASSERT_PERIOD)) {
    topic_assert_period = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_TOPIC_ASSERT_PERIOD)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_KILL_PUBLISHER_AFTER)) {
    kill_publisher_after = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_KILL_PUBLISHER_AFTER)));
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_POLICY)) {
    policy_name = rcutils_cli_get_option(argv, argv + argc, OPTION_POLICY);
  }

  if (strcmp(policy_name, "AUTOMATIC") == 0) {
    liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  } else if (strcmp(policy_name, "MANUAL_BY_NODE") == 0) {
    liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE;
  } else if (strcmp(policy_name, "MANUAL_BY_TOPIC") == 0) {
    liveliness_policy_kind = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  } else {
    printf("Unknown liveliness policy: %s\n", policy_name);
    print_usage();
    return 1;
  }

  // Demo execution
  rclcpp::init(argc, argv);
  LivelinessDemo demo(
    topic,
    liveliness_policy_kind,
    liveliness_lease_duration,
    node_assert_period,
    topic_assert_period,
    kill_publisher_after);
  demo.spin();
  rclcpp::shutdown();
  return 0;
}
