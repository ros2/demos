// Copyright 2023 Sony Group Corporation.
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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// This demo program shows matched event work.
// Matched event occurs when publisher and subscription establishes the connection.
// Class MatchedEventDetectNode output connection information of publisher and subscription.
// Class MultiSubNode is used to create/destroy subscription to connect/disconnect the publisher of
// MatchedEventDetectNode.
// Class MultiPubNode is used to created/destroy publisher to connect/disconnect the subscription
// of MatchedEventDetectNode.

class MatchedEventDetectNode : public rclcpp::Node
{
public:
  explicit MatchedEventDetectNode(
    const std::string & pub_topic_name,
    const std::string & sub_topic_name)
  : Node("matched_event_detect_node")
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.event_callbacks.matched_callback =
      [this](rclcpp::MatchedInfo & s) {
        if (any_subscription_connected_) {
          if (s.current_count == 0) {
            RCLCPP_INFO(this->get_logger(), "Last subscription is disconnected.");
            any_subscription_connected_ = false;
          } else {
            RCLCPP_INFO(
              this->get_logger(),
              "The changed number of connected subscription is %d and current number of connected"
              " subscription is %lu.", s.current_count_change, s.current_count);
          }
        } else {
          if (s.current_count != 0) {
            RCLCPP_INFO(this->get_logger(), "First subscription is connected.");
            any_subscription_connected_ = true;
          }
        }
        promise_->set_value(true);
      };

    pub_ = create_publisher<std_msgs::msg::String>(
      pub_topic_name, 10, pub_options);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.event_callbacks.matched_callback =
      [this](rclcpp::MatchedInfo & s) {
        if (any_publisher_connected_) {
          if (s.current_count == 0) {
            RCLCPP_INFO(this->get_logger(), "Last publisher is disconnected.");
            any_publisher_connected_ = false;
          } else {
            RCLCPP_INFO(
              this->get_logger(),
              "The changed number of connected publisher is %d and current number of connected"
              " publisher is %lu.", s.current_count_change, s.current_count);
          }
        } else {
          if (s.current_count != 0) {
            RCLCPP_INFO(this->get_logger(), "First publisher is connected.");
            any_publisher_connected_ = true;
          }
        }
        promise_->set_value(true);
      };
    sub_ = create_subscription<std_msgs::msg::String>(
      sub_topic_name,
      10,
      [](std_msgs::msg::String::ConstSharedPtr) {},
      sub_options);
  }

  std::future<bool> get_future()
  {
    promise_.reset(new std::promise<bool>());
    return promise_->get_future();
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  bool any_subscription_connected_{false};
  bool any_publisher_connected_{false};
  std::shared_ptr<std::promise<bool>> promise_;
};

class MultiSubNode : public rclcpp::Node
{
public:
  explicit MultiSubNode(const std::string & topic_name)
  : Node("multi_sub_node"),
    topic_name_(topic_name)
  {}

  rclcpp::Subscription<std_msgs::msg::String>::WeakPtr create_one_sub(void)
  {
    RCLCPP_INFO(this->get_logger(), "Create a new subscription.");
    auto sub = create_subscription<std_msgs::msg::String>(
      topic_name_,
      10,
      [](std_msgs::msg::String::ConstSharedPtr) {});

    subs_.emplace_back(sub);
    return sub;
  }

  void destroy_one_sub(rclcpp::Subscription<std_msgs::msg::String>::WeakPtr sub)
  {
    auto sub_shared_ptr = sub.lock();
    if (sub_shared_ptr == nullptr) {
      return;
    }

    for (auto s = subs_.begin(); s != subs_.end(); s++) {
      if (*s == sub_shared_ptr) {
        RCLCPP_INFO(this->get_logger(), "Destroy a subscription.");
        subs_.erase(s);
        break;
      }
    }
  }

private:
  std::string topic_name_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
};

class MultiPubNode : public rclcpp::Node
{
public:
  explicit MultiPubNode(const std::string & topic_name)
  : Node("multi_pub_node"),
    topic_name_(topic_name)
  {}

  rclcpp::Publisher<std_msgs::msg::String>::WeakPtr create_one_pub(void)
  {
    RCLCPP_INFO(this->get_logger(), "Create a new publisher.");
    auto pub = create_publisher<std_msgs::msg::String>(topic_name_, 10);
    pubs_.emplace_back(pub);

    return pub;
  }

  void destroy_one_pub(rclcpp::Publisher<std_msgs::msg::String>::WeakPtr pub)
  {
    auto pub_shared_ptr = pub.lock();
    if (pub_shared_ptr == nullptr) {
      return;
    }

    for (auto s = pubs_.begin(); s != pubs_.end(); s++) {
      if (*s == pub_shared_ptr) {
        RCLCPP_INFO(this->get_logger(), "Destroy a publisher.");
        pubs_.erase(s);
        break;
      }
    }
  }

private:
  std::string topic_name_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  std::string topic_name_for_detect_pub_matched_event = "pub_topic_matched_event_detect";
  std::string topic_name_for_detect_sub_matched_event = "sub_topic_matched_event_detect";

  rclcpp::executors::SingleThreadedExecutor executor;

  auto matched_event_detect_node = std::make_shared<MatchedEventDetectNode>(
    topic_name_for_detect_pub_matched_event,
    topic_name_for_detect_sub_matched_event);

  auto multi_sub_node = std::make_shared<MultiSubNode>(
    topic_name_for_detect_pub_matched_event);

  auto multi_pub_node = std::make_shared<MultiPubNode>(
    topic_name_for_detect_sub_matched_event);

  auto maximum_wait_time = 10s;

  executor.add_node(matched_event_detect_node);
  executor.add_node(multi_sub_node);
  executor.add_node(multi_pub_node);

  // MatchedEventDetectNode will output:
  // First subscription is connected.
  auto sub1 = multi_sub_node->create_one_sub();
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // MatchedEventDetectNode will output:
  // The changed number of connected subscription is 1 and current number of connected subscription
  // is 2.
  auto sub2 = multi_sub_node->create_one_sub();
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // MatchedEventDetectNode will output:
  // The changed number of connected subscription is -1 and current number of connected subscription
  // is 1.
  multi_sub_node->destroy_one_sub(sub1);
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // MatchedEventDetectNode will output:
  // Last subscription is disconnected.
  multi_sub_node->destroy_one_sub(sub2);
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // MatchedEventDetectNode will output:
  // First publisher is connected.
  auto pub1 = multi_pub_node->create_one_pub();
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // MatchedEventDetectNode will output:
  // The changed number of connected publisher is 1 and current number of connected publisher
  // is 2.
  auto pub2 = multi_pub_node->create_one_pub();
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // MatchedEventDetectNode will output:
  // The changed number of connected publisher is -1 and current number of connected publisher
  // is 1.
  multi_pub_node->destroy_one_pub(pub1);
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  // MatchedEventDetectNode will output:
  // Last publisher is disconnected.
  multi_pub_node->destroy_one_pub(pub2);
  executor.spin_until_future_complete(matched_event_detect_node->get_future(), maximum_wait_time);

  rclcpp::shutdown();
  return 0;
}
