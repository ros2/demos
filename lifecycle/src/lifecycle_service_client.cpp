// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

// service topics for node-attached services
static constexpr char const * lifecycle_node = "lc_talker";
static constexpr char const * node_get_state_topic = "lc_talker__get_state";
static constexpr char const * node_change_state_topic = "lc_talker__change_state";

class LifecycleServiceClient : public rclcpp::node::Node
{
public:
  explicit LifecycleServiceClient(const std::string & node_name)
  : rclcpp::node::Node(node_name)
  {}

  void
  init()
  {
    // Service client pointing to each individual service
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_change_state_topic);
  }

  unsigned int
  get_state(std::chrono::seconds time_out = 3_s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      fprintf(stderr, "Service %s is not available.\n",
        client_get_state_->get_service_name().c_str());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto result = client_get_state_->async_send_request(request);
    if (result.wait_for(std::chrono::milliseconds(time_out)) != std::future_status::ready) {
      fprintf(stderr, "[%s] Failed to get current state for node %s. Server timed out.\n",
        get_name().c_str(), lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (result.get()) {
      printf("[%s] Node %s has current state %s.\n",
        get_name().c_str(), lifecycle_node, result.get()->current_state.label.c_str());
      return result.get()->current_state.id;
    } else {
      fprintf(stderr, "[%s] Failed to get current state for node %s\n",
        get_name().c_str(), lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  bool
  change_state(std::uint8_t transition, std::chrono::seconds time_out = 3_s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      fprintf(stderr, "Service %s is not available.\n",
        client_change_state_->get_service_name().c_str());
      return false;
    }

    auto result = client_change_state_->async_send_request(request);
    if (result.wait_for(std::chrono::milliseconds(time_out)) != std::future_status::ready) {
      fprintf(stderr, "[%s] Failed to change state for node %s. Server timed out.\n",
        get_name().c_str(), lifecycle_node);
    }

    if (result.get()->success) {
      printf("[%s] Transition %d successfully triggered.\n",
        get_name().c_str(), static_cast<int>(transition));
      return true;
    } else {
      fprintf(stderr, "[%s] Failed to trigger transition %u\n",
        get_name().c_str(), static_cast<unsigned int>(transition));
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::client::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::client::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};

void
callee_script(std::shared_ptr<LifecycleServiceClient> lc_client)
{
  auto sleep_time = 10_s;

  {  // configure
    lc_client->change_state(lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING);
    lc_client->get_state();
  }
  {  // activate
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING);
    lc_client->get_state();
  }
  {  // deactivate
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING);
    lc_client->get_state();
  }
  {  // activate
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING);
    lc_client->get_state();
  }
  {  // deactivate
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING);
    lc_client->get_state();
  }
  {  // shutdown
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN);
    lc_client->get_state();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");
  lc_client->init();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(lc_client);

  std::shared_future<void> script = std::async(std::launch::async,
      std::bind(callee_script, lc_client));
  exe.spin_until_future_complete(script);

  return 0;
}
