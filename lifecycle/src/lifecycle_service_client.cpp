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

// which node to handle
static constexpr char const * lifecycle_node = "lc_talker";
/*
 * Every lifecycle node has various services
 * attached to it. By convention, we use its
 * node name followed by a doule underscore
 * and the service name.
 * In this demo, we use get_state and change_state
 * and thus the two service topics are:
 * lc_talker__get_state
 * lc_talker__change_state
 */
static constexpr char const * node_get_state_topic = "lc_talker__get_state";
static constexpr char const * node_change_state_topic = "lc_talker__change_state";

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleServiceClient : public rclcpp::node::Node
{
public:
  explicit LifecycleServiceClient(const std::string & node_name)
  : rclcpp::node::Node(node_name)
  {}

  void
  init()
  {
    /*
     * Every lifecycle node spawns automatically a couple
     * of services which allow an external interaction with
     * these nodes.
     * The two main important ones are GetState and ChangeState.
     */
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_change_state_topic);
  }

  /*
   * In this function, we send a service request
   * asking for the current state of the node
   * lc_talker.
   * If it does return within the given time_out,
   * we return the current state of the node, if
   * not, we return an unknown state.
   */
  unsigned int
  get_state(std::chrono::seconds time_out = 3_s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      fprintf(stderr, "Service %s is not available.\n",
        client_get_state_->get_service_name().c_str());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    /*
     * We send the service request for asking the current
     * state of the lc_talker node.
     */
    auto future_result = client_get_state_->async_send_request(request);
    /*
     * Let's wait until we have the answer from the node.
     * If the request times out, we return an unknown state.
     */
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      fprintf(stderr, "[%s] Failed to get current state for node %s. Server timed out.\n",
        get_name(), lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    /*
     * We have an succesful answer. So let's print the
     * current state.
     */
    if (future_result.get()) {
      printf("[%s] Node %s has current state %s.\n",
        get_name(), lifecycle_node, future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      fprintf(stderr, "[%s] Failed to get current state for node %s\n",
        get_name(), lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  /*
   * We send a Service request and indicate
   * that we want to invoke transition with
   * the id "transition".
   * By default, these transitions are
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   */
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

    /*
     * We send the request with the transition
     * we want to invoke.
     */
    auto future_result = client_change_state_->async_send_request(request);
    /*
     * Let's wait until we have the answer from the node.
     * If the request times out, we return an unknown state.
     */
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      fprintf(stderr, "[%s] Failed to change state for node %s. Server timed out.\n",
        get_name(), lifecycle_node);
      return false;
    }

    /*
     * We have an answer, let's print
     * our success.
     */
    if (future_result.get()->success) {
      printf("[%s] Transition %d successfully triggered.\n",
        get_name(), static_cast<int>(transition));
      return true;
    } else {
      fprintf(stderr, "[%s] Failed to trigger transition %u\n",
        get_name(), static_cast<unsigned int>(transition));
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::client::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::client::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};

/*
 * This is a little independent
 * script which triggers the
 * default lifecycle of a node.
 * It starts with configure, activate,
 * deactivate, activate, deactivate,
 * cleanup and finally shutdown
 */
void
callee_script(std::shared_ptr<LifecycleServiceClient> lc_client)
{
  auto sleep_time = 10_s;

  // configure
  {
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // activate
  {
    std::this_thread::sleep_for(sleep_time);
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // deactivate
  {
    std::this_thread::sleep_for(sleep_time);
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // activate it again
  {
    std::this_thread::sleep_for(sleep_time);
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // and deactivate it again
  {
    std::this_thread::sleep_for(sleep_time);
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // we cleanup
  {
    std::this_thread::sleep_for(sleep_time);
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // and finally shutdown
  {
    std::this_thread::sleep_for(sleep_time);
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_SHUTDOWN)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
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
