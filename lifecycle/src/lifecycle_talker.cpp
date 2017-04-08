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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/// LifeycycleTalker inheriting from rclcpp_lifecycle::LifecycleNode
/**
 * The lifecycle talker does not like the regular "talker" node
 * inherit from node, but rather from lifecyclenode. This brings
 * in a set of callbacks which are getting invoked depending on
 * the current state of the node.
 * Every lifecycle node has a set of services attached to it
 * which make it controllable from the outside and invoke state
 * changes.
 * Available Services as for Beta1:
 * - <node_name>__get_state
 * - <node_name>__change_state
 * - <node_name>__get_available_states
 * - <node_name>__get_available_transitions
 * Additionally, a publisher for state change notifications is
 * created:
 * - <node_name>__transition_event
 */
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// LifecycleTalker constructor
  /**
   * The lifecycletalker/lifecyclenode constructor has the same
   * arguments a regular node.
   */
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name, "", intra_process_comms)
  {}

  /// Callback for walltimer in order to publish the message.
  /**
   * Callback for walltimer. This function gets invoked by the timer
   * and executes the publishing.
   * For this demo, we ask the node for its current state. If the
   * lifecycle publisher is not activate, we still invoke publish, but
   * the communication is blocked so that no messages is actually transferred.
   */
  void publish()
  {
    static size_t count = 0;
    msg_->data = "Lifecycle HelloWorld #" + std::to_string(++count);

    // Print the current state for demo purposes
    if (!pub_->is_activated()) {
      printf("[%s] Lifecycle publisher is currently inactive. Messages are not published.\n",
        get_name());
    } else {
      printf("[%s] Lifecycle publisher is active. Publishing: [%s]\n",
        get_name(), msg_->data.c_str());
    }

    // We independently from the current state call publish on the lifecycle
    // publisher.
    // Only if the publisher is in an active state, the message transfer is
    // enabled and the message actually published.
    pub_->publish(msg_);
  }

  /// Transition callback for state configuring
  /**
   * on_configure callback is being called when the lifecycle node
   * enters the "configuring" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "unconfigured".
   * RCL_LIFECYCLE_RET_OK transitions to "inactive"
   * RCL_LIFECYCLE_RET_FAILURE transitions to "unconfigured"
   * RCL_LIFECYCLE_RET_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rcl_lifecycle_ret_t on_configure(const rclcpp_lifecycle::State &)
  {
    // This callback is supposed to be used for initialization and
    // configuring purposes.
    // We thus initalize and configure our messages, publishers and timers.
    // The lifecycle node API does return lifecycle components such as
    // lifecycle publishers. These entities obey the lifecycle and
    // can comply to the current state of the node.
    // As of the beta version, there is only a lifecycle publisher
    // available.
    msg_ = std::make_shared<std_msgs::msg::String>();
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter");
    timer_ = this->create_wall_timer(
      1s, std::bind(&LifecycleTalker::publish, this));

    printf("[%s] on_configure() is called.\n", get_name());

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned RCL_LIFECYCLE_RET_FAILURE instead, the state machine
    // would stay in the "unconfigured" state.
    // In case of RCL_LIFECYCLE_RET_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return RCL_LIFECYCLE_RET_OK;
  }

  /// Transition callback for state activating
  /**
   * on_activate callback is being called when the lifecycle node
   * enters the "activating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "active" state or stays
   * in "inactive".
   * RCL_LIFECYCLE_RET_OK transitions to "active"
   * RCL_LIFECYCLE_RET_FAILURE transitions to "inactive"
   * RCL_LIFECYCLE_RET_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rcl_lifecycle_ret_t on_activate(const rclcpp_lifecycle::State &)
  {
    // We explicitly activate the lifecycle publisher.
    // Starting from this point, all messages are no longer
    // ignored but sent into the network.
    pub_->on_activate();

    printf("[%s] on_activate() is called.\n", get_name());

    // Let's sleep for 2 seconds.
    // We emulate we are doing important
    // work in the activating phase.
    std::this_thread::sleep_for(2s);

    // We return a success and hence invoke the transition to the next
    // step: "active".
    // If we returned RCL_LIFECYCLE_RET_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of RCL_LIFECYCLE_RET_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return RCL_LIFECYCLE_RET_OK;
  }

  /// Transition callback for state activating
  /**
   * on_deactivate callback is being called when the lifecycle node
   * enters the "deactivating" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "inactive" state or stays
   * in "active".
   * RCL_LIFECYCLE_RET_OK transitions to "inactive"
   * RCL_LIFECYCLE_RET_FAILURE transitions to "active"
   * RCL_LIFECYCLE_RET_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rcl_lifecycle_ret_t on_deactivate(const rclcpp_lifecycle::State &)
  {
    // We explicitly deactivate the lifecycle publisher.
    // Starting from this point, all messages are no longer
    // sent into the network.
    pub_->on_deactivate();

    printf("[%s] on_deactivate() is called.\n", get_name());

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned RCL_LIFECYCLE_RET_FAILURE instead, the state machine
    // would stay in the "active" state.
    // In case of RCL_LIFECYCLE_RET_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return RCL_LIFECYCLE_RET_OK;
  }

  /// Transition callback for state deactivating
  /**
   * on_cleanup callback is being called when the lifecycle node
   * enters the "cleaningup" state.
   * Depending on the return value of this function, the state machine
   * either invokes a transition to the "uncofigured" state or stays
   * in "inactive".
   * RCL_LIFECYCLE_RET_OK transitions to "unconfigured"
   * RCL_LIFECYCLE_RET_FAILURE transitions to "inactive"
   * RCL_LIFECYCLE_RET_ERROR or any uncaught exceptions to "errorprocessing"
   */
  rcl_lifecycle_ret_t on_cleanup(const rclcpp_lifecycle::State &)
  {
    // In our cleanup phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    timer_.reset();
    pub_.reset();

    printf("[%s] on cleanup is called.\n", get_name());

    // We return a success and hence invoke the transition to the next
    // step: "unconfigured".
    // If we returned RCL_LIFECYCLE_RET_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of RCL_LIFECYCLE_RET_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return RCL_LIFECYCLE_RET_OK;
  }

private:
  std::shared_ptr<std_msgs::msg::String> msg_;

  // We hold an instance of a lifecycle publisher. This lifecycle publisher
  // can be activated or deactivated regarding on which state the lifecycle node
  // is in.
  // By default, a lifecycle publisher is inactive by creation and has to be
  // activated to publish messages into the ROS world.
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

  // We hold an instance of a timer which periodically triggers the publish function.
  // As for the beta version, this is a regular timer. In a future version, a
  // lifecycle timer will be created which obeys the same lifecycle management as the
  // lifecycle publisher.
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<LifecycleTalker> lc_node =
    std::make_shared<LifecycleTalker>("lc_talker");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  return 0;
}
