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
#include "lifecycle/lifecycle_talker.hpp"

#include <memory>
#include <string>
#include <utility>


/// LifecycleTalker constructor
/**
 * The lifecycletalker/lifecyclenode constructor has the same
 * arguments a regular node.
 */
LifecycleTalker::LifecycleTalker(const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(node_name,
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{}

/// Callback for walltimer in order to publish the message.
/**
 * Callback for walltimer. This function gets invoked by the timer
 * and executes the publishing.
 * For this demo, we ask the node for its current state. If the
 * lifecycle publisher is not activate, we still invoke publish, but
 * the communication is blocked so that no messages is actually transferred.
 */
void
LifecycleTalker::publish()
{
  static size_t count = 0;
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

  // Print the current state for demo purposes
  if (!pub_->is_activated()) {
    RCLCPP_INFO(
      this->get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
  }

  // We independently from the current state call publish on the lifecycle
  // publisher.
  // Only if the publisher is in an active state, the message transfer is
  // enabled and the message actually published.
  pub_->publish(std::move(msg));
}

/// Transition callback for state configuring
/**
 * on_configure callback is being called when the lifecycle node
 * enters the "configuring" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "inactive" state or stays
 * in "unconfigured".
 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
 * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleTalker::on_configure(const rclcpp_lifecycle::State &)
{
  // This callback is supposed to be used for initialization and
  // configuring purposes.
  // We thus initialize and configure our publishers and timers.
  // The lifecycle node API does return lifecycle components such as
  // lifecycle publishers. These entities obey the lifecycle and
  // can comply to the current state of the node.
  // As of the beta version, there is only a lifecycle publisher
  // available.
  pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&LifecycleTalker::publish, this));

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  // We return a success and hence invoke the transition to the next
  // step: "inactive".
  // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
  // would stay in the "unconfigured" state.
  // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
  // this callback, the state machine transitions to state "errorprocessing".
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state activating
/**
 * on_activate callback is being called when the lifecycle node
 * enters the "activating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "active" state or stays
 * in "inactive".
 * TRANSITION_CALLBACK_SUCCESS transitions to "active"
 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleTalker::on_activate(const rclcpp_lifecycle::State &)
{
  // We explicitly activate the lifecycle publisher.
  // Starting from this point, all messages are no longer
  // ignored but sent into the network.
  pub_->on_activate();

  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  // Let's sleep for 2 seconds.
  // We emulate we are doing important
  // work in the activating phase.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // We return a success and hence invoke the transition to the next
  // step: "active".
  // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
  // would stay in the "inactive" state.
  // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
  // this callback, the state machine transitions to state "errorprocessing".
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state deactivating
/**
 * on_deactivate callback is being called when the lifecycle node
 * enters the "deactivating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "inactive" state or stays
 * in "active".
 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
 * TRANSITION_CALLBACK_FAILURE transitions to "active"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleTalker::on_deactivate(const rclcpp_lifecycle::State &)
{
  // We explicitly deactivate the lifecycle publisher.
  // Starting from this point, all messages are no longer
  // sent into the network.
  pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  // We return a success and hence invoke the transition to the next
  // step: "inactive".
  // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
  // would stay in the "active" state.
  // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
  // this callback, the state machine transitions to state "errorprocessing".
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup
/**
 * on_cleanup callback is being called when the lifecycle node
 * enters the "cleaningup" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "unconfigured" state or stays
 * in "inactive".
 * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleTalker::on_cleanup(const rclcpp_lifecycle::State &)
{
  // In our cleanup phase, we release the shared pointers to the
  // timer and publisher. These entities are no longer available
  // and our node is "clean".
  timer_.reset();
  pub_.reset();

  RCLCPP_INFO(this->get_logger(), "on cleanup is called.");

  // We return a success and hence invoke the transition to the next
  // step: "unconfigured".
  // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
  // would stay in the "inactive" state.
  // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
  // this callback, the state machine transitions to state "errorprocessing".
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state shutting down
/**
 * on_shutdown callback is being called when the lifecycle node
 * enters the "shuttingdown" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "finalized" state or stays
 * in its current state.
 * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
 * TRANSITION_CALLBACK_FAILURE transitions to current state
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleTalker::on_shutdown(const rclcpp_lifecycle::State & state)
{
  // In our shutdown phase, we release the shared pointers to the
  // timer and publisher. These entities are no longer available
  // and our node is "clean".
  timer_.reset();
  pub_.reset();

  RCLCPP_INFO(
    this->get_logger(),
    "on shutdown is called from state %s.",
    state.label().c_str());

  // We return a success and hence invoke the transition to the next
  // step: "finalized".
  // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
  // would stay in the current state.
  // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
  // this callback, the state machine transitions to state "errorprocessing".
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


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

  rclcpp::shutdown();

  return 0;
}
