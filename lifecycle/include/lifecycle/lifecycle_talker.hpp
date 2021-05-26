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
#ifndef LIFECYCLE__LIFECYCLE_TALKER_HPP_
#define LIFECYCLE__LIFECYCLE_TALKER_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"

/// LifecycleTalker inheriting from rclcpp_lifecycle::LifecycleNode
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
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false);

  /// Callback for walltimer in order to publish the message.
  /**
   * Callback for walltimer. This function gets invoked by the timer
   * and executes the publishing.
   * For this demo, we ask the node for its current state. If the
   * lifecycle publisher is not activate, we still invoke publish, but
   * the communication is blocked so that no messages is actually transferred.
   */
  void publish();

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
  on_configure(const rclcpp_lifecycle::State &);

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
  on_activate(const rclcpp_lifecycle::State &);

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
  on_deactivate(const rclcpp_lifecycle::State &);

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
  on_cleanup(const rclcpp_lifecycle::State &);

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
  on_shutdown(const rclcpp_lifecycle::State & state);

private:
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

#endif  // LIFECYCLE__LIFECYCLE_TALKER_HPP_
