#!/usr/bin/env python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetAvailableStates, GetAvailableTransitions, GetState
import rclpy


def change_state(lifecycle_node, change_state_args=''):
    node = rclpy.create_node('lc_client_py')

    service_name = lifecycle_node + '/change_state'
    cli = node.create_client(ChangeState, service_name)
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn(
            'Unable to call service %s' % service_name)
        return

    req = ChangeState.Request()
    if change_state_args == 'configure':
        req.transition.id = Transition.TRANSITION_CONFIGURE
    elif change_state_args == 'cleanup':
        req.transition.id = Transition.TRANSITION_CLEANUP
    elif change_state_args == 'shutdown':
        req.transition.id = Transition.TRANSITION_SHUTDOWN
    elif change_state_args == 'activate':
        req.transition.id = Transition.TRANSITION_ACTIVATE
    elif change_state_args == 'deactivate':
        req.transition.id = Transition.TRANSITION_DEACTIVATE
    cli.call(req)
    cli.wait_for_future()
    if cli.response.success:
        node.get_logger().info(
            '%s successfully triggered transition %s' % (lifecycle_node, change_state_args))
    else:
        node.get_logger().info(
            '%s failed to triggered transition %s' % (lifecycle_node, change_state_args))


def get_state(lifecycle_node):
    node = rclpy.create_node('lc_client_py')

    service_name = lifecycle_node + '/get_state'
    cli = node.create_client(GetState, service_name)
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn(
            'Unable to call service %s' % service_name)
        return

    req = GetState.Request()
    cli.call(req)
    cli.wait_for_future()
    node.get_logger().info(
        '%s is in state %s(%u)'
        % (lifecycle_node, cli.response.current_state.label, cli.response.current_state.id))


def get_available_states(lifecycle_node):
    node = rclpy.create_node('lc_client_py')

    service_name = lifecycle_node + '/get_available_states'
    cli = node.create_client(GetAvailableStates, service_name)
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn(
            'Unable to call service %s' % service_name)
        return

    req = GetAvailableStates.Request()
    cli.call(req)
    cli.wait_for_future()
    node.get_logger().info(
        '%s has %u available states' % (lifecycle_node, len(cli.response.available_states)))
    for state in cli.response.available_states:
        node.get_logger().info('id: %u\tlabel: %s' % (state.id, state.label))


def get_available_transitions(lifecycle_node):
    node = rclpy.create_node('lc_client_py')

    service_name = lifecycle_node + '/get_available_transitions'
    cli = node.create_client(GetAvailableTransitions, service_name)
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn(
            'Unable to call service %s' % service_name)
        return

    req = GetAvailableTransitions.Request()
    cli.call(req)
    cli.wait_for_future()
    node.get_logger().info(
        '%s has %u available transitions'
        % (lifecycle_node, len(cli.response.available_transitions)))
    for transition in cli.response.available_transitions:
        node.get_logger().info(
            'Transition id: %u\tlabel: %s'
            % (transition.transition.id, transition.transition.label))
        node.get_logger().info(
            '\tStart id: %u\tlabel: %s'
            % (transition.start_state.id, transition.start_state.label))
        node.get_logger().info(
            '\tGoal  id: %u\tlabel: %s'
            % (transition.goal_state.id, transition.goal_state.label))


def main(service_type, lifecycle_node, change_state_args='', args=None):
    rclpy.init(args=args)

    if service_type == 'change_state':
        change_state(lifecycle_node, change_state_args)

    if service_type == 'get_state':
        get_state(lifecycle_node)

    if service_type == 'get_available_states':
        get_available_states(lifecycle_node)

    if service_type == 'get_available_transitions':
        get_available_transitions(lifecycle_node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'service', action='store',
        choices=['change_state', 'get_state', 'get_available_states', 'get_available_transitions'],
        help='specifies lifeycle service to call.')
    parser.add_argument(
        '--change-state-args', action='store',
        choices=['configure', 'cleanup', 'shutdown', 'activate', 'deactivate'],
        help='specify the transation to trigger.')
    parser.add_argument('node', help='which node to address')
    args = parser.parse_args()
    main(args.service, args.node, args.change_state_args)
