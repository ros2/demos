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

import rclpy

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import GetState, ChangeState


def main(service_type, lifecycle_node, change_state_args='', args=None):
    rclpy.init(args)

    if not rclpy.ok():
        print("Something is wrong with rclpy init")
        return

    node = rclpy.create_node('lc_client_py')

    if service_type == 'get_state':
        cli = node.create_client(GetState, lifecycle_node + '__get_state')
        req = GetState.Request()
        cli.call(req)
        cli.wait_for_future()
        if cli.response.current_state.id == State.PRIMARY_STATE_UNKNOWN:
            print('%s is in state UNKNOWN' % lifecycle_node)
        elif cli.response.current_state.id == State.PRIMARY_STATE_UNCONFIGURED:
            print('%s is in state UNCONFIGURED' % lifecycle_node)
        elif cli.response.current_state.id == State.PRIMARY_STATE_INACTIVE:
            print('%s is in state INACTIVE' % lifecycle_node)
        elif cli.response.current_state.id == State.PRIMARY_STATE_ACTIVE:
            print('%s is in state ACTIVE' % lifecycle_node)
        elif cli.response.current_state.id == State.PRIMARY_STATE_FINALIZED:
            print('%s is in state FINALIZED' % lifecycle_node)
        elif cli.response.current_state.id == State.TRANSITION_STATE_CONFIGURING:
            print('%s is in transition CONFIGURING ' % lifecycle_node)
        elif cli.response.current_state.id == State.TRANSITION_STATE_CLEANINGUP:
            print('%s is in tansition CLEANINGUP' % lifecycle_node)
        elif cli.response.current_state.id == State.TRANSITION_STATE_SHUTTINGDOWN:
            print('%s is in transition SHUTTINGDOWN' % lifecycle_node)
        elif cli.response.current_state.id == State.TRANSITION_STATE_ACTIVATING:
            print('%s is in transition ACTIVATING' % lifecycle_node)
        elif cli.response.current_state.id == State.TRANSITION_STATE_DEACTIVATING:
            print('%s is in transition DEACTIVATING' % lifecycle_node)
        elif cli.response.current_state.id == State.TRANSITION_STATE_ERRORPROCESSING:
            print('%s is in transition ERROR' % lifecycle_node)
        else:
            print('%s unknown state %u' % (lifecycle_node, cli.response.current_state.id))

    if service_type == 'change_state':
        cli = node.create_client(ChangeState, lifecycle_node + '__change_state')
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
            print('%s successfully triggered transition %s' % (lifecycle_node, change_state_args))
        else:
            print('%s failed to triggered transition %s' % (lifecycle_node, change_state_args))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'service', action='store',
        choices=['get_state', 'change_state'], help='specifies lifeycle service to call.')
    parser.add_argument(
        '--change-state-args', action='store',
        choices=['configure', 'cleanup', 'shutdown', 'activate', 'deactivate'],
        help='specify the transation to trigger.')
    parser.add_argument('node', help='which node to address')
    args = parser.parse_args()
    main(args.service, args.node, args.change_state_args)
