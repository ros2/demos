#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# limitations under the License.import time

import time

from action_tutorials_interfaces.action import Fibonacci

import rclpy
<<<<<<< HEAD
from rclpy.action import ActionServer
=======
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
>>>>>>> f747b8f (support cancel handler in action_tutorials_py action server. (#727))
from rclpy.node import Node


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
<<<<<<< HEAD
            self.execute_callback)
=======
            self.execute_callback,
            cancel_callback=self.cancel_callback)
        self.add_on_set_parameters_callback(self.on_set_parameters_callback)
        self.add_post_set_parameters_callback(self.on_post_set_parameters_callback)
        self.declare_parameter('action_server_configure_introspection', 'disabled')

    def _check_parameter(self, parameter_list, parameter_name):
        result = SetParametersResult()
        result.successful = True
        for param in parameter_list:
            if param.name != parameter_name:
                continue

            if param.type_ != Parameter.Type.STRING:
                result.successful = False
                result.reason = 'must be a string'
                break

            if param.value not in ('disabled', 'metadata', 'contents'):
                result.successful = False
                result.reason = "must be one of 'disabled', 'metadata', or 'contents"
                break

        return result

    def on_set_parameters_callback(self, parameter_list) -> SetParametersResult:
        return self._check_parameter(parameter_list, 'action_server_configure_introspection')

    def on_post_set_parameters_callback(self, parameter_list):
        for param in parameter_list:
            if param.name != 'action_server_configure_introspection':
                continue

            introspection_state = ServiceIntrospectionState.OFF
            if param.value == 'disabled':
                introspection_state = ServiceIntrospectionState.OFF
            elif param.value == 'metadata':
                introspection_state = ServiceIntrospectionState.METADATA
            elif param.value == 'contents':
                introspection_state = ServiceIntrospectionState.CONTENTS

            self._action_server.configure_introspection(self.get_clock(),
                                                        qos_profile_system_default,
                                                        introspection_state)
            break
>>>>>>> f747b8f (support cancel handler in action_tutorials_py action server. (#727))

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
<<<<<<< HEAD
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
=======
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.sequence))
>>>>>>> f747b8f (support cancel handler in action_tutorials_py action server. (#727))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal...')
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    try:
<<<<<<< HEAD
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
=======
        with rclpy.init(args=args):
            fibonacci_action_server = FibonacciActionServer()
            executor = MultiThreadedExecutor()
            rclpy.spin(fibonacci_action_server, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
>>>>>>> f747b8f (support cancel handler in action_tutorials_py action server. (#727))
        pass


if __name__ == '__main__':
    main()
