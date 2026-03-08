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
# limitations under the License.

# The program has a short runtime, so you can directly set the parameter
# "action_client_configure_introspection" at execution command
# e.g.
# ros2 run action_tutorials_py fibonacci_action_client --ros-args -p
# "action_client_configure_introspection:=contents"

from typing import Union

from example_interfaces.action import Fibonacci

from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState
from rclpy.task import Future
from rclpy.type_support import FeedbackMessage
from rclpy.type_support import GetResultServiceResponse


class FibonacciActionClient(Node):

    def __init__(self) -> None:
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.add_on_set_parameters_callback(self.on_set_parameters_callback)
        self.add_post_set_parameters_callback(self.on_post_set_parameters_callback)
        self.declare_parameter('action_client_configure_introspection', 'disabled')

    def _check_parameter(self, parameter_list: list[Parameter[str]],
                         parameter_name: str) -> SetParametersResult:
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
                result.reason = "must be one of 'disabled', 'metadata', or 'contents'"
                break

        return result

    def on_set_parameters_callback(self,
                                   parameter_list: list[Parameter[str]]) -> SetParametersResult:
        return self._check_parameter(parameter_list, 'action_client_configure_introspection')

    def on_post_set_parameters_callback(self, parameter_list: list[Parameter[str]]) -> None:
        for param in parameter_list:
            if param.name != 'action_client_configure_introspection':
                continue

            introspection_state = ServiceIntrospectionState.OFF
            if param.value == 'disabled':
                introspection_state = ServiceIntrospectionState.OFF
            elif param.value == 'metadata':
                introspection_state = ServiceIntrospectionState.METADATA
            elif param.value == 'contents':
                introspection_state = ServiceIntrospectionState.CONTENTS

            self._action_client.configure_introspection(self.get_clock(),
                                                        qos_profile_system_default,
                                                        introspection_state)
            break

    def send_goal(self, order: int) -> None:
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(
            self,
            future: Future[
            ClientGoalHandle[Fibonacci.Goal,
                             Fibonacci.Result,
                             Fibonacci.Feedback,
                             Fibonacci.Impl]]) -> None:
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error('Exception while calling service')
            return

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future:
                            Future[GetResultServiceResponse[Fibonacci.Result]]) -> None:
        future_result = future.result()

        if future_result is None:
            self.get_logger().error('Exception while getting result')
            return
        result = future_result.result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg: FeedbackMessage[Fibonacci.Feedback]) -> None:
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback))


def main(args: Union[list[str], None] = None) -> None:
    try:
        with rclpy.init(args=args):
            action_client = FibonacciActionClient()

            action_client.send_goal(10)

            rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
