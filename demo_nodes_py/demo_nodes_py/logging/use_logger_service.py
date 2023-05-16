# Copyright 2023 Sony Group Corporation.
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

import threading
import time

from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.srv import GetLoggerLevels
from rcl_interfaces.srv import SetLoggerLevels
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import String

"""
This demo program shows how to enable logger service and control logger level via logger service.
Class LoggerServiceNode enable logger service and create a subscription. The callback of
subscription output received message by different log functions.
Class TestNode can set/get logger level of LoggerServiceNode and send message to it.
"""


class LoggerServiceNode(Node):

    def __init__(self):
        super().__init__('LoggerServiceNode', enable_logger_service=True)
        self.sub = self.create_subscription(String, 'output', self.callback, 10)

    def callback(self, msg):
        self.get_logger().debug(msg.data + ' with DEBUG logger level.')
        self.get_logger().info(msg.data + ' with INFO logger level.')
        self.get_logger().warn(msg.data + ' with WARN logger level.')
        self.get_logger().error(msg.data + ' with ERROR logger level.')


class TestNode(Node):

    def __init__(self, remote_node_name):
        super().__init__('TestNode')
        self.pub = self.create_publisher(String, 'output', 10)
        self.logger_get_client = self.create_client(
            GetLoggerLevels, remote_node_name + '/get_logger_levels')
        self._logger_set_client = self.create_client(
            SetLoggerLevels, remote_node_name + '/set_logger_levels')
        self._remote_node_name = remote_node_name

    def set_logger_level_on_remote_node(self, logger_level) -> bool:
        if not self._logger_set_client.service_is_ready():
            return False

        request = SetLoggerLevels.Request()
        set_logger_level = LoggerLevel()
        set_logger_level.name = self._remote_node_name
        set_logger_level.level = logger_level
        request.levels.append(set_logger_level)

        future = self._logger_set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        ret_results = future.result()
        if not ret_results:
            return False

        if not ret_results.results[0].successful:
            self.get_logger().error('Failed to change logger level: '
                                    + ret_results.results[0].reason)
            return False

        return True

    def get_logger_level_on_remote_node(self):
        if not self.logger_get_client.service_is_ready():
            return [False, None]

        request = GetLoggerLevels.Request()
        request.names.append(self._remote_node_name)

        future = self.logger_get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        ret_results = future.result()
        if not ret_results:
            return [False, None]

        return [True, ret_results.levels[0].level]


def get_logger_level_func(test_node):
    ret, level = test_node.get_logger_level_on_remote_node()
    if ret:
        test_node.get_logger().info('Current logger level: ' + str(level))
    else:
        test_node.get_logger().error('Failed to get logger level via logger service !')


def main(args=None):
    rclpy.init(args=args)

    logger_service_node = LoggerServiceNode()
    test_node = TestNode('LoggerServiceNode')

    executor = SingleThreadedExecutor()
    executor.add_node(logger_service_node)

    thread = threading.Thread(target=executor.spin)
    thread.start()

    # Output with default logger level
    test_node.get_logger().info('Output with default logger level:')
    msg = String()
    msg.data = 'Output 1'
    test_node.pub.publish(msg)
    time.sleep(0.5)

    # Get logger level. Logger level should be 0 (Unset)
    get_logger_level_func(test_node)

    # Output with debug logger level
    test_node.get_logger().info('Output with debug logger level:')
    if test_node.set_logger_level_on_remote_node(LoggingSeverity.DEBUG):
        msg = String()
        msg.data = 'Output 2'
        test_node.pub.publish(msg)
        time.sleep(0.5)
    else:
        test_node.get_logger().error('Failed to set debug logger level via logger service !')

    # Get logger level. Logger level should be 10 (Debug)
    get_logger_level_func(test_node)

    # Output with warn logger level
    test_node.get_logger().info('Output with warn logger level:')
    if test_node.set_logger_level_on_remote_node(LoggingSeverity.WARN):
        msg = String()
        msg.data = 'Output 3'
        test_node.pub.publish(msg)
        time.sleep(0.5)
    else:
        test_node.get_logger().error('Failed to set warn logger level via logger service !')

    # Get logger level. Logger level should be 30 (warn)
    get_logger_level_func(test_node)

    # Output with error logger level
    test_node.get_logger().info('Output with error logger level:')
    if test_node.set_logger_level_on_remote_node(LoggingSeverity.ERROR):
        msg = String()
        msg.data = 'Output 4'
        test_node.pub.publish(msg)
        time.sleep(0.5)
    else:
        test_node.get_logger().error('Failed to set error logger level via logger service !')

    # Get logger level. Logger level should be 40 (Error)
    get_logger_level_func(test_node)

    executor.shutdown()
    if thread.is_alive():
        thread.join()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
