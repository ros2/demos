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

import argparse
import threading
import time

from example_interfaces.msg import String
from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.srv import GetLoggerLevels
from rcl_interfaces.srv import SetLoggerLevels
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

"""
This demo program shows how to enable logger service and control logger level via logger service.
Class LoggerServiceNode enables logger service, creates a child logger, and subscribes to a
topic. The subscription callback outputs received messages using both the parent and child
loggers at different severity levels, demonstrating how child loggers inherit or independently
override the parent logger level.
Class TestNode can set/get logger level of LoggerServiceNode and send message to it.

Usage:
  Default (demo mode): runs the full automated demo sequence with TestNode.
  --service-only:      starts only LoggerServiceNode and spins forever,
                        so you can interact with it via ros2 CLI tools.

  Example ros2 CLI commands (use in a separate terminal while --service-only is running).

  Get logger level (parent):
    ros2 service call /LoggerServiceNode/get_logger_levels \\
      rcl_interfaces/srv/GetLoggerLevels "{names: ['LoggerServiceNode']}"

  Get logger level (child):
    ros2 service call /LoggerServiceNode/get_logger_levels \\
      rcl_interfaces/srv/GetLoggerLevels "{names: ['LoggerServiceNode.child']}"

  Set parent logger level to DEBUG (10):
    ros2 service call /LoggerServiceNode/set_logger_levels \\
      rcl_interfaces/srv/SetLoggerLevels \\
      "{levels: [{name: 'LoggerServiceNode', level: 10}]}"

  Set child logger level to ERROR (40):
    ros2 service call /LoggerServiceNode/set_logger_levels \\
      rcl_interfaces/srv/SetLoggerLevels \\
      "{levels: [{name: 'LoggerServiceNode.child', level: 40}]}"

  Publish a test message:
    ros2 topic pub /output example_interfaces/msg/String "{data: hello}" --once
"""


class LoggerServiceNode(Node):

    def __init__(self):
        super().__init__('LoggerServiceNode', enable_logger_service=True)
        self.child_logger = self.get_logger().get_child('child')
        self.sub = self.create_subscription(String, 'output', self.callback, 10)

    def callback(self, msg):
        self.get_logger().debug(msg.data + ' with DEBUG logger level.')
        self.get_logger().info(msg.data + ' with INFO logger level.')
        self.get_logger().warning(msg.data + ' with WARN logger level.')
        self.get_logger().error(msg.data + ' with ERROR logger level.')
        self.child_logger.debug('[child] ' + msg.data + ' with DEBUG logger level.')
        self.child_logger.info('[child] ' + msg.data + ' with INFO logger level.')
        self.child_logger.warning('[child] ' + msg.data + ' with WARN logger level.')
        self.child_logger.error('[child] ' + msg.data + ' with ERROR logger level.')


class TestNode(Node):

    def __init__(self, remote_node_name):
        super().__init__('TestNode')
        self.pub = self.create_publisher(String, 'output', 10)
        self.logger_get_client = self.create_client(
            GetLoggerLevels, remote_node_name + '/get_logger_levels')
        self._logger_set_client = self.create_client(
            SetLoggerLevels, remote_node_name + '/set_logger_levels')
        self._remote_node_name = remote_node_name

    def set_logger_level_on_remote_node(self, logger_level, logger_name='') -> bool:
        if not self._logger_set_client.service_is_ready():
            return False

        request = SetLoggerLevels.Request()
        set_logger_level = LoggerLevel()
        set_logger_level.name = logger_name if logger_name else self._remote_node_name
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

    def get_logger_level_on_remote_node(self, logger_name=''):
        if not self.logger_get_client.service_is_ready():
            return [False, None]

        request = GetLoggerLevels.Request()
        request.names.append(logger_name if logger_name else self._remote_node_name)

        future = self.logger_get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        ret_results = future.result()
        if not ret_results:
            return [False, None]

        return [True, ret_results.levels[0].level]


def get_logger_level_func(test_node, child_logger_name):
    ret, level = test_node.get_logger_level_on_remote_node()
    if ret:
        test_node.get_logger().info('Current logger level: ' + str(level))
    else:
        test_node.get_logger().error('Failed to get logger level via logger service !')
    ret, child_level = test_node.get_logger_level_on_remote_node(child_logger_name)
    if ret:
        test_node.get_logger().info('Current child logger level: ' + str(child_level))
    else:
        test_node.get_logger().error('Failed to get child logger level via logger service !')


def main(args=None):
    # Check for --service-only flag before ROS 2 consumes the arguments
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--service-only', action='store_true', default=False)
    parsed, remaining = parser.parse_known_args(args)
    service_only = parsed.service_only

    try:
        with rclpy.init(args=remaining):
            node_name = 'LoggerServiceNode'
            child_logger_name = node_name + '.child'
            logger_service_node = LoggerServiceNode()

            if service_only:
                # Service-only mode: spin LoggerServiceNode forever so that users can
                # interact with it using ros2 CLI tools (ros2 service, ros2 topic, etc.)
                logger_service_node.get_logger().info(
                    'Started in service-only mode. Use ros2 CLI to interact with this node.')
                rclpy.spin(logger_service_node)
                return

            # Default demo mode: run the full automated sequence
            test_node = TestNode(node_name)

            executor = SingleThreadedExecutor()
            executor.add_node(logger_service_node)

            thread = threading.Thread(target=executor.spin)
            thread.start()

            logger = test_node.get_logger()

            # Output with default logger level
            logger.info('Output with default logger level:')
            msg = String()
            msg.data = 'Output 1'
            test_node.pub.publish(msg)
            time.sleep(0.5)

            # Get logger level. Logger level should be 0 (Unset)
            get_logger_level_func(test_node, child_logger_name)

            # Output with parent=debug, child=error logger level
            # Parent shows all messages, child only shows error.
            logger.info('Output with parent=debug, child=error logger level:')
            if (test_node.set_logger_level_on_remote_node(LoggingSeverity.DEBUG) and
                    test_node.set_logger_level_on_remote_node(
                        LoggingSeverity.ERROR, child_logger_name)):
                msg = String()
                msg.data = 'Output 2'
                test_node.pub.publish(msg)
                time.sleep(0.5)
            else:
                logger.error('Failed to set logger levels via logger service !')

            # Parent should be 10 (Debug), child should be 40 (Error)
            get_logger_level_func(test_node, child_logger_name)

            # Output with parent=warn, child=debug logger level
            # Parent suppresses debug/info, child shows everything.
            logger.info('Output with parent=warn, child=debug logger level:')
            if (test_node.set_logger_level_on_remote_node(LoggingSeverity.WARN) and
                    test_node.set_logger_level_on_remote_node(
                        LoggingSeverity.DEBUG, child_logger_name)):
                msg = String()
                msg.data = 'Output 3'
                test_node.pub.publish(msg)
                time.sleep(0.5)
            else:
                logger.error('Failed to set logger levels via logger service !')

            # Parent should be 30 (Warn), child should be 10 (Debug)
            get_logger_level_func(test_node, child_logger_name)

            # Output with parent=error, child=debug logger level
            # Parent only shows error, child shows everything.
            logger.info('Output with parent=error, child=debug logger level:')
            if (test_node.set_logger_level_on_remote_node(LoggingSeverity.ERROR) and
                    test_node.set_logger_level_on_remote_node(
                        LoggingSeverity.DEBUG, child_logger_name)):
                msg = String()
                msg.data = 'Output 4'
                test_node.pub.publish(msg)
                time.sleep(0.5)
            else:
                logger.error('Failed to set logger levels via logger service !')

            # Parent should be 40 (Error), child should be 10 (Debug)
            get_logger_level_func(test_node, child_logger_name)

            executor.shutdown()
            if thread.is_alive():
                thread.join()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
