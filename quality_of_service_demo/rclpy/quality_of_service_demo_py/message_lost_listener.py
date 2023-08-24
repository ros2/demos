# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import sys

import rclpy
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image


class MessageLostListener(Node):
    """Listener node to demonstrate how to get a notification on lost messages."""

    def __init__(self):
        """Create a MessageLostListener."""
        super().__init__('message_lost_listener')

        # Create an object describing the event handlers that will
        # be registered in the subscription.
        # In this case, only a handler for a message lost event is registered.
        event_callbacks = SubscriptionEventCallbacks(
            message_lost=self._message_lost_event_callback)
        # Create a subscription, passing the previously created event handlers.
        self.subscription = self.create_subscription(
            Image,
            'message_lost_chatter',
            self._message_callback,
            1,
            event_callbacks=event_callbacks)

    def _message_callback(self, message):
        """Log when a message is received."""
        now = self.get_clock().now()
        diff = now - Time.from_msg(message.header.stamp)
        self.get_logger().info(
            f'I heard an Image. Message single trip latency: [{diff.nanoseconds}]\n---')

    def _message_lost_event_callback(self, message_lost_status):
        """Log the number of lost messages when the event is triggered."""
        self.get_logger().info(
            'Some messages were lost:\n>\tNumber of new lost messages: '
            f'{message_lost_status.total_count_change}'
            f' \n>\tTotal number of messages lost: {message_lost_status.total_count}',
        )


def main(args=None):
    rclpy.init(args=args)

    listener = MessageLostListener()
    executor = SingleThreadedExecutor()
    executor.add_node(listener)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
