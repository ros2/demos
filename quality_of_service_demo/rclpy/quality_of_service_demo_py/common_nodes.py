# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

from typing import Optional

from example_interfaces.msg import String
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rclpy.timer import Timer


class Talker(Node):
    def __init__(
        self,
        topic_name: str,
        qos_profile: QoSProfile,
        event_callbacks: Optional[PublisherEventCallbacks],
        publish_count: int = 0,
        assert_topic_period: Optional[float] = None
    ) -> None:
        """
        Create a Talker.

        @param topic_name: Topic to publish to.
        @param qos_profile QoS profile for Publisher.
        @param event_callbacks QoS Event callbacks for Publisher.
        @param publish_count Number of messages to publish before stopping.
        @param asert_topic_period How often to manually assert Publisher liveliness.
        """
        super().__init__('talker')
        self.get_logger().info('Talker starting up')
        self.publisher: Publisher[String] = self.create_publisher(
            String, topic_name, qos_profile,
            event_callbacks=event_callbacks)
        self.publish_timer: Timer = self.create_timer(0.5, self.publish)
        if assert_topic_period:
            self.assert_topic_timer: Optional[Timer] = self.create_timer(
                assert_topic_period, self.publisher.assert_liveliness)
        else:
            self.assert_topic_timer = None
        self.pause_timer: Optional[Timer] = None
        self.publish_count = 0
        self.stop_at_count = publish_count

    def pause_for(self, seconds: float) -> None:
        """
        Stop publishing for a while.

        Stops the Publisher for the specified amount of time.
        A message will be published immediately on the expiration of pause_duration.
        The regular publishing interval will resume at that point.
        If publishing is already paused, this call will be ignored.
        The remaining pause duration will not be affected.
        @param seconds Amount of time to pause for.
        """
        if self.pause_timer:
            return
        self.publish_timer.cancel()
        self.pause_timer = self.create_timer(seconds, self._pause_expired)

    def _pause_expired(self) -> None:
        self.publish()
        self.publish_timer.reset()
        if self.pause_timer is not None:
            self.destroy_timer(self.pause_timer)
        self.pause_timer = None

    def publish(self) -> None:
        """
        Publish a single message.

        Counts toward total message count that will be published.
        """
        message = String()
        message.data = 'Talker says {}'.format(self.publish_count)
        self.get_logger().info("Publishing: '{}'".format(message.data))
        self.publish_count += 1
        if self.stop_at_count > 0 and self.publish_count >= self.stop_at_count:
            self.publish_timer.cancel()
        self.publisher.publish(message)

    def stop(self) -> None:
        """Cancel publishing and any manual liveliness assertions."""
        if self.assert_topic_timer:
            self.assert_topic_timer.cancel()
        self.publish_timer.cancel()
        self.assert_topic_timer = None


class Listener(Node):

    def __init__(
        self,
        topic_name: str,
        qos_profile: QoSProfile,
        event_callbacks: Optional[SubscriptionEventCallbacks],
        defer_subscribe: bool = False
    ) -> None:
        """
        Create a Listener.

        @param topic_name Topic to subscribe to.
        @param qos_profile QoS profile for Subscription.
        @param event_callbacks QoS event callbacks for Subscription.
        @param defer_subscribe Don't create Subscription until user calls start_listening()
        """
        super().__init__('listener')
        self.subscription: Optional[Subscription[String]] = None
        self.topic_name = topic_name
        self.qos_profile = qos_profile
        self.event_callbacks = event_callbacks
        if not defer_subscribe:
            self.start_listening()

    def start_listening(self) -> None:
        """
        Instantiate Subscription.

        Does nothing if it has already been called.
        """
        if not self.subscription:
            self.subscription = self.create_subscription(
                String, self.topic_name, self._message_callback,
                self.qos_profile,
                event_callbacks=self.event_callbacks)
            self.get_logger().info('Listener starting up')

    def _message_callback(self, message: String) -> None:
        self.get_logger().info('Listener heard: [{}]'.format(message.data))
