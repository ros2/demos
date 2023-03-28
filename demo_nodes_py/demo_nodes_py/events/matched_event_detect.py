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

import rclpy
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.event_handler import QoSPublisherMatchedInfo
from rclpy.event_handler import QoSSubscriptionMatchedInfo
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.task import Future
from std_msgs.msg import String

"""
This demo program shows detected matched event.
Matched event occurs when publisher and subscription establishes the connection.

Class MatchedEventDetectNode output connection information of publisher and subscription.
Class MultiSubNode is used to create/destroy subscription to connect/disconnect the publisher of
MatchedEventDetectNode.
Class MultiPubNode is used to created/destroy publisher to connect/disconnect the subscription
of MatchedEventDetectNode.
"""


class MatchedEventDetectNode(Node):

    def __init__(self, pub_topic_name: String, sub_topic_name: String):
        super().__init__('matched_event_detection_node')
        self.__any_subscription_connected = False  # used for publisher event
        self.__any_publisher_connected = False  # used for subscription event

        pub_event_callback = PublisherEventCallbacks(matched=self.__pub_matched_event_callback)
        self.pub = self.create_publisher(String, pub_topic_name, 10,
                                         event_callbacks=pub_event_callback)

        sub_event_callback = SubscriptionEventCallbacks(matched=self.__sub_matched_event_callback)
        self.sub = self.create_subscription(String, sub_topic_name, lambda msg: ...,
                                            10, event_callbacks=sub_event_callback)

    def __pub_matched_event_callback(self, info: QoSPublisherMatchedInfo):
        if self.__any_subscription_connected:
            if info.current_count == 0:
                self.get_logger().info('Last subscription is disconnected.')
                self.__any_subscription_connected = False
            else:
                self.get_logger().info('The changed number of connected subscription is '
                                       + str(info.current_count_change) + ' and '
                                       'current number of connected subscription is '
                                       + str(info.current_count))
        else:
            if info.current_count != 0:
                self.get_logger().info('First subscription is connected.')
                self.__any_subscription_connected = True

        self.future.set_result(True)

    def __sub_matched_event_callback(self, info: QoSSubscriptionMatchedInfo):
        if self.__any_publisher_connected:
            if info.current_count == 0:
                self.get_logger().info('Last publisher is disconnected.')
                self.__any_publisher_connected = False
            else:
                self.get_logger().info('The changed number of connected publisher is '
                                       + str(info.current_count_change) + ' and current '
                                       'number of connected publisher is '
                                       + str(info.current_count))
        else:
            if info.current_count != 0:
                self.get_logger().info('First publisher is connected.')
                self.__any_publisher_connected = True

        self.future.set_result(True)

    def get_future(self):
        self.future = Future()
        return self.future


class MultiSubNode(Node):

    def __init__(self, topic_name: String):
        super().__init__('multi_sub_node')
        self.__subs = []
        self.__topic_name = topic_name

    def create_one_sub(self) -> Subscription:
        self.get_logger().info('Create a new subscription.')
        sub = self.create_subscription(String, self.__topic_name, lambda msg: ..., 10)
        self.__subs.append(sub)
        return sub

    def destroy_one_sub(self, sub: Subscription):

        if sub in self.__subs:
            self.get_logger().info('Destroy a subscription.')
            self.__subs.remove(sub)
            self.destroy_subscription(sub)


class MultiPubNode(Node):

    def __init__(self, topic_name: String):
        super().__init__('multi_pub_node')
        self.__pubs = []
        self.__topic_name = topic_name

    def create_one_pub(self) -> Publisher:
        self.get_logger().info('Create a new publisher.')
        pub = self.create_publisher(String, self.__topic_name, 10)
        self.__pubs.append(pub)
        return pub

    def destroy_one_pub(self, pub: Publisher):

        if pub in self.__pubs:
            self.get_logger().info('Destroy a publisher.')
            self.__pubs.remove(pub)
            self.destroy_publisher(pub)


def main(args=None):
    rclpy.init(args=args)

    topic_name_for_detect_pub_matched_event = 'pub_topic_matched_event_detect'
    topic_name_for_detect_sub_matched_event = 'sub_topic_matched_event_detect'

    matched_event_detect_node = MatchedEventDetectNode(
        topic_name_for_detect_pub_matched_event, topic_name_for_detect_sub_matched_event)
    multi_subs_node = MultiSubNode(topic_name_for_detect_pub_matched_event)
    multi_pubs_node = MultiPubNode(topic_name_for_detect_sub_matched_event)

    maximum_wait_time = 10  # 10s

    executor = SingleThreadedExecutor()

    executor.add_node(matched_event_detect_node)
    executor.add_node(multi_subs_node)
    executor.add_node(multi_pubs_node)

    # MatchedEventDetectNode will output:
    # First subscription is connected.
    sub1 = multi_subs_node.create_one_sub()
    executor.spin_until_future_complete(matched_event_detect_node.get_future(), maximum_wait_time)

    # MatchedEventDetectNode will output:
    # The changed number of connected subscription is 1 and current number of connected
    # subscription is 2.
    sub2 = multi_subs_node.create_one_sub()
    executor.spin_until_future_complete(matched_event_detect_node.get_future(), maximum_wait_time)

    # MatchedEventDetectNode will output:
    # The changed number of connected subscription is -1 and current number of connected
    # subscription is 1.
    multi_subs_node.destroy_one_sub(sub1)
    executor.spin_until_future_complete(matched_event_detect_node.get_future(), maximum_wait_time)

    # MatchedEventDetectNode will output:
    # Last subscription is disconnected.
    multi_subs_node.destroy_one_sub(sub2)
    executor.spin_until_future_complete(matched_event_detect_node.get_future(), maximum_wait_time)

    # MatchedEventDetectNode will output:
    # First publisher is connected.
    pub1 = multi_pubs_node.create_one_pub()
    executor.spin_until_future_complete(matched_event_detect_node.get_future(), maximum_wait_time)

    # MatchedEventDetectNode will output:
    # The changed number of connected publisher is 1 and current number of connected publisher
    # is 2.
    pub2 = multi_pubs_node.create_one_pub()
    executor.spin_until_future_complete(matched_event_detect_node.get_future(), maximum_wait_time)

    # MatchedEventDetectNode will output:
    # The changed number of connected publisher is -1 and current number of connected publisher
    # is 1.
    multi_pubs_node.destroy_one_pub(pub1)
    executor.spin_until_future_complete(matched_event_detect_node.get_future(), maximum_wait_time)

    # MatchedEventDetectNode will output:
    # Last publisher is disconnected.
    multi_pubs_node.destroy_one_pub(pub2)
    executor.spin_until_future_complete(matched_event_detect_node.get_future(), maximum_wait_time)

    multi_pubs_node.destroy_node()
    multi_subs_node.destroy_node()
    matched_event_detect_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
